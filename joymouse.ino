/*
 * Joystick BLE/USB Mouse - Version 7: flick + cruise (AVERAGE), with ADC oversampling (ESP32-S3)
 * ---------------------------------------------------------------------------
 * Behavior:
 * - Inside the deadzone: idle, zero speed, timer/average reset.
 * - Just outside the deadzone: a timer starts, and the AVERAGE gesture speed
 *   (ADC counts per ms, mean of the samples) seen so far in this "outing"
 *   is tracked.
 *   - After HOLD_TIMEOUT_MS the average is frozen: from then on, if you're
 *     still outside the deadzone (a sustained push, not a flick), the
 *     CRUISE speed stays constant, proportional to the AVERAGE gesture
 *     speed of the initial phase (not the current instantaneous gesture).
 *     A stronger flick gives a faster cruise, but always "small" relative
 *     to the original average (CRUISE_SCALE < 1).
 * - Returning to the deadzone resets everything (speed, timer, average).
 *
 * ADC readings are oversampled (averaged over a few samples) to reduce
 * jitter, since speed is derived directly from the raw ADC delta.
 *
 * Requires: Arduino-ESP32 core 3.x, ESP32-S3 board with native USB OTG/HID
 * Tools > USB Mode: "USB-OTG (TinyUSB)"
 * Tools > USB CDC On Boot: Enabled (handy for Serial Monitor/debugging)
 */

#include "USB.h"
#include "USBHIDMouse.h"

USBHIDMouse Mouse;

// ---- Pin configuration (match your actual wiring) ----
const int PIN_VRX        = 1;   // joystick X axis (ADC)
const int PIN_VRY        = 2;   // joystick Y axis (ADC)
const int PIN_BTN_LEFT   = 4;   // left click button
const int PIN_BTN_RIGHT  = 5;   // right click button
const int PIN_BTN_SCROLL = 6;   // "enable scroll" button
const int PIN_BTN_PRECISION = 7;   // "precision mode" button

// Set to true if a given axis is physically mounted mirrored (e.g. the
// joystick is rotated), so that axis needs to be flipped around the
// calibrated center. Independent flags so you can flip just one, both, or neither.
const bool FLIP_X = true;
const bool FLIP_Y = true;

// ---- ADC oversampling ----
const int ADC_OVERSAMPLE_COUNT = 5; // number of samples averaged per reading

int readAveraged(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return sum / samples;
}

// ---- Tuning: "gesture" phase (first HOLD_TIMEOUT_MS outside the deadzone) ----
const float VELOCITY_TO_SPEED_MOVE   = 2.2;   // scales velocityPerMs -> px/cycle (raised)
const float VELOCITY_TO_SPEED_SCROLL = 0.8;
const int   MAX_SPEED_CAP_MOVE   = 80;        // raised max cursor speed
const int   MAX_SPEED_CAP_SCROLL = 6;
const float VELOCITY_NOISE_FLOOR = 0.15;      // below this, treat as ADC noise, not real movement

// ---- Tuning: "cruise" phase (after HOLD_TIMEOUT_MS, still outside the deadzone) ----
// NOTE: the average velocity is lower than the peak, so these scales were
// raised compared to v6 (was 0.15) to get a similar cruise speed. Tune by feel.
const unsigned long HOLD_TIMEOUT_MS = 100;    // time before the average is frozen (cruise)
const float CRUISE_SCALE_MOVE   = 0.40;       // cruise = average_velocity * this scale
const float CRUISE_SCALE_SCROLL = 0.40;
const int   CRUISE_MIN_SPEED_MOVE   = 1;      // guaranteed minimum cruise speed
const int   CRUISE_MIN_SPEED_SCROLL = 1;


const int   LOOP_DELAY_MS        = 12;   // ~80Hz, main loop (movement)
const int   LOOP_DELAY_MS_SCROLL = 36;   // slower update rate while scrolling

const float PRECISION_MODE_INCREMENT = (15.0 * LOOP_DELAY_MS) / 1000.0;   // 15 pixel / sec


// ---- Center calibration (read at boot, joystick at rest) ----
int centerX = 2048;
int centerY = 2048;

// ---- Adaptive range calibration (min/max observed during use) ----
int minX = 4095, maxX = 0, minY = 4095, maxY = 0;
int dynamicDeadzoneX = 100;
int dynamicDeadzoneY = 100;

void updateRange(int rawX, int rawY) {
  if (rawX < minX) minX = rawX;
  if (rawX > maxX) maxX = rawX;
  if (rawY < minY) minY = rawY;
  if (rawY > maxY) maxY = rawY;

  dynamicDeadzoneX = (int)((maxX - centerX) * 0.025);
  dynamicDeadzoneY = (int)((maxY - centerY) * 0.025);
}

void calibrateCenter() {
  long sumX = 0, sumY = 0;
  const int samples = 50;
  for (int i = 0; i < samples; i++) {
    sumX += analogRead(PIN_VRX);
    sumY += analogRead(PIN_VRY);
    delay(5);
  }
  centerX = sumX / samples;
  centerY = sumY / samples;
}

// ---- State for cursor movement (X, Y) and scroll (Y) ----
// Kept separate so the two contexts don't interfere with each other.
int lastRawX_move = -1, lastRawY_move = -1;
int lastRawY_scroll = -1;

unsigned long lastMicrosX_move = 0, lastMicrosY_move = 0;
unsigned long lastMicrosY_scroll = 0;

// Timestamp of when we exited the deadzone (0 = reset / not yet exited)
unsigned long exitDeadzoneTimeX_move = 0, exitDeadzoneTimeY_move = 0;
unsigned long exitDeadzoneTimeY_scroll = 0;

// Sum of the signed velocityPerMs samples collected during the "gesture"
// phase of this outing, and how many samples were collected.
// average = velocitySum / sampleCount (the sign of the average is the direction).
float velocitySumX_move = 0, velocitySumY_move = 0;
float velocitySumY_scroll = 0;
int   sampleCountX_move = 0, sampleCountY_move = 0;
int   sampleCountY_scroll = 0;

// Fractional remainder for cruise speed accumulation (avoids truncation loss)
float cruiseAccumX = 0, cruiseAccumY = 0, cruiseAccumScroll = 0;

bool lastScrollMode = false; // to detect the transition and reset scroll state

// Computes the output speed for this cycle (px or scroll ticks).
int computeSpeed(int raw, int &lastRaw, unsigned long &lastMicrosRef,
                  unsigned long &exitDeadzoneTime, float &velocitySum, int &sampleCount,
                  float &cruiseAccum,
                  int center, int deadzone,
                  float velocityToSpeed, int maxSpeedCap,
                  float cruiseScale, int cruiseMinSpeed, bool precisionMode, float precisionModeSpeed) {
  unsigned long now = micros();

  if (lastRaw == -1) {
    lastRaw = raw;
    lastMicrosRef = now;
  }

  unsigned long elapsedMicros = now - lastMicrosRef;
  float dtMs = elapsedMicros / 1000.0;
  if (dtMs < 0.5) dtMs = 0.5;

  int rawDelta = raw - lastRaw;
  lastRaw = raw;
  lastMicrosRef = now;

  float velocityPerMs = rawDelta / dtMs;

  int delta = raw - center;

  if (abs(delta) < deadzone) {
    // Inside the deadzone: full reset
    velocitySum = 0;
    sampleCount = 0;
    exitDeadzoneTime = 0;
    cruiseAccum = 0;
    return 0;
  }

  // Outside the deadzone: if this is the first cycle of this outing, mark the timestamp
  if (exitDeadzoneTime == 0) {
    exitDeadzoneTime = now;
    velocitySum = 0;
    sampleCount = 0;
  }

  unsigned long timeOutsideMs = (now - exitDeadzoneTime) / 1000;

  if (timeOutsideMs < HOLD_TIMEOUT_MS) {
    // ---- Gesture phase: accumulate samples for the average ----
    // After HOLD_TIMEOUT_MS the sum and count stay frozen, so the average
    // (and therefore the cruise speed) stays constant.
    velocitySum += velocityPerMs;
    sampleCount++;
  }


  if (precisionMode) {
    // move at fixed speed, direction given by where the stick is relative to center
    int sign = delta > 0 ? 1 : -1;
    Serial.printf("raw=%d center=%d sign=%d\n", raw, center, sign);

    cruiseAccum += precisionModeSpeed * sign;
  }

  else {
    // ---- move at average velocity: constant speed, proportional to the initial flick's AVERAGE ----
    float avgVelocity = (sampleCount > 0) ? (velocitySum / sampleCount) : 0;
    float cruiseSpeed = abs(avgVelocity) * velocityToSpeed * cruiseScale;
    if (cruiseSpeed > maxSpeedCap) cruiseSpeed = maxSpeedCap;
    float speedPerCycle = (avgVelocity >= 0 ? 1 : -1) * cruiseSpeed;
    cruiseAccum += speedPerCycle;
  }

  int output = (int)cruiseAccum;
  cruiseAccum -= output;
  return output;

}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BTN_SCROLL, INPUT_PULLUP);
  pinMode(PIN_BTN_PRECISION, INPUT_PULLUP);

  analogReadResolution(12); // 0-4095

  calibrateCenter();

  Mouse.begin();
  USB.begin();
}

// Previous button state, to avoid repeated clicks every loop
bool lastLeft = false;
bool lastRight = false;

void loop() {
  int rawX = readAveraged(PIN_VRX, ADC_OVERSAMPLE_COUNT);
  int rawY = readAveraged(PIN_VRY, ADC_OVERSAMPLE_COUNT);

  if (FLIP_X) {
    rawX = 2 * centerX - rawX;
  }
  if (FLIP_Y) {
    rawY = 2 * centerY - rawY;
  }

  updateRange(rawX, rawY);

  bool scrollMode = (digitalRead(PIN_BTN_SCROLL) == LOW);
  bool leftPressed = (digitalRead(PIN_BTN_LEFT) == LOW);
  bool rightPressed = (digitalRead(PIN_BTN_RIGHT) == LOW);
  bool precisionMode = (digitalRead(PIN_BTN_PRECISION) == LOW);

  // Clean reset of scroll state when entering/exiting scroll mode
  if (scrollMode != lastScrollMode) {
    lastRawY_scroll = -1;
    exitDeadzoneTimeY_scroll = 0;
    velocitySumY_scroll = 0;
    sampleCountY_scroll = 0;
    cruiseAccumScroll = 0;
  }
  lastScrollMode = scrollMode;

  if (scrollMode) {
    int scrollY = computeSpeed(rawY, lastRawY_scroll, lastMicrosY_scroll,
                                exitDeadzoneTimeY_scroll, velocitySumY_scroll, sampleCountY_scroll,
                                cruiseAccumScroll,
                                centerY, dynamicDeadzoneY,
                                VELOCITY_TO_SPEED_SCROLL, MAX_SPEED_CAP_SCROLL,
                                CRUISE_SCALE_SCROLL, CRUISE_MIN_SPEED_SCROLL, precisionMode, PRECISION_MODE_INCREMENT);
    if (scrollY != 0) {
      Mouse.move(0, 0, scrollY);
    }
  } else {
    int moveX = computeSpeed(rawX, lastRawX_move, lastMicrosX_move,
                              exitDeadzoneTimeX_move, velocitySumX_move, sampleCountX_move,
                              cruiseAccumX,
                              centerX, dynamicDeadzoneX,
                              VELOCITY_TO_SPEED_MOVE, MAX_SPEED_CAP_MOVE,
                              CRUISE_SCALE_MOVE, CRUISE_MIN_SPEED_MOVE, precisionMode, PRECISION_MODE_INCREMENT);
    int moveY = computeSpeed(rawY, lastRawY_move, lastMicrosY_move,
                              exitDeadzoneTimeY_move, velocitySumY_move, sampleCountY_move,
                              cruiseAccumY,
                              centerY, dynamicDeadzoneY,
                              VELOCITY_TO_SPEED_MOVE, MAX_SPEED_CAP_MOVE,
                              CRUISE_SCALE_MOVE, CRUISE_MIN_SPEED_MOVE, precisionMode, PRECISION_MODE_INCREMENT);
    if (moveX != 0 || moveY != 0) {
      Mouse.move(moveX, moveY);
    }
  }

  // Left/right click with explicit press/release
  // (useful if you want drag support later, not just instant click)
  if (leftPressed && !lastLeft) Mouse.press(MOUSE_LEFT);
  if (!leftPressed && lastLeft) Mouse.release(MOUSE_LEFT);
  lastLeft = leftPressed;

  if (rightPressed && !lastRight) Mouse.press(MOUSE_RIGHT);
  if (!rightPressed && lastRight) Mouse.release(MOUSE_RIGHT);
  lastRight = rightPressed;

  delay(scrollMode ? LOOP_DELAY_MS_SCROLL : LOOP_DELAY_MS);
}
