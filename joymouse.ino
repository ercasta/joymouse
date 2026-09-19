/*
 * Joystick USB Mouse - Version 7.2: flick + cruise (AVERAGE, 4-point speed curve) + slow auto-recentering (ESP32-S3)
 * ---------------------------------------------------------------------------
 * Behavior:
 * - Inside the deadzone: idle, zero speed, timer/average reset.
 * - Just outside the deadzone: a timer starts, and the AVERAGE gesture speed
 *   (ADC counts per ms, mean of the samples) seen so far in this "outing"
 *   is tracked.
 *   - After HOLD_TIMEOUT_MS the average is frozen: from then on, if you're
 *     still outside the deadzone (a sustained push, not a flick), the
 *     CRUISE speed stays constant. Its value comes from a 4-point curve
 *     that maps the average flick velocity to a speed in px/s.
 * - Returning to the deadzone resets everything (speed, timer, average).
 * - When the stick stays still near the center for a while, the calibrated
 *   center slowly follows it (absorbs ADC/thermal drift).
 *
 * ADC readings are oversampled (averaged over a few samples) to reduce
 * jitter, since speed is derived directly from the raw ADC delta.
 *
 * Requires: Arduino-ESP32 core 3.x, ESP32-S3 board with native USB OTG/HID
 * Tools > USB Mode: "USB-OTG (TinyUSB)"
 */

#include "USB.h"
#include "USBHIDMouse.h"

USBHIDMouse Mouse;

// ---- Types ----
// Must stay ABOVE the first function: the Arduino IDE auto-generates function
// prototypes there, and they need this type to be already declared.
struct RecenterState {
  int ref = -1;                       // value the stick has been sitting around
  unsigned long stillSinceMs = 0;
};

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

// ---- Tuning: speed curve (average flick velocity -> cruise speed) ----
// 4 points: CURVE_VEL[i] (average velocity of the initial flick, ADC counts/ms)
// gives CURVE_SPEED_*[i] (px/s for the cursor, wheel ticks/s for scroll).
//  - Between points the curve is interpolated as a power law (a straight line
//    on a log-log plot), so it is smooth.
//  - Below the first point it keeps falling (same slope as the first segment).
//  - Above the last point the speed stays at the last value.
//  - Points must be strictly increasing, both in velocity and in speed.
// The velocities below are PLACEHOLDERS: use DEBUG_CURVE to read the average
// velocity of your own slow / medium / fast / very fast flicks and put them here.
const float CURVE_VEL[4]          = {1.0, 7.0, 12.0, 20.0};        // counts/ms
const float CURVE_SPEED_MOVE[4]   = {30.0, 80.0, 200.0, 3000.0};  // px/s
const float CURVE_SPEED_SCROLL[4] = {9.0, 26.0, 62.0, 176.0};     // wheel ticks/s (about the old linear feel)

// Safety cap on what is sent in one cycle (px or wheel ticks per cycle)
const int   MAX_SPEED_CAP_MOVE   = 80;
const int   MAX_SPEED_CAP_SCROLL = 6;

// Prints the average velocity and the resulting speed on Serial, every cycle
// while outside the deadzone. Flick and read the "avg" value.
const bool DEBUG_CURVE = false;

// Time before the average is frozen (start of the constant cruise)
const unsigned long HOLD_TIMEOUT_MS = 100;

const int   LOOP_DELAY_MS        = 12;   // ~80Hz, main loop (movement)
const int   LOOP_DELAY_MS_SCROLL = 36;   // slower update rate while scrolling

const float PRECISION_MODE_INCREMENT = (15.0 * LOOP_DELAY_MS) / 1000.0;   // 15 pixel / sec


// ---- Center calibration (read at boot, joystick at rest) ----
int   centerX = 2048;
int   centerY = 2048;
float centerXf = 2048;   // float versions, so the center can move by fractions of a count
float centerYf = 2048;

// ---- Slow auto-recentering ----
// If the stick stays within RECENTER_STILL_BAND counts of the same value for
// RECENTER_SETTLE_MS, and is close to the current center, the center slowly
// moves toward it. "Close" = within RECENTER_MAX_OFFSET_FACTOR * deadzone, so
// a deliberate push (e.g. holding for cruise) is not absorbed.
const unsigned long RECENTER_SETTLE_MS = 500;
const int   RECENTER_STILL_BAND        = 20;    // counts: "still" means staying within this band
const float RECENTER_MAX_OFFSET_FACTOR = 1.5;   // x deadzone
const float RECENTER_RATE              = 0.01;  // fraction of the offset absorbed per cycle

RecenterState recX, recY;

// rawUnflipped: the ADC reading BEFORE the FLIP_X/FLIP_Y mirroring.
void updateRecenter(RecenterState &st, int rawUnflipped, float &centerF, int &centerInt, int deadzone) {
  unsigned long nowMs = millis();

  if (st.ref < 0 || abs(rawUnflipped - st.ref) > RECENTER_STILL_BAND) {
    st.ref = rawUnflipped;
    st.stillSinceMs = nowMs;
    return;
  }
  if (nowMs - st.stillSinceMs < RECENTER_SETTLE_MS) return;

  float offset = rawUnflipped - centerF;
  if (fabs(offset) > deadzone * RECENTER_MAX_OFFSET_FACTOR) return;

  centerF += offset * RECENTER_RATE;
  centerInt = (int)(centerF + 0.5f);
}

// ---- Adaptive range calibration (min/max observed during use) ----
int minX = 4095, maxX = 0, minY = 4095, maxY = 0;
int dynamicDeadzoneX = 100;
int dynamicDeadzoneY = 100;

// Floor for the deadzone, in ADC counts. Must be above the ADC noise at rest,
// otherwise the stick is seen as "outside the deadzone" even when untouched
// (the adaptive deadzone starts at ~0 until the stick is pushed to full range).
const int MIN_DEADZONE = 60;

void updateRange(int rawX, int rawY) {
  if (rawX < minX) minX = rawX;
  if (rawX > maxX) maxX = rawX;
  if (rawY < minY) minY = rawY;
  if (rawY > maxY) maxY = rawY;

  int rangeX = max(maxX - centerX, centerX - minX);
  int rangeY = max(maxY - centerY, centerY - minY);
  dynamicDeadzoneX = max(MIN_DEADZONE, (int)(rangeX * 0.025));
  dynamicDeadzoneY = max(MIN_DEADZONE, (int)(rangeY * 0.025));
}

void calibrateCenter() {
  long sumX = 0, sumY = 0;
  const int samples = 50;
  for (int i = 0; i < samples; i++) {
    sumX += analogRead(PIN_VRX);
    sumY += analogRead(PIN_VRY);
    delay(5);
  }
  centerXf = (float)sumX / samples;
  centerYf = (float)sumY / samples;
  centerX = (int)(centerXf + 0.5f);
  centerY = (int)(centerYf + 0.5f);
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

// Speed (units per second) for a given |average velocity| (counts/ms),
// read from the 4-point curve CURVE_VEL -> speeds.
float curveSpeed(float v, const float *speeds) {
  if (v <= 0) return 0;
  if (v >= CURVE_VEL[3]) return speeds[3];

  // find the segment [i, i+1] that contains v (below the first point: segment 0, extrapolated)
  int i = 0;
  while (i < 2 && v > CURVE_VEL[i + 1]) i++;

  float slope = logf(speeds[i + 1] / speeds[i]) / logf(CURVE_VEL[i + 1] / CURVE_VEL[i]);
  return speeds[i] * powf(v / CURVE_VEL[i], slope);
}

// Computes the output speed for this cycle (px or scroll ticks).
int computeSpeed(int raw, int &lastRaw, unsigned long &lastMicrosRef,
                  unsigned long &exitDeadzoneTime, float &velocitySum, int &sampleCount,
                  float &cruiseAccum,
                  int center, int deadzone,
                  const float *speedCurve, int maxSpeedCap,
                  bool precisionMode, float precisionModeSpeed) {
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
    cruiseAccum += precisionModeSpeed * sign;
  }

  else {
    // ---- cruise: constant speed, from the initial flick's AVERAGE through the 4-point curve ----
    float avgVelocity = (sampleCount > 0) ? (velocitySum / sampleCount) : 0;

    // The curve gives units per SECOND; convert to units per CYCLE with the real
    // cycle time. The clamp guards against a stale timestamp (e.g. right after
    // switching between scroll and move mode).
    float speedPerSec = curveSpeed(fabsf(avgVelocity), speedCurve);
    float cycleMs = (dtMs > 60.0) ? 60.0 : dtMs;
    float cruiseSpeed = speedPerSec * cycleMs / 1000.0;
    if (cruiseSpeed > maxSpeedCap) cruiseSpeed = maxSpeedCap;

    if (DEBUG_CURVE) Serial.printf("avg=%.2f counts/ms -> %.0f /s\n", avgVelocity, speedPerSec);

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

  // Slow auto-recentering works on the UN-flipped readings
  updateRecenter(recX, rawX, centerXf, centerX, dynamicDeadzoneX);
  updateRecenter(recY, rawY, centerYf, centerY, dynamicDeadzoneY);

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
                                CURVE_SPEED_SCROLL, MAX_SPEED_CAP_SCROLL,
                                precisionMode, PRECISION_MODE_INCREMENT);
    if (scrollY != 0) {
      Mouse.move(0, 0, scrollY);
    }
  } else {
    int moveX = computeSpeed(rawX, lastRawX_move, lastMicrosX_move,
                              exitDeadzoneTimeX_move, velocitySumX_move, sampleCountX_move,
                              cruiseAccumX,
                              centerX, dynamicDeadzoneX,
                              CURVE_SPEED_MOVE, MAX_SPEED_CAP_MOVE,
                              precisionMode, PRECISION_MODE_INCREMENT);
    int moveY = computeSpeed(rawY, lastRawY_move, lastMicrosY_move,
                              exitDeadzoneTimeY_move, velocitySumY_move, sampleCountY_move,
                              cruiseAccumY,
                              centerY, dynamicDeadzoneY,
                              CURVE_SPEED_MOVE, MAX_SPEED_CAP_MOVE,
                              precisionMode, PRECISION_MODE_INCREMENT);
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