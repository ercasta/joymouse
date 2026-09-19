/*
 * Joystick USB Mouse - Version 9.3: speed proportional to tilt (% of the learned travel) + instant boost when the end of the travel is reached fast + slow auto-recentering (ESP32-S3)
 * ---------------------------------------------------------------------------
 * Behavior:
 * - Inside the deadzone: idle, zero speed.
 * - Outside the deadzone: the speed depends on how far the stick is tilted, as
 *   a PERCENTAGE of its full travel on that side (learned from the min/max seen
 *   since boot), through a gentle curve with several thresholds (interpolated
 *   between them).
 * - Boost: if the stick goes from the deadzone to the END of its travel in less
 *   than BOOST_REACH_MS, the speed goes to a top speed right away (over
 *   BOOST_RAMP_MS). It lasts while the stick stays near the end and stops as soon
 *   as it comes back. Reaching the end more slowly never boosts.
 * - Precision mode (button held): same behavior, but the resulting speed
 *   (cursor and scroll) is multiplied by PRECISION_SCALE.
 * - Scroll mode (button held): the Y axis scrolls instead of moving the cursor.
 * - When the stick stays still near the center for a while, the calibrated
 *   center slowly follows it (absorbs ADC/thermal drift).
 *
 * ADC readings are oversampled (averaged over a few samples) to reduce jitter.
 *
 * Requires: Arduino-ESP32 core 3.x, ESP32-S3 board with native USB OTG/HID
 * Tools > USB Mode: "USB-OTG (TinyUSB)"
 */

#include "USB.h"
#include "USBHIDMouse.h"

USBHIDMouse Mouse;

// ---- Types ----
// Must stay ABOVE the first function: the Arduino IDE auto-generates function
// prototypes there, and they need these types to be already declared.
struct RecenterState {
  int ref = -1;                       // value the stick has been sitting around
  unsigned long stillSinceMs = 0;
};

// State of one output (X move, Y move, Y scroll). AxisState() = everything reset.
struct AxisState {
  float accum = 0;        // fractional remainder of the speed (avoids losing sub-pixel movement)
  float pushMs = 0;       // time since the stick left the deadzone (this push)
  float boostMs = -1;     // time since the boost started (-1 = no boost)
  bool  endSeen = false;  // the end of the travel was already reached during this push
  int   side = 0;         // side of this push: -1, +1 (0 = idle)
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

// ---- Tuning: speed curve (tilt -> speed) ----
// The tilt is a fraction (0..1) of the usable travel on that side of the center:
// 0 = edge of the deadzone, 1 = end of the stick's travel.
// CURVE_N thresholds: CURVE_TILT[i] (fraction of the travel) gives CURVE_SPEED_*[i]
// (px/s for the cursor, wheel ticks/s for scroll).
//  - Between thresholds the curve is interpolated as a power law (a straight
//    line on a log-log plot), so it is smooth.
//  - From the deadzone edge up to the first threshold the speed rises smoothly
//    from zero (same slope as the first segment).
//  - Beyond the last threshold the speed stays at the last value.
//  - Thresholds must be strictly increasing, both in tilt and in speed.
//  - To add or remove a threshold, change CURVE_N and the arrays together.
const int   CURVE_N = 4;
const float CURVE_TILT[CURVE_N]         = {0.15, 0.40, 0.70, 0.93};    // fraction of the travel
const float CURVE_SPEED_MOVE[CURVE_N]   = {40.0, 120.0, 300.0, 600.0}; // px/s
const float CURVE_SPEED_SCROLL[CURVE_N] = {4.0, 12.0, 30.0, 60.0};     // wheel ticks/s

// ---- Tuning: boost (top speed) ----
// The boost is a deliberate gesture: if the stick goes from the deadzone to the
// END of its travel (tilt >= BOOST_TILT) in less than BOOST_REACH_MS, the speed
// goes to BOOST_SPEED_* right away (over BOOST_RAMP_MS; 0 = instantly).
// Reaching the end more slowly gives no boost, only the normal curve speed.
// The boost stays on while the stick stays near the end
// (tilt >= BOOST_TILT - BOOST_TILT_HYST) and stops as soon as it comes back below.
const float BOOST_TILT       = 0.90;    // fraction of the travel
const float BOOST_TILT_HYST  = 0.08;
const float BOOST_REACH_MS   = 30.0;   // max time from leaving the deadzone to the end of the travel
const float BOOST_RAMP_MS    = 0.0;     // time to go from the curve value to the top speed (0 = instantly)
const float BOOST_SPEED_MOVE   = 3000.0;   // px/s
const float BOOST_SPEED_SCROLL = 176.0;    // wheel ticks/s

// The full travel on each side of the center is learned from the min/max seen
// since boot (minX, maxX, minY, maxY): push the stick to the end once and it is
// known. Until then, the travel is assumed to be at least RANGE_FLOOR counts
// from the center. Set it a bit BELOW the smallest maximum you measure with the
// range diagnostic sketch: too high and the end of the travel is never reached, too
// low and the boost can start too early until the first full push.
const int RANGE_FLOOR = 1850;   // smallest maximum measured on this joystick: 1873

// Safety cap on what is sent in one cycle (px or wheel ticks per cycle)
const int   MAX_SPEED_CAP_MOVE   = 80;
const int   MAX_SPEED_CAP_SCROLL = 6;

// Prints the tilt (percentage of the travel), the time since the push started,
// whether the boost is on, and the resulting speed on Serial, every cycle while
// outside the deadzone.
const bool DEBUG_CURVE = false;

const int   LOOP_DELAY_MS        = 12;   // ~80Hz, main loop (movement)
const int   LOOP_DELAY_MS_SCROLL = 36;   // slower update rate while scrolling

// Precision mode: while the button is held, the speed from the curve is multiplied by this
const float PRECISION_SCALE = 0.1;


// ---- Center calibration (read at boot, joystick at rest) ----
int   centerX = 2048;
int   centerY = 2048;
float centerXf = 2048;   // float versions, so the center can move by fractions of a count
float centerYf = 2048;

// ---- Slow auto-recentering ----
// If the stick stays within RECENTER_STILL_BAND counts of the same value for
// RECENTER_SETTLE_MS, and is close to the current center, the center slowly
// moves toward it. "Close" = within RECENTER_MAX_OFFSET_FACTOR * deadzone, so
// a deliberate push is not absorbed.
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

// ---- Output state ----
AxisState axisX, axisY, axisScroll;

bool lastScrollMode = false;     // to detect the transition and reset the states
unsigned long lastLoopUs = 0;    // to measure the real cycle time

// Speed (units per second) for a given tilt (fraction of the travel, 0..1),
// read from the CURVE_N-threshold curve CURVE_TILT -> speeds. Beyond the last
// threshold it stays at the last speed.
float curveSpeed(float x, const float *speeds) {
  if (x <= 0) return 0;
  if (x >= CURVE_TILT[CURVE_N - 1]) return speeds[CURVE_N - 1];

  // find the segment [i, i+1] that contains x (below the first threshold: segment 0, extrapolated)
  int i = 0;
  while (i < CURVE_N - 2 && x > CURVE_TILT[i + 1]) i++;

  float slope = logf(speeds[i + 1] / speeds[i]) / logf(CURVE_TILT[i + 1] / CURVE_TILT[i]);
  return speeds[i] * powf(x / CURVE_TILT[i], slope);
}

// Computes what to send this cycle (px or scroll ticks), from the tilt of one axis.
// rangePos / rangeNeg: full travel of the stick on each side, in counts from the center.
int computeSpeed(int raw, AxisState &st, int center, int deadzone, int rangePos, int rangeNeg,
                 const float *speedCurve, float topSpeed, int maxSpeedCap,
                 float dtSec, bool precisionMode) {
  int delta = raw - center;

  if (abs(delta) < deadzone) {
    // Inside the deadzone: stop and reset everything
    st = AxisState();
    return 0;
  }

  // A new push starts when the stick leaves the deadzone, or when it crosses to the other side
  int side = (delta > 0) ? 1 : -1;
  if (side != st.side) {
    st.side = side;
    st.pushMs = 0;
    st.boostMs = -1;
    st.endSeen = false;
  }
  st.pushMs += dtSec * 1000.0;

  int range = (delta > 0) ? rangePos : rangeNeg;   // full travel on the side we are tilting to
  float travel = range - deadzone;                 // usable travel beyond the deadzone
  if (travel < 1) travel = 1;
  float tilt = (abs(delta) - deadzone) / travel;   // fraction of the travel: 0..1

  // Boost trigger: the FIRST time the end of the travel is reached in this push,
  // it counts only if it happened fast enough
  if (!st.endSeen && tilt >= BOOST_TILT) {
    st.endSeen = true;
    if (st.pushMs <= BOOST_REACH_MS) st.boostMs = 0;
  }
  // The boost lasts while the stick stays near the end
  if (st.boostMs >= 0) {
    if (tilt >= BOOST_TILT - BOOST_TILT_HYST) st.boostMs += dtSec * 1000.0;
    else st.boostMs = -1;
  }

  float speedPerSec = curveSpeed(tilt, speedCurve);

  // Boost on: go (with an optional ramp) from the curve speed to the top speed
  if (st.boostMs >= 0 && speedPerSec > 0 && topSpeed > speedPerSec) {
    float k = (BOOST_RAMP_MS > 0) ? st.boostMs / BOOST_RAMP_MS : 1;
    if (k > 1) k = 1;
    speedPerSec *= powf(topSpeed / speedPerSec, k);   // geometric ramp: from speedPerSec (k=0) to topSpeed (k=1)
  }
  if (precisionMode) speedPerSec *= PRECISION_SCALE;

  float speedPerCycle = speedPerSec * dtSec;       // units per second -> units per cycle
  if (speedPerCycle > maxSpeedCap) speedPerCycle = maxSpeedCap;

  if (DEBUG_CURVE) Serial.printf("tilt=%d%% push=%dms boost=%d -> %.0f /s\n",
                                 (int)(tilt * 100), (int)st.pushMs, st.boostMs >= 0 ? 1 : 0, speedPerSec);

  st.accum += (delta > 0 ? 1 : -1) * speedPerCycle;
  int output = (int)st.accum;
  st.accum -= output;
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

  lastLoopUs = micros();
}

// Previous button state, to avoid repeated clicks every loop
bool lastLeft = false;
bool lastRight = false;

void loop() {
  // Real time since the previous cycle, used to turn px/s into px per cycle.
  // The clamp guards against a stale timestamp.
  unsigned long nowUs = micros();
  float dtSec = (nowUs - lastLoopUs) / 1000000.0;
  lastLoopUs = nowUs;
  if (dtSec > 0.06) dtSec = 0.06;

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

  // Full travel on each side of the center, in counts (never below RANGE_FLOOR)
  int rangePosX = max(maxX - centerX, RANGE_FLOOR);
  int rangeNegX = max(centerX - minX, RANGE_FLOOR);
  int rangePosY = max(maxY - centerY, RANGE_FLOOR);
  int rangeNegY = max(centerY - minY, RANGE_FLOOR);

  bool scrollMode = (digitalRead(PIN_BTN_SCROLL) == LOW);
  bool leftPressed = (digitalRead(PIN_BTN_LEFT) == LOW);
  bool rightPressed = (digitalRead(PIN_BTN_RIGHT) == LOW);
  bool precisionMode = (digitalRead(PIN_BTN_PRECISION) == LOW);

  // Clean reset of the states when entering/exiting scroll mode
  if (scrollMode != lastScrollMode) {
    axisX = AxisState();
    axisY = AxisState();
    axisScroll = AxisState();
  }
  lastScrollMode = scrollMode;

  if (scrollMode) {
    int scrollY = computeSpeed(rawY, axisScroll, centerY, dynamicDeadzoneY, rangePosY, rangeNegY,
                               CURVE_SPEED_SCROLL, BOOST_SPEED_SCROLL, MAX_SPEED_CAP_SCROLL,
                               dtSec, precisionMode);
    if (scrollY != 0) {
      Mouse.move(0, 0, scrollY);
    }
  } else {
    int moveX = computeSpeed(rawX, axisX, centerX, dynamicDeadzoneX, rangePosX, rangeNegX,
                             CURVE_SPEED_MOVE, BOOST_SPEED_MOVE, MAX_SPEED_CAP_MOVE,
                             dtSec, precisionMode);
    int moveY = computeSpeed(rawY, axisY, centerY, dynamicDeadzoneY, rangePosY, rangeNegY,
                             CURVE_SPEED_MOVE, BOOST_SPEED_MOVE, MAX_SPEED_CAP_MOVE,
                             dtSec, precisionMode);
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