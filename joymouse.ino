/*
 * Joystick BLE/USB Mouse - Prima versione USB (ESP32-S3)
 * ---------------------------------------------------------
 * - Joystick KY-023 (VRx/VRy) => movimento cursore stile TrackPoint
 *   (velocità proporzionale all'inclinazione, non posizione assoluta)
 * - Pulsante sinistro / destro => click sinistro/destro
 * - Pulsante scroll => mentre premuto, il joystick pilota lo scroll
 *   invece del movimento del cursore
 *
 * Richiede: Arduino-ESP32 core 3.x, board ESP32-S3 con USB OTG/HID
 * Tools > USB Mode: "USB-OTG (TinyUSB)"
 */

#include "USB.h"
#include "USBHIDMouse.h"

USBHIDMouse Mouse;

// ---- Pin configuration (adatta ai tuoi collegamenti reali) ----
const int PIN_VRX        = 1;   // asse X joystick (ADC)
const int PIN_VRY        = 2;   // asse Y joystick (ADC)
const int PIN_BTN_LEFT   = 4;   // pulsante click sinistro
const int PIN_BTN_RIGHT  = 5;   // pulsante click destro
const int PIN_BTN_SCROLL = 6;   // pulsante "abilita scroll"

// ---- Parametri di tuning (da regolare provando) ----
const int   DEADZONE       = 100;   // zona morta attorno al centro (su scala 0-4095)
const float CURVE_EXPONENT = 4;   // >1 = più dolce vicino al centro, più veloce agli estremi
const float CURVE_EXPONENT_SCROLL = 4;   // più aggressiva, scroll fine più facile
const int   MAX_SPEED      = 48;    // velocità massima cursore (px per ciclo)
const int   MAX_SCROLL     = 4;     // velocità massima scroll (tick per ciclo)
const int   LOOP_DELAY_MS  = 12;    // ~80Hz update rate
const int   LOOP_DELAY_MS_SCROLL  = 50;    // slower

// ---- Calibrazione centro (letta a boot, joystick a riposo) ----
int centerX = 2048;
int centerY = 2048;

float accumX = 0, accumY = 0, accumScroll =0;


// calibrazione adattativa
int minX = 4095, maxX = 0, minY = 4095, maxY = 0;
int dynamicDeadzoneY = DEADZONE;
int dynamicDeadzoneX = DEADZONE;

void updateRange(int rawX, int rawY) {
  if (rawX < minX) minX = rawX;
  if (rawX > maxX) maxX = rawX;
  if (rawY < minY) minY = rawY;
  if (rawY > maxY) maxY = rawY;

  dynamicDeadzoneY = (int)((maxY - centerY) * 0.1); // 10% del range disponibile
  dynamicDeadzoneX = (int)((maxX - centerX) * 0.1); // 10% del range disponibile

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

// Breakpoints: {normalized_threshold, output_fraction}
// Es: fino al 50% della corsa, usa solo il 10% della velocità max
//     dal 50% all'80%, sali gradualmente al 40%
//     dall'80% al 100%, sali rapidamente al 100%

float piecewiseCurve(float normalized) {
  // Punti di controllo (x = posizione 0-1, y = output 0-1)
  const float x1 = 0.5, y1 = 0.05;
  const float x2 = 0.8, y2 = 0.10;
  const float x3 = 1.0, y3 = 1.00;

  if (normalized <= x1) {
    // Tratto 1: da (0,0) a (x1,y1)
    return (normalized / x1) * y1;
  } else if (normalized <= x2) {
    // Tratto 2: da (x1,y1) a (x2,y2)
    float t = (normalized - x1) / (x2 - x1);
    return y1 + t * (y2 - y1);
  } else {
    // Tratto 3: da (x2,y2) a (x3,y3)
    float t = (normalized - x2) / (x3 - x2);
    return y2 + t * (y3 - y2);
  }
}


// Applica deadzone e curva non lineare, ritorna valore -maxOut..+maxOut
int applyCurveAccumulated(int raw, int center, int rangeMin, int rangeMax, 
                            int maxOut, float curveExponent, int axis, float &accum) {
  int delta = raw - center;
  int deadzone = (axis == 0) ? dynamicDeadzoneX : dynamicDeadzoneY;

  if (abs(delta) < deadzone) {
    accum = 0;
    return 0;
  }

  int sign = (delta > 0) ? 1 : -1;
  int magnitude = abs(delta) - deadzone;

  int maxRange = (sign > 0) ? (rangeMax - center) : (center - rangeMin);
  maxRange -= deadzone;
  if (maxRange <= 0) maxRange = 1;

  float normalized = (float)magnitude / (float)maxRange;
  if (normalized > 1.0) normalized = 1.0;

  float curved = piecewiseCurve(normalized);
  float speed = sign * curved * maxOut; // valore continuo, non troncato

  accum += speed;
  int output = (int)accum; // parte intera da inviare ora
  accum -= output;         // tieni il resto per il prossimo ciclo

  return output;
}

void setup() {
  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BTN_SCROLL, INPUT_PULLUP);

  analogReadResolution(12); // 0-4095

  calibrateCenter();

  Mouse.begin();
  USB.begin();
}

// Stato precedente dei pulsanti, per evitare click ripetuti ad ogni loop
bool lastLeft = false;
bool lastRight = false;

void loop() {
  //int rawX = (centerX * 2 - analogRead(PIN_VRX)) ;
  //int rawY = (centerY * 2 - analogRead(PIN_VRY)) ;

  int rawX = analogRead(PIN_VRX) ;
  int rawY = analogRead(PIN_VRY) ;

  updateRange(rawX, rawY);

  bool scrollMode = (digitalRead(PIN_BTN_SCROLL) == LOW);
  bool leftPressed = (digitalRead(PIN_BTN_LEFT) == LOW);
  bool rightPressed = (digitalRead(PIN_BTN_RIGHT) == LOW);

  if (scrollMode) {
    int scrollY = applyCurveAccumulated(rawY, centerY, minY, maxY, MAX_SCROLL, CURVE_EXPONENT_SCROLL, 1, accumScroll);
    if (scrollY != 0) {
      Mouse.move(0, 0, scrollY);
    }
  } else {
    int moveX = applyCurveAccumulated(rawX, centerX, minX, maxX, MAX_SPEED, CURVE_EXPONENT, 0, accumX);
    int moveY = applyCurveAccumulated(rawY, centerY, minY, maxY, MAX_SPEED, CURVE_EXPONENT, 1, accumY);
    if (moveX != 0 || moveY != 0) {
      Mouse.move(moveX, moveY);
    }
  }

  // Click sinistro/destro con press/release espliciti
  // (utile se in futuro vuoi drag, non solo click istantaneo)
  if (leftPressed && !lastLeft) Mouse.press(MOUSE_LEFT);
  if (!leftPressed && lastLeft) Mouse.release(MOUSE_LEFT);
  lastLeft = leftPressed;

  if (rightPressed && !lastRight) Mouse.press(MOUSE_RIGHT);
  if (!rightPressed && lastRight) Mouse.release(MOUSE_RIGHT);
  lastRight = rightPressed;

  if (scrollMode) {
    delay(LOOP_DELAY_MS_SCROLL);
  } else {
    delay(LOOP_DELAY_MS);
  }
}