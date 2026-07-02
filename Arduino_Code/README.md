Arduino source code will be uploaded here.

// LPG Leakage Detection & Fire Management System
// Hardware: MQ-2 (analog), Servo, MOSFET, Buzzer, Reset button
// For Arduino Uno
// Author: example sketch - adapt thresholds & wiring before real deployment
// Safety: This is educational example code. Test carefully with safe hardware setups.

#include <Servo.h>

// ---------- User-configurable pins ----------
const int mq2Pin         = A0;   // MQ-2 analog output
const int servoPin       = 3;    // Servo signal pin
const int mosfetPin      = 7;    // MOSFET gate pin (drives extinguisher/relay)
const int buzzerPin      = 8;    // Buzzer pin (active buzzer or tone)
const int resetButtonPin = 2;    // Manual reset button (use INPUT_PULLUP)

// ---------- Thresholds & timing (tune for your environment) ----------
const unsigned long baselineSampleTime = 3000UL; // ms to sample baseline on startup
const int baselineSamples = 30;                  // number of readings during baseline
const int triggeredThreshold = 10;  // MQ-2 delta above baseline to consider "leak" (tune)
const unsigned long alarmDebounceMs = 1000UL;    // require sustained over-threshold for this ms

// Extinguish & servo behaviour
const unsigned long extinguishDelay    = 5000UL; // ms after alarm start to trigger extinguisher
const unsigned long extinguishDuration = 10000UL; // ms to keep MOSFET on to activate extinguisher
const int VALVE_OPEN_POS  = 90;   // servo position representing valve open (tweak)
const int VALVE_CLOSE_POS = 0;    // servo position representing valve closed (tweak)

// Buzzer pattern
const unsigned long buzzerOnMs  = 200UL;
const unsigned long buzzerOffMs = 200UL;

// ---------- Internal state ----------
Servo valveServo;

int baseline = 0;
float smoothedReading = 0.0;

enum SystemState { STATE_MONITOR, STATE_ALARM, STATE_EXTINGUISH, STATE_LOCKED };
SystemState state = STATE_MONITOR;

unsigned long alarmStartTime = 0;
unsigned long lastOverThresholdTime = 0;
unsigned long lastBuzzerToggle = 0;
bool buzzerState = false;
unsigned long extinguishStartTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(mosfetPin, OUTPUT);
  digitalWrite(mosfetPin, LOW);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  pinMode(resetButtonPin, INPUT_PULLUP);

  valveServo.attach(servoPin);
  valveServo.write(VALVE_OPEN_POS); // safe default (open) at startup

  // Calibrate baseline
  Serial.println(F("Calibrating baseline... keep environment normal."));
  baseline = measureBaseline();
  smoothedReading = baseline;
  Serial.print(F("Baseline calibrated: "));
  Serial.println(baseline);
  delay(200);
}

void loop() {
  // Read MQ-2 and smooth
  int raw = analogRead(mq2Pin);
  smoothedReading = smoothedReading * 0.9 + raw * 0.1; // simple EMA smoothing
  // Debug
  //Serial.print("Raw: "); Serial.print(raw); Serial.print(" Smooth: "); Serial.println(smoothedReading);

  // Manual reset (button pressed => LOW)
  if (digitalRead(resetButtonPin) == LOW) {
    doReset();
    delay(250); // basic debounce
    return;
  }

  switch (state) {
    case STATE_MONITOR:
      monitorState();
      break;
    case STATE_ALARM:
      alarmState();
      break;
    case STATE_EXTINGUISH:
      extinguishState();
      break;
    case STATE_LOCKED:
      // Locked until manual reset. Keep outputs safe
      digitalWrite(mosfetPin, LOW);
      buzzerOff();
      break;
  }

  // Small loop delay - keep it responsive but not too busy
  delay(50);
}

// ---------- STATES ----------
void monitorState() {
  int current = (int)round(smoothedReading);
  if (current > baseline + triggeredThreshold) {
    // If newly over threshold, remember time
    if (lastOverThresholdTime == 0) lastOverThresholdTime = millis();
    // If sustained long enough, trigger alarm
    if (millis() - lastOverThresholdTime >= alarmDebounceMs) {
      Serial.println(F("ALARM: Gas level exceeded threshold."));
      alarmStartTime = millis();
      state = STATE_ALARM;
      // Actions on entering alarm
      valveServo.write(VALVE_CLOSE_POS); // try to close gas valve
      lastBuzzerToggle = millis();
      buzzerState = true;
      buzzerOn();
    }
  } else {
    lastOverThresholdTime = 0;
  }
}

void alarmState() {
  // buzzer beeping pattern (non-blocking)
  if (buzzerState && millis() - lastBuzzerToggle >= buzzerOnMs) {
    buzzerOff();
    buzzerState = false;
    lastBuzzerToggle = millis();
  } else if (!buzzerState && millis() - lastBuzzerToggle >= buzzerOffMs) {
    buzzerOn();
    buzzerState = true;
    lastBuzzerToggle = millis();
  }

  // After delay, trigger extinguisher
  if (millis() - alarmStartTime >= extinguishDelay) {
    Serial.println(F("Triggering extinguisher (MOSFET ON)"));
    digitalWrite(mosfetPin, HIGH);
    extinguishStartTime = millis();
    state = STATE_EXTINGUISH;
  }
}

void extinguishState() {
  // Keep buzzer on steady during extinguish (optional) or continue pattern.
  // We'll keep beeping rapidly:
  if (millis() - lastBuzzerToggle >= 150) {
    if (buzzerState) buzzerOff();
    else buzzerOn();
    buzzerState = !buzzerState;
    lastBuzzerToggle = millis();
  }

  // Stop extinguish after duration
  if (millis() - extinguishStartTime >= extinguishDuration) {
    digitalWrite(mosfetPin, LOW);
    buzzerOff();
    Serial.println(F("Extinguish cycle complete. System locked until manual reset."));
    // Move servo back to safe position if desired (leave closed for safety)
    valveServo.write(VALVE_CLOSE_POS);
    state = STATE_LOCKED;
  }
}

// ---------- Utilities ----------
int measureBaseline() {
  long sum = 0;
  int count = baselineSamples;
  unsigned long start = millis();
  for (int i = 0; i < count; ++i) {
    int v = analogRead(mq2Pin);
    sum += v;
    delay(baselineSampleTime / count); // spread sampling over baselineSampleTime
  }
  int base = (int)(sum / count);
  return base;
}

void doReset() {
  Serial.println(F("Manual reset pressed: returning to MONITOR."));
  digitalWrite(mosfetPin, LOW);
  buzzerOff();
  valveServo.write(VALVE_OPEN_POS);
  lastOverThresholdTime = 0;
  alarmStartTime = 0;
  extinguishStartTime = 0;
  state = STATE_MONITOR;
}

void buzzerOn() {
  // If you have active buzzer use digital HIGH; else use tone() for passive buzzer
  //digitalWrite(buzzerPin, HIGH);
  tone(buzzerPin, 2000); // play 2kHz tone (works for passive too); tone() is non-blocking
}

void buzzerOff() {
  noTone(buzzerPin);
  digitalWrite(buzzerPin, LOW);
}
