// EMA-smoothed differential monitor for ATmega328P
// Modes:
// 1: Voltage -> display (V(A0)-V(A1)) * divider_ratio * 2, then apply sign-aware offset
// 2: Current -> display ((V(A2)-V(A3)) / shunt_resistor), then apply sign-aware offset
// 3: Resistance -> prints "resistance measurement goes here"
// Button on D8 (wired to GND when pressed) cycles mode 1->2->3->1
// LEDs: D7 = voltage indicator (mode == 1), D6 = current indicator (mode == 2)
// D9 = negative indicator for numeric displayed values

const int pinA0 = A0;
const int pinA1 = A1;
const int pinA2 = A2;
const int pinA3 = A3;

const int btnPin = 8;      // pushbutton, active LOW
const int ledVoltage = 7;  // indicates voltage mode (mode == 1)
const int ledCurrent = 6;  // indicates current mode (mode == 2)
const int ledNegative = 9; // indicates displayed numeric value is negative

const float shunt_resistor = 0.05f; // ohms
const float divider_ratio = 11.0f;  // unchanged

// Sign-aware offset magnitude: applied as +offset for positive measurements,
// -offset for negative measurements
const float offset_magnitude_voltage = 0.3f; // volts
const float offset_magnitude_current = 0.3f; // amps (or same units as current measurement)

const float ADC_REF_V = 5.0f; // ADC reference voltage
const int ADC_MAX = 1023;

int current_mode = 1; // 1 = voltage, 2 = current, 3 = resistance placeholder

// EMA configuration: alpha between 0 (very smooth, slow) and 1 (no smoothing)
const float alpha = 0.2f; // as requested

// EMA state variables (initialized on first loop)
float emaA0 = NAN;
float emaA1 = NAN;
float emaA2 = NAN;
float emaA3 = NAN;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
int lastButtonState = HIGH; // INPUT_PULLUP -> HIGH when unpressed
int stableButtonState = HIGH;

void setup() {
  pinMode(btnPin, INPUT_PULLUP);
  pinMode(ledVoltage, OUTPUT);
  pinMode(ledCurrent, OUTPUT);
  pinMode(ledNegative, OUTPUT);

  digitalWrite(ledVoltage, LOW);
  digitalWrite(ledCurrent, LOW);
  digitalWrite(ledNegative, LOW);

  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("EMA-smoothed Differential Monitor starting...");
  printMode();
}

float rawToVoltage(int raw) {
  return (raw * ADC_REF_V) / (float)ADC_MAX;
}

float readVoltage(int pin) {
  int raw = analogRead(pin);
  return rawToVoltage(raw);
}

// Update EMA: returns new EMA value
float updateEMA(float sample, float prevEMA) {
  if (isnan(prevEMA)) return sample; // initialize EMA on first sample
  return alpha * sample + (1.0f - alpha) * prevEMA;
}

// Apply sign-aware offset: adds offset_magnitude for positive values,
// subtracts offset_magnitude for negative values, leaves zero unchanged.
float applySignAwareOffset(float value, float offsetMagnitude) {
  if (value > 0.0f) return value + offsetMagnitude;
  if (value < 0.0f) return value - offsetMagnitude;
  return value;
}

void loop() {
  handleButton();        // updates current_mode when button pressed
  updateModeLEDs();      // set D7/D6 per mode

  // Read raw voltages
  float v0 = readVoltage(pinA0);
  float v1 = readVoltage(pinA1);
  float v2 = readVoltage(pinA2);
  float v3 = readVoltage(pinA3);

  // Update EMAs
  emaA0 = updateEMA(v0, emaA0);
  emaA1 = updateEMA(v1, emaA1);
  emaA2 = updateEMA(v2, emaA2);
  emaA3 = updateEMA(v3, emaA3);

  bool printedNumeric = false;
  bool valueIsNegative = false;

  if (current_mode == 1) {
    float diff = emaA0 - emaA1;
    float displayValue = diff * divider_ratio * 2.0f; // requested scaling
    displayValue = applySignAwareOffset(displayValue, offset_magnitude_voltage); // symmetric offset
    printedNumeric = true;
    valueIsNegative = (displayValue < 0.0f);
    Serial.print("Mode 1 - Voltage (EMA) | Vdiff = ");
    Serial.print(diff, 6);
    Serial.print(" V  | Scaled+Offset = ");
    Serial.print(displayValue, 6);
    Serial.println(" V");
  }
  else if (current_mode == 2) {
    float diff = emaA2 - emaA3;
    float displayValue = (diff / shunt_resistor);
    displayValue = applySignAwareOffset(displayValue, offset_magnitude_current); // symmetric offset
    printedNumeric = true;
    valueIsNegative = (displayValue < 0.0f);
    Serial.print("Mode 2 - Current (EMA) | Vdiff = ");
    Serial.print(diff, 6);
    Serial.print(" V  | I+Offset = ");
    Serial.print(displayValue, 6);
    Serial.println(" A");
  }
  else { // mode 3 placeholder
    Serial.println("resistance measurement goes here");
    printedNumeric = false;
    valueIsNegative = false;
  }

  // Negative indicator LED for numeric outputs
  if (printedNumeric && valueIsNegative) digitalWrite(ledNegative, HIGH);
  else digitalWrite(ledNegative, LOW);

  delay(120); // tweak between 50-300 ms for responsiveness vs. smoothing
}

void handleButton() {
  int reading = digitalRead(btnPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != stableButtonState) {
      stableButtonState = reading;
      if (stableButtonState == LOW) {
        current_mode++;
        if (current_mode > 3) current_mode = 1;
        Serial.print("Mode changed to ");
        printMode();
      }
    }
  }
  lastButtonState = reading;
}

void updateModeLEDs() {
  if (current_mode == 1) {
    digitalWrite(ledVoltage, HIGH);
    digitalWrite(ledCurrent, LOW);
  } else if (current_mode == 2) {
    digitalWrite(ledVoltage, LOW);
    digitalWrite(ledCurrent, HIGH);
  } else {
    digitalWrite(ledVoltage, HIGH);
    digitalWrite(ledCurrent, HIGH);
  }
}

void printMode() {
  if (current_mode == 1) Serial.println("1 - VOLTAGE");
  else if (current_mode == 2) Serial.println("2 - CURRENT");
  else Serial.println("3 - RESISTANCE (placeholder)");
}