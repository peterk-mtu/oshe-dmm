#include <Wire.h>
#include <Arduino.h>

// ---------- BU91796 LCD driver (I2C) ----------

#define BU91796_I2C_ADDR 0x3E
#define DDRAM_SIZE 20

const uint8_t CMD_ICSET_SWRESET = 0xEA;
const uint8_t CMD_BLKCTL        = 0xF0;
const uint8_t CMD_DISCTL_ON     = 0xBF;
const uint8_t CMD_APCTL         = 0xFC;
const uint8_t CMD_MODESET_ON    = 0xC8;

static inline uint8_t ADSET(uint8_t addr) { return (addr & 0x7F); }

uint8_t ddram[DDRAM_SIZE];

struct SegMap { uint8_t addr; uint8_t mask; };

// Segmap taken from your provided mapping
SegMap digitSegments[4][8] = {
  { {0x0A,0x01}, {0x0A,0x02}, {0x0A,0x04}, {0x1A,0x51},
    {0x1A,0x42}, {0x1A,0x16}, {0x1A,0x20}, {0x0A,0x08} },
  { {0x0B,0x01}, {0x0B,0x02}, {0x0B,0x04}, {0x0B,0x42},
    {0x0B,0x40}, {0x0B,0x16}, {0x0B,0x20}, {0x0B,0x08} },
  { {0x0C,0x01}, {0x0C,0x02}, {0x0C,0x04}, {0x0C,0x42},
    {0x0C,0x40}, {0x0C,0x16}, {0x0C,0x20}, {0x0C,0x08} },
  { {0x0D,0x01}, {0x0D,0x02}, {0x0D,0x04}, {0x0D,0x42},
    {0x0D,0x40}, {0x0D,0x16}, {0x0D,0x20}, {0x0D,0x08} }
};

// 7-segment patterns for digits 0..9 (a..g)
const bool segPattern[10][7] = {
  {1,1,1,1,1,1,0}, {0,1,1,0,0,0,0}, {1,1,0,1,1,0,1},
  {1,1,1,1,0,0,1}, {0,1,1,0,0,1,1}, {1,0,1,1,0,1,1},
  {1,0,1,1,1,1,1}, {1,1,1,0,0,0,0}, {1,1,1,1,1,1,1},
  {1,1,1,1,0,1,1}
};

void writeCommandByte(uint8_t cmd) {
  Wire.beginTransmission(BU91796_I2C_ADDR);
  Wire.write(cmd);
  Wire.endTransmission();
  delay(1);
}

void writeDDRAM(uint8_t addr, const uint8_t *buf, size_t len) {
  Wire.beginTransmission(BU91796_I2C_ADDR);
  Wire.write(ADSET(addr));
  for (size_t i = 0; i < len; ++i) Wire.write(buf[i]);
  Wire.endTransmission();
  delay(1);
}

void bu91796_init() {
  delay(5);
  writeCommandByte(CMD_ICSET_SWRESET);
  delay(5);
  writeCommandByte(CMD_BLKCTL);
  writeCommandByte(CMD_DISCTL_ON);
  writeCommandByte(CMD_APCTL);
  writeCommandByte(CMD_MODESET_ON);
  memset(ddram, 0x00, DDRAM_SIZE);
  writeDDRAM(0x00, ddram, DDRAM_SIZE);
}

// Set or clear a mapped segment for a digit (digitIndex 0..3, segIndex 0..7 where 7 is DP)
void setSegmentLocal(int digitIndex, int segIndex, bool on) {
  if (digitIndex < 0 || digitIndex >= 4) return;
  SegMap m = digitSegments[digitIndex][segIndex];
  if (m.addr >= DDRAM_SIZE || m.mask == 0x00) return;
  if (on) ddram[m.addr] |= m.mask;
  else    ddram[m.addr] &= ~m.mask;
}

void updateDisplayFromBuffer() {
  writeDDRAM(0x00, ddram, DDRAM_SIZE);
}

// Display four characters (digits, '.' for DP, space for blank, '-' for dash)
void displayStringFour(const char *s) {
  memset(ddram, 0x00, DDRAM_SIZE);
  int digit = 0;
  for (size_t i = 0; s[i] != '\0' && digit < 4; ++i) {
    char c = s[i];
    if (c == '.') {
      if (digit > 0) setSegmentLocal(digit-1, 7, true); // DP tied to previous digit
    } else if (c >= '0' && c <= '9') {
      int val = c - '0';
      for (int si = 0; si < 7; ++si) setSegmentLocal(digit, si, segPattern[val][si]);
      digit++;
    } else if (c == ' ') {
      digit++;
    } else if (c == '-') {
      // Use segment 'd' (index 3) to approximate a minus sign
      setSegmentLocal(digit, 3, true);
      digit++;
    }
  }
  updateDisplayFromBuffer();
}

// ---------- Measurement, EMA and alpha-beta filters ----------

const int pinA0 = A0;
const int pinA1 = A1;
const int pinA2 = A2;
const int pinA3 = A3;

const int btnPin = 8;      // pushbutton, active LOW
const int ledVoltage = 7;  // indicates voltage mode (mode == 1)
const int ledCurrent = 6;  // indicates current mode (mode == 2)
const int ledNegative = 9; // indicates displayed numeric value is negative

const float shunt_resistor = 10.0f; // ohms
const float divider_ratio = 10.2f;

const float offset_magnitude_voltage = 0.0f; // volts
const float offset_magnitude_current = -0.025f; // amps

const float ADC_REF_V = 5.0f; // ADC reference
const int ADC_MAX = 1023;

int current_mode = 1; // 1 = voltage, 2 = current

// EMA
const float alpha = 0.05f;
float emaA0 = NAN;
float emaA1 = NAN;
float emaA2 = NAN;
float emaA3 = NAN;

// Debounce
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
int lastButtonState = HIGH;
int stableButtonState = HIGH;

// alpha-beta filter state for current
const float abc_alpha = 0.6f;
const float abc_beta  = 0.2f;
float abc_est = 0.0f;
float abc_rate = 0.0f;
bool abc_initialized = false;
unsigned long abc_lastTime = 0;

// alpha-beta filter state for voltage
const float abv_alpha = 0.6f;
const float abv_beta  = 0.2f;
float abv_est = 0.0f;
float abv_rate = 0.0f;
bool abv_initialized = false;
unsigned long abv_lastTime = 0;

float rawToVoltage(int raw) {
  return (raw * ADC_REF_V) / (float)ADC_MAX;
}

float readVoltagePin(int pin) {
  int raw = analogRead(pin);
  return rawToVoltage(raw);
}

float updateEMA(float sample, float prevEMA) {
  if (isnan(prevEMA)) return sample;
  return alpha * sample + (1.0f - alpha) * prevEMA;
}

float applySignAwareOffset(float value, float offsetMagnitude) {
  if (value > 0.1f) return value + offsetMagnitude;
  if (value < -0.1f) return value - offsetMagnitude;
  return value;
}

float updateAlphaBetaGeneric(float measurement, float &est, float &rate, bool &initialized, unsigned long &lastTime, float gain_alpha, float gain_beta) {
  unsigned long now = millis();
  if (!initialized) {
    est = measurement;
    rate = 0.0f;
    initialized = true;
    lastTime = now;
    return est;
  }
  float dt = (now - lastTime) * 0.001f;
  if (dt <= 0.0f) return est;
  float pred = est + rate * dt;
  float resid = measurement - pred;
  est  = pred + gain_alpha * resid;
  rate = rate + (gain_beta * resid) / dt;
  lastTime = now;
  return est;
}

// ---------- Formatting and display of absolute XX.XX ----------

// Build and show absolute value formatted as "II.FF" (two digits before and after decimal).
// Displays leading zero by default (e.g., 3.14 -> "03.14"). To blank the leading zero, uncomment the relevant line.
void displayAbsXXdotXX(float v) {
  if (isnan(v)) v = 0.0f;
  float absVal = fabs(v);
  if (absVal > 99.995f) absVal = 99.995f; // clamp

  int totalCents = (int)roundf(absVal * 100.0f);
  int frac = totalCents % 100;
  int integer = totalCents / 100;

  char out[8];
  int tens = integer / 10;
  int units = integer % 10;
  int tenths = frac / 10;
  int hundredths = frac % 10;

  // Build 4-character string with '.' after digit2 (between digit2 and digit3)
  // Format: D1 D2 . D3 D4  -> e.g. "12.34"
  out[0] = '0' + tens;    // set to '0'..'9'; to blank leading zero, use ' ' instead of '0' when tens==0
  if (tens == 0) out[0] = ' '; // uncomment to blank leading tens
  out[1] = '0' + units;
  out[2] = '.';
  out[3] = '0' + tenths;
  out[4] = '0' + hundredths;
  out[5] = '\0';

  displayStringFour(out);
}

// ---------- Setup / Loop / UI ----------

void setup() {
  pinMode(btnPin, INPUT_PULLUP);
  pinMode(ledVoltage, OUTPUT);
  pinMode(ledCurrent, OUTPUT);
  pinMode(ledNegative, OUTPUT);

  digitalWrite(ledVoltage, LOW);
  digitalWrite(ledCurrent, LOW);
  digitalWrite(ledNegative, LOW);

  Wire.begin();
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println("Integrated EMA + AB + BU91796 display sketch");
  bu91796_init();
  displayStringFour("00.00"); // initial display
}

void handleButton() {
  int reading = digitalRead(btnPin);
  if (reading != lastButtonState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != stableButtonState) {
      stableButtonState = reading;
      if (stableButtonState == LOW) {
        current_mode++;
        if (current_mode > 2) current_mode = 1;
        // Reinitialize AB filter for the mode just entered
        if (current_mode == 1) {
          abv_initialized = false;
          abv_est = 0.0f;
          abv_rate = 0.0f;
          abv_lastTime = millis();
        } else if (current_mode == 2) {
          abc_initialized = false;
          abc_est = 0.0f;
          abc_rate = 0.0f;
          abc_lastTime = millis();
        }
        Serial.print("Mode changed to ");
        if (current_mode == 1) Serial.println("1 - VOLTAGE");
        else Serial.println("2 - CURRENT");
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
  }
}

void loop() {
  handleButton();
  updateModeLEDs();

  // Read ADCs and update EMAs
  float v0 = readVoltagePin(pinA0);
  float v1 = readVoltagePin(pinA1);
  float v2 = readVoltagePin(pinA2);
  float v3 = readVoltagePin(pinA3);

  emaA0 = updateEMA(v0, emaA0);
  emaA1 = updateEMA(v1, emaA1);
  emaA2 = updateEMA(v2, emaA2);
  emaA3 = updateEMA(v3, emaA3);

  bool showNegative = false;

  if (current_mode == 1) {
    float vdiff = emaA0 - emaA1;
    float scaled = vdiff * divider_ratio * 2.0f;
    float measured = applySignAwareOffset(scaled, offset_magnitude_voltage);

    float abv_filtered = updateAlphaBetaGeneric(measured, abv_est, abv_rate, abv_initialized, abv_lastTime, abv_alpha, abv_beta);

    displayAbsXXdotXX(abv_filtered);
    showNegative = (abv_filtered < 0.0f);

    Serial.print("Mode1 Voltage | Vdiff(EMA)=");
    Serial.print(vdiff,6);
    Serial.print(" V | Scaled+Offset=");
    Serial.print(measured,6);
    Serial.print(" V | AB=");
    Serial.print(abv_filtered,6);
    Serial.println(" V");
  } else { // mode == 2
    float vdiff = emaA2 - emaA3;
    float raw_current = (vdiff / shunt_resistor) * 2.0f;
    float measured = applySignAwareOffset(raw_current, offset_magnitude_current);

    float abc_filtered = updateAlphaBetaGeneric(measured, abc_est, abc_rate, abc_initialized, abc_lastTime, abc_alpha, abc_beta);

    displayAbsXXdotXX(abc_filtered);
    showNegative = (abc_filtered < 0.0f);

    Serial.print("Mode2 Current | Vdiff(EMA)=");
    Serial.print(vdiff,6);
    Serial.print(" V | RawI=");
    Serial.print(raw_current,6);
    Serial.print(" A | AB=");
    Serial.print(abc_filtered,6);
    Serial.println(" A");
  }

  // Negative indicator LED for filtered value sign
  digitalWrite(ledNegative, showNegative ? HIGH : LOW);

  delay(200); // keep measurement cadence; display data is written each loop via writeDDRAM
}