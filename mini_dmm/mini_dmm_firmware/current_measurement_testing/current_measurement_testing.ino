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

// Segmap from your provided mapping (kept as-is)
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

// ---------- Measurement + display helpers ----------

const int pinA3 = A3;
const float ADC_REF_V = 3.3f;
const int ADC_MAX = 1023;

// New variables per your request
const float gain_var = 6.8f;   // gain = 10
const float shunt_var = 10.0f;  // shunt = 10


// Display absolute value XX.XX on panel (leading zero shown by default)
void displayAbsXXdotXX(float v) {
  if (isnan(v)) v = 0.0f;
  float absVal = fabs(v);
  if (absVal > 99.995f) absVal = 99.995f; // clamp

  int totalCents = (int)roundf(absVal * 100.0f);
  int frac = totalCents % 100;
  int integer = totalCents / 100;

  char out[6];
  int tens = integer / 10;
  int units = integer % 10;
  int tenths = frac / 10;
  int hundredths = frac % 10;

  out[0] = '0' + tens;
  out[1] = '0' + units;
  out[2] = '.';
  out[3] = '0' + tenths;
  out[4] = '0' + hundredths;
  out[5] = '\0';

  displayStringFour(out);
}

// ---------- Setup / Loop ----------

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Single-channel current reader (A3) - I = V / (gain * shunt)");

  bu91796_init();
  displayStringFour("00.00");
  delay(200);
}

void loop() {
  
  
// // Read ADC, convert to voltage
// float rawToVoltage(int raw) {
//   return (raw * ADC_REF_V) / (float)ADC_MAX;
// }

// float readVoltageA3() {
//   int raw = analogRead(pinA3);
//   return rawToVoltage(raw);
// }
  
  
  
  
  
  
  // // Read voltage at A3
  int raw = analogRead(pinA3);
  float v = ((raw / (float)ADC_MAX ) * ADC_REF_V) / gain_var;

  float current = v / shunt_var;

  // Print signed current to Serial with 3 decimal places
  Serial.print("A3 raw V = ");
  Serial.print(v, 6);
  Serial.print(" V  -> I = ");
  Serial.print(current, 6);
  Serial.println(" A");

  // Display absolute value on LCD as XX.XX
  displayAbsXXdotXX(current);

  delay(250); // update rate (adjust as needed)
}