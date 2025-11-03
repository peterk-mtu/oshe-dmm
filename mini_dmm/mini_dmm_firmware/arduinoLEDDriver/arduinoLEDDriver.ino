#include <Wire.h>

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

// Mapping from your message, with 0x0G -> 0x10 and 0x0H -> 0x11
SegMap digitSegments[4][8] = {
  // D1: a b c d e f g dp
  { {0x0A,0x01}, {0x0A,0x02}, {0x0A,0x04}, {0x1A,0x51},
    {0x1A,0x42}, {0x1A,0x16}, {0x1A,0x20}, {0x0A,0x08} },
  // D2
  { {0x0B,0x01}, {0x0B,0x02}, {0x0B,0x04}, {0x0B,0x42},
    {0x0B,0x40}, {0x0B,0x16}, {0x0B,0x20}, {0x0B,0x08} },
  // D3
  { {0x0C,0x01}, {0x0C,0x02}, {0x0C,0x04}, {0x0C,0x42},
    {0x0C,0x40}, {0x0C,0x16}, {0x0C,0x20}, {0x0C,0x08} },
  // D4 (assumed 0x10/0x11 in place of 0x0G/0x0H)
  { {0x0D,0x01}, {0x0D,0x02}, {0x0D,0x04}, {0x0D,0x42},
    {0x0D,0x40}, {0x0D,0x16}, {0x0D,0x20}, {0x0D,0x08} }
};

const bool segPattern[10][7] = {
  {1,1,1,1,1,1,0}, // 0
  {0,1,1,0,0,0,0}, // 1
  {1,1,0,1,1,0,1}, // 2
  {1,1,1,1,0,0,1}, // 3
  {0,1,1,0,0,1,1}, // 4
  {1,0,1,1,0,1,1}, // 5
  {1,0,1,1,1,1,1}, // 6
  {1,1,1,0,0,0,0}, // 7
  {1,1,1,1,1,1,1}, // 8
  {1,1,1,1,0,1,1}  // 9
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

void displayStringFour(const char *s) {
  memset(ddram, 0x00, DDRAM_SIZE);
  int digit = 0;
  for (size_t i = 0; s[i] != '\0' && digit < 4; ++i) {
    char c = s[i];
    if (c == '.') {
      if (digit > 0) setSegmentLocal(digit-1, 7, true);
    } else if (c >= '0' && c <= '9') {
      int val = c - '0';
      for (int si = 0; si < 7; ++si) setSegmentLocal(digit, si, segPattern[val][si]);
      digit++;
    } else if (c == ' ') {
      digit++;
    }
  }
  updateDisplayFromBuffer();
}

// ---- USER-CONFIGURABLE VALUE ----
// Change this string to the value you want displayed. Use '.' after a digit to set decimal point.
char displayValue[8] = "12.34";

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("BU91796 mapping test");
  bu91796_init();
  displayStringFour(displayValue);
}

void loop() {
  // keep the requested string displayed
  delay(1000);
}