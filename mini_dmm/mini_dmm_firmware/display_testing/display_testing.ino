// // Copilot Attempt 1:

// // Digit-enable (common cathode) pins D13…D10
// const uint8_t digitPins[4]   = { 13, 12, 11, 10 };

// // Segment pins A,B,C,D,E,F,G and COL on D9…D2
// enum : uint8_t { SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G, SEG_COL };
// const uint8_t segmentPins[8] = { 9, 8, 7, 6, 5, 4, 3, 2 };

// // 0–9 segment patterns {A,B,C,D,E,F,G}
// const bool segmentPatterns[10][7] = {
//   {1,1,1,1,1,1,0}, {0,1,1,0,0,0,0}, {1,1,0,1,1,0,1},
//   {1,1,1,1,0,0,1}, {0,1,1,0,0,1,1}, {1,0,1,1,0,1,1},
//   {1,0,1,1,1,1,1}, {1,1,1,0,0,0,0}, {1,1,1,1,1,1,1},
//   {1,1,1,1,0,1,1}
// };

// // Refresh timing and current display values
// const uint16_t refreshDelay = 2000;  
// uint8_t displayValue[4]       = {2,0,2,5};

// // Toggles each full-cycle to reverse drive polarity
// bool acPhase = false;

// void setup() {
//   for (uint8_t i = 0; i < 4; i++) {
//     pinMode(digitPins[i],   OUTPUT);
//     digitalWrite(digitPins[i], HIGH);  // Commons start off
//   }
//   for (uint8_t s = 0; s < 8; s++) {
//     pinMode(segmentPins[s], OUTPUT);
//     digitalWrite(segmentPins[s], LOW); // Segments start off
//   }
// }

// void loop() {
//   // Flip AC phase every full 4-digit pass
//   acPhase = !acPhase;

//   for (uint8_t digit = 0; digit < 4; digit++) {
//     // 1) Turn all commons off
//     for (uint8_t d = 0; d < 4; d++) {
//       digitalWrite(digitPins[d], acPhase ? LOW : HIGH);
//     }

//     // 2) Drive segments for this digit
//     uint8_t val = displayValue[digit];
//     for (uint8_t seg = 0; seg < 7; seg++) {
//       bool on = segmentPatterns[val][seg];
//       // In phase 0: HIGH lights; in phase 1: LOW lights
//       digitalWrite(segmentPins[seg], (on ^ acPhase) ? HIGH : LOW);
//     }
//     // Colon always off in this example
//     digitalWrite(segmentPins[SEG_COL], acPhase ? LOW : HIGH);

//     // 3) Enable this digit common
//     digitalWrite(digitPins[digit], acPhase ? HIGH : LOW);

//     // 4) Hold to complete its 1/4-duty slot
//     delayMicroseconds(refreshDelay);
//   }
// }


//COPILOT ATTEMPT 2:
// // Pin assignments
// const uint8_t digitPins[4] = { 13, 12, 11, 10 }; // D13..D10 sources for digit0..digit3
// const uint8_t ctrlPins[8]  = { 9, 8, 7, 6, 5, 4, 3, 2 }; // D9..D2

// // Segment indices
// enum { SEG_A=0, SEG_B=1, SEG_C=2, SEG_D=3, SEG_E=4, SEG_F=5, SEG_G=6, SEG_DP=7, SEG_COL=8 };

// // Standard 7-segment patterns for 0..9 (A..G). DP handled separately.
// const bool digit7segTable[10][7] = {
//   {1,1,1,1,1,1,0}, // 0
//   {0,1,1,0,0,0,0}, // 1
//   {1,1,0,1,1,0,1}, // 2
//   {1,1,1,1,0,0,1}, // 3
//   {0,1,1,0,0,1,1}, // 4
//   {1,0,1,1,0,1,1}, // 5
//   {1,0,1,1,1,1,1}, // 6
//   {1,1,1,0,0,0,0}, // 7
//   {1,1,1,1,1,1,1}, // 8
//   {1,1,1,1,0,1,1}  // 9
// };

// // Per-digit pattern buffer: A,B,C,D,E,F,G,DP
// bool digitPattern[4][8];

// // Colon flag and per-digit DP flags
// bool showColon = false;
// bool dpFlags[4] = { false, false, false, false };

// // Numeric values to display (0..9). Call applyDisplayValues() after changing this.
// uint8_t displayValue[4] = { 2, 0, 2, 5 };

// // Per-source-to-ctrl mapping (as provided earlier)
// struct MapEntry { uint8_t targetDigit; uint8_t segIndex; };
// MapEntry mapping[4][8] = {
//   { {0,SEG_D}, {0,SEG_DP}, {1,SEG_D}, {1,SEG_DP}, {2,SEG_D}, {2,SEG_DP}, {3,SEG_D}, {0,SEG_COL} },
//   { {0,SEG_E}, {0,SEG_C},  {1,SEG_E}, {1,SEG_C},  {2,SEG_E}, {2,SEG_C},  {3,SEG_E}, {3,SEG_C}  },
//   { {0,SEG_G}, {0,SEG_B},  {1,SEG_G}, {1,SEG_B},  {2,SEG_G}, {2,SEG_B},  {3,SEG_G}, {3,SEG_B}  },
//   { {0,SEG_F}, {0,SEG_A},  {1,SEG_F}, {1,SEG_A},  {2,SEG_F}, {2,SEG_A},  {3,SEG_F}, {3,SEG_A}  }
// };

// // Timing and AC phase control
// const uint16_t refreshDelayUs = 1800;
// bool acPhase = false;

// // Helpers to control ctrl pins (sink = OUTPUT LOW, release = INPUT high-Z)
// inline void ctrlSink(uint8_t idx) { pinMode(ctrlPins[idx], OUTPUT); digitalWrite(ctrlPins[idx], LOW); }
// inline void ctrlRelease(uint8_t idx) { pinMode(ctrlPins[idx], INPUT); }

// // Release all ctrl pins
// void releaseAllCtrl() {
//   for (uint8_t i = 0; i < 8; ++i) ctrlRelease(i);
// }

// // Set all digit pins to inactive polarity
// void setAllDigitsInactive() {
//   for (uint8_t d = 0; d < 4; ++d) {
//     if (!acPhase) digitalWrite(digitPins[d], LOW); else digitalWrite(digitPins[d], HIGH);
//   }
// }

// // Show one source (one digit timeslot)
// void showSource(uint8_t sourceIndex) {
//   // prepare
//   releaseAllCtrl();
//   setAllDigitsInactive();

//   // configure sinks according to mapping and current digitPattern/flags
//   for (uint8_t c = 0; c < 8; ++c) {
//     MapEntry me = mapping[sourceIndex][c];

//     if (me.segIndex == SEG_COL) {
//       if (showColon) ctrlSink(c); else ctrlRelease(c);
//       continue;
//     }

//     if (me.targetDigit < 4) {
//       bool want = false;
//       if (me.segIndex <= SEG_G) want = digitPattern[me.targetDigit][me.segIndex];
//       else if (me.segIndex == SEG_DP) want = digitPattern[me.targetDigit][SEG_DP];
//       if (want) ctrlSink(c); else ctrlRelease(c);
//     } else {
//       ctrlRelease(c);
//     }
//   }

//   // enable source at active polarity
//   if (!acPhase) digitalWrite(digitPins[sourceIndex], HIGH); else digitalWrite(digitPins[sourceIndex], LOW);

//   // hold timeslot
//   delayMicroseconds(refreshDelayUs);

//   // disable source
//   if (!acPhase) digitalWrite(digitPins[sourceIndex], LOW); else digitalWrite(digitPins[sourceIndex], HIGH);

//   // release sinks
//   releaseAllCtrl();
// }

// // Helper: convert displayValue[] + dpFlags[] into digitPattern[][] using standard 7-seg table
// void applyDisplayValues() {
//   for (uint8_t d = 0; d < 4; ++d) {
//     uint8_t v = displayValue[d];
//     if (v <= 9) {
//       for (uint8_t s = 0; s < 7; ++s) digitPattern[d][s] = digit7segTable[v][s];
//     } else {
//       // For values outside 0-9 turn segments off (could be used for blank)
//       for (uint8_t s = 0; s < 7; ++s) digitPattern[d][s] = false;
//     }
//     // decimal point from dpFlags
//     digitPattern[d][SEG_DP] = dpFlags[d];
//   }
// }

// void setup() {
//   // initialize digit pins and set inactive state
//   for (uint8_t i = 0; i < 4; ++i) {
//     pinMode(digitPins[i], OUTPUT);
//     digitalWrite(digitPins[i], LOW); // start inactive assuming acPhase==false
//   }
//   // start with ctrl pins released
//   releaseAllCtrl();

//   // initialize digitPattern from initial displayValue
//   applyDisplayValues();
// }

// void loop() {
//   // Example: update displayValue or dpFlags here as needed, then call applyDisplayValues()
//   // displayValue[0]=2; displayValue[1]=0; displayValue[2]=2; displayValue[3]=5;
//   // dpFlags[1] = true; showColon = true; applyDisplayValues();

//   // toggle AC phase once per full refresh
//   acPhase = !acPhase;

//   for (uint8_t s = 0; s < 4; ++s) showSource(s);
// }


//TESTING:

// // Square wave on D13 with 1/4 duty cycle
// // Adjust frequencyHz to change the output frequency.

// const uint8_t outPin = 13;      // D13
// const unsigned long frequencyHz = 1000UL; // desired frequency in Hz (example: 1 kHz)

// void setup() {
//   pinMode(outPin, OUTPUT);
//   digitalWrite(outPin, LOW);
// }

// void loop() {
//   unsigned long periodUs = 1000000UL / frequencyHz;       // period in microseconds
//   unsigned long highTimeUs = periodUs / 4UL;              // 1/4 duty = 25% high
//   unsigned long lowTimeUs  = periodUs - highTimeUs;      // remaining 75% low

//   digitalWrite(outPin, HIGH);
//   delayMicroseconds(highTimeUs);

//   digitalWrite(outPin, LOW);
//   delayMicroseconds(lowTimeUs);
//}








// ANOTHER ATTEMPT w/ Display Driver Chip
// BU91796_float_display.ino
// Arduino sketch (NUCLEO-F303K8 Arduino core) to display a float on a 4-digit LCD
// using a BU91796FS-M over I2C. Adjust I2C_ADDR, REG_BASE, and posToRamIndex[]
// to match your hardware/datasheet before use.

// Include Wire (I2C)
#include <Wire.h>

// Configuration - set these to match your hardware/datasheet
#define I2C_ADDR 0x3C          // <- change to actual BU91796 I2C address
#define REG_DISPLAY_BASE 0x00  // <- change to first display RAM register per datasheet

// Timing
#define DISPLAY_UPDATE_MS 200

// 7-seg bit order convention (example): bit0 = segment A, bit1 = B, ..., bit6 = G, bit7 = DP
// Change bit assignments to match your panel/driver mapping.
const uint8_t digitToSeg[16] = {
  // 0-9, A-F (bit mask: 0b0GFEDCBA)
  0b00111111, // 0 = A B C D E F
  0b00000110, // 1 = B C
  0b01011011, // 2 = A B D E G
  0b01001111, // 3 = A B C D G
  0b01100110, // 4 = B C F G
  0b01101101, // 5 = A C D F G
  0b01111101, // 6 = A C D E F G
  0b00000111, // 7 = A B C
  0b01111111, // 8 = A B C D E F G
  0b01101111, // 9 = A B C D F G
  0b01110111, // A
  0b01111100, // b (lowercase)
  0b00111001, // C
  0b01011110, // d (lowercase)
  0b01111001, // E
  0b01110001  // F
};

// Map logical digit positions to BU91796 display RAM indices.
// pos 0 = leftmost digit, pos 3 = rightmost digit.
// You must set these to the RAM addresses/register indices the BU91796 expects.
uint8_t posToRamIndex[4] = { 0, 1, 2, 3 };

// Colon and decimal dot handling:
// - This example expects decimal point as the MSB (bit7) of each digit byte.
// - Colon is toggled by a separate RAM byte or shared bit depending on your panel; adjust below.
int colonRamIndex = -1;     // set to RAM index controlling colon if applicable, else -1

// Helper: write buffer to BU91796 display RAM starting at REG_DISPLAY_BASE
void bu91796_write_display_ram(uint8_t startReg, const uint8_t *buf, size_t len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(startReg);
  for (size_t i = 0; i < len; ++i) Wire.write(buf[i]);
  Wire.endTransmission();
}

// Compose RAM buffer from a float value
// value: numeric value to display; decimals: number of decimal digits to show (0..3)
void displayFloatOnBU91796(float value, uint8_t decimals) {
  // Bound decimals to 0..3
  if (decimals > 3) decimals = 3;

  // Handle negative numbers
  bool negative = (value < 0.0f);
  if (negative) value = -value;

  // Scale and round to integer
  long scaled = (long)round(value * pow(10, decimals));

  // Build digits right-to-left
  uint8_t ramBuf[8]; // buffer sufficiently large to match driver RAM layout; adjust as needed
  memset(ramBuf, 0x00, sizeof(ramBuf));

  // Extract 4 digits (max 4 display digits)
  for (int pos = 3; pos >= 0; --pos) {
    int digit = scaled % 10;
    scaled /= 10;
    uint8_t seg = digitToSeg[digit] & 0x7F; // mask out DP bit
    // Add decimal dot if this digit is the decimal position
    if (pos == (int)(4 - 1 - (int)decimals) && decimals > 0) {
      seg |= 0x80; // set DP as bit7
    }
    // If we ran out of digits and the digit is leading zero, blank it unless needed
    // We'll handle leading blanking after extracting all digits
    ramBuf[posToRamIndex[pos]] = seg;
  }

  // If negative and space exists, show minus sign on leftmost digit (example: segment G only)
  if (negative) {
    // choose G segment only (bit6) for minus; set to 0x40 if mapping uses bit6 for G
    uint8_t minusMask = 0x40;
    ramBuf[posToRamIndex[0]] = minusMask;
  } else {
    // perform leading zero blanking: replace leading '0' patterns with 0x00
    for (int p = 0; p < 3; ++p) {
      uint8_t b = ramBuf[posToRamIndex[p]];
      if (b == digitToSeg[0] && !(ramBuf[posToRamIndex[p+1]] == 0)) {
        // when the next digit is non-zero or has DP, stop blanking
        // Conservative approach: blank if equal to zero pattern and more significant digits are zero
        // Simpler approach: if topmost and equals zero pattern and no DP, blank
      }
    }
    // A simpler and safe blanking: blank leftmost digits that equal '0' and have no DP and are leading
    for (int p = 0; p < 3; ++p) {
      uint8_t b = ramBuf[posToRamIndex[p]] & 0x7F;
      if (b == digitToSeg[0]) {
        // check if any lower-significance digit is non-zero
        bool anyLowerNonZero = false;
        for (int q = p+1; q < 4; ++q) {
          if ((ramBuf[posToRamIndex[q]] & 0x7F) != digitToSeg[0]) { anyLowerNonZero = true; break; }
        }
        if (!anyLowerNonZero) {
          // all lower digits are zero -> blank this leading zero
          ramBuf[posToRamIndex[p]] = 0x00;
        } else break;
      } else break;
    }
  }

  // Handle colon if applicable
  if (colonRamIndex >= 0) {
    // Example: colon on when value has fractional part (customize as needed)
    uint8_t colonByte = ( (decimals > 0) ? 0x01 : 0x00 );
    ramBuf[colonRamIndex] = colonByte;
  }

  // Finally write the appropriate number of bytes to the driver.
  // Choose length so it covers the highest RAM index you modified
  uint8_t highestIndex = 0;
  for (int i = 0; i < 4; ++i) if (posToRamIndex[i] > highestIndex) highestIndex = posToRamIndex[i];
  if (colonRamIndex > highestIndex) highestIndex = colonRamIndex;
  size_t len = highestIndex + 1;

  // Write buffer starting at REG_DISPLAY_BASE
  bu91796_write_display_ram(REG_DISPLAY_BASE, ramBuf, len);
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(10);
  Serial.println("BU91796 display demo");
}

void loop() {
  // Example: read float from Serial input like "20.25\n"
  static char inbuf[32];
  static size_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        inbuf[idx] = '\0';
        float v = atof(inbuf);
        // default: 2 decimal places if input contains '.' with two digits, else infer
        uint8_t decimals = 0;
        char *dot = strchr(inbuf, '.');
        if (dot) {
          decimals = strlen(dot+1);
          if (decimals > 3) decimals = 3;
        }
        displayFloatOnBU91796(v, decimals);
        idx = 0;
      }
    } else {
      if (idx < sizeof(inbuf)-1) inbuf[idx++] = c;
    }
  }

  delay(DISPLAY_UPDATE_MS);
}

