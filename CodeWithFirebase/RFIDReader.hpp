#ifndef RFIDReader_H
#define RFIDReader_H

#include <Wire.h>

#include "Arduino.h"
#include "MFRC522_I2C.hpp"
#include "Pinout.hpp"

namespace RFIDReader {
/*Creating the Class for RFID Reader*/
extern MFRC522 mfrc522;
struct RFIDTag {
  char uid[9];
};

void Init();
String GetTagUID();
}  // namespace RFIDReader

#endif