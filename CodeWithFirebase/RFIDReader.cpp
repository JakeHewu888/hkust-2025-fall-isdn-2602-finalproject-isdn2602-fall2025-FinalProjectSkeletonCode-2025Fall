#include "RFIDReader.hpp"

MFRC522 RFIDReader::mfrc522(0x28, Pinout::RFID_RST);

void RFIDReader::Init() {
  // Initialize RFID
  Wire.begin(Pinout::RFID_SDA, Pinout::RFID_SCL);
  mfrc522.PCD_Init();
  Serial.println("RFID Initialized");
}

String RFIDReader::GetTagUID() {
  String tagUID = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    tagUID += mfrc522.uid.uidByte[i] < 0x10 ? "0" : "";
    tagUID += String(mfrc522.uid.uidByte[i], HEX);
  }
  tagUID.toUpperCase();
  return tagUID;
}