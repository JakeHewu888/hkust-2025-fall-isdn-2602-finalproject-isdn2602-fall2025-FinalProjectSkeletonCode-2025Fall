#ifndef RFIDmap_HPP
#define RFIDmap_HPP

#include <Arduino.h>

#define GRID_ROWS 5
#define GRID_COLS 5
#define RFID_PER_TILE 9

// 每个 tile 的 RFID 集合
struct RFIDTile {
  char rfid[RFID_PER_TILE][9];
  int tileId;
  int row;
  int col;
};

int scanRFIDtoTILE(const String &currentUID);

#endif