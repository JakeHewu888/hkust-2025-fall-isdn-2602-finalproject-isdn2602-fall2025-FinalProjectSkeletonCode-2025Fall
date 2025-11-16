#ifndef Buzzer_H
#define Buzzer_H

#include "Pinout.hpp"
#include "Arduino.h"
#include "pitches.h"

// -------------------------------------------------
//  Declaration of the demo arrays (extern)
// -------------------------------------------------
extern int DemoMelody[];
extern int DemoDurations[];

namespace Buzzer {
void Init();
void PlayMelody(int* melody, int* durations);
}

#endif