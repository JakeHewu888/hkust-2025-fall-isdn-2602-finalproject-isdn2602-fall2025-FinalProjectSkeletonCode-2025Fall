#include "Buzzer.hpp"

//Here is a reference for the melody and duration format, also refer to the pitches.h
int DemoMelody[] = {
  // Phrase 1: Intro Fanfare (16th notes)
  NOTE_FS4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_FS5,
  // Phrase 2: Countdown Motif
  NOTE_B4, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_FS4, NOTE_A4, NOTE_B4,

  NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B6
};

// Note Durations: 
// 2 = half, 4 = quarter, 8 = eighth, 16 = sixteenth
int DemoDurations[] = { 
  // Durations for Intro Fanfare (all 16th notes)
  16, 16, 16, 16, 16, 16, 16, 16,
  // Durations for Countdown Motif
  8, 8, 8, 8, 4, 8, 2,

  2, 2, 2, 1
};


void Buzzer::Init(){
  pinMode(Pinout::Buzzer, OUTPUT);
}

void Buzzer::PlayMelody(int *melody, int *durations){
      // Calculate the number of notes in the melody
    int numNotes = sizeof(melody) / sizeof(melody[0]);
    
    // Iterate over the notes
    for (int thisNote = 0; thisNote < numNotes; thisNote++) {
      
      // To calculate the note duration, take one second divided by the note type.
      // e.g., quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / durations[thisNote];
      
      // Play the note using the tone() function
      tone(Pinout::Buzzer, melody[thisNote], noteDuration);
      
      // To distinguish between notes, set a minimum time between them.
      // The note's duration + 30% (to create a short pause) works well.
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      
      // Stop the tone playing (creates a cleaner staccato effect)
      noTone(Pinout::Buzzer);
    }
    // Final cleanup - ensure the PWM channel is detached
    ledcDetach(Pinout::Buzzer);

}