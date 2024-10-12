#ifndef ENCODERMOTOR_H
#define ENCODERMOTOR_H

#include "Arduino.h"

class EncoderMotor {
private:
    int pinA, pinB;      // Pin A dan B dari encoder
    int pulses;          // Jumlah pulsa yang telah dihitung
    int lastStateA;      // Menyimpan state terakhir dari pin A

public:
    // Konstruktor
    EncoderMotor(int encA, int encB);

    // Fungsi untuk mendapatkan jumlah pulsa yang terbaca
    int getPulses(bool reset);

    // Fungsi untuk mereset hitungan pulsa
    void reset();
};

#endif
