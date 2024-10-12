#include "EncMotor.h"
#include "Arduino.h"

// Konstruktor: inisialisasi pin encoder dan state awal
EncoderMotor::EncoderMotor(int encA, int encB) : pinA(encA), pinB(encB), pulses(0), lastStateA(0) {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    lastStateA = digitalRead(pinA); // Baca state awal dari pin A
}

// Fungsi untuk membaca pulsa dari encoder
int EncoderMotor::getPulses(bool reset) {
    int currentStateA = digitalRead(pinA); // Baca state pin A

    // Jika terjadi perubahan pada pin A
    if (currentStateA != lastStateA) {
        // Jika pin B berbeda dengan pin A, hitung sebagai rotasi CCW (Counter-Clockwise)
        if (digitalRead(pinB) != currentStateA) {
            pulses--;  // Rotasi CCW
        } else {
            pulses++;  // Rotasi CW (Clockwise)
        }
    }

    lastStateA = currentStateA; // Simpan state terakhir dari pin A

    int tempPulses = pulses;
    if (reset) {
        pulses = 0; // Reset hitungan pulsa jika diinstruksikan
    }
    return tempPulses;
}

// Fungsi untuk mereset hitungan pulsa
void EncoderMotor::reset() {
    pulses = 0;
}
