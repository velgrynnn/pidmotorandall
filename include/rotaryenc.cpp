// Rotary Encoder Inputs (sesuaikan dengan pin yang terhubung)
#define ENC_A 2
#define ENC_B 3
#include "Arduino.h"

int counter = 0;
int currentStateA;
int lastStateA;
String currentDir = "";

void setup() {
    pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);

    Serial.begin(9600);

    // Inisialisasi keadaan awal dari pin A
    lastStateA = digitalRead(ENC_A);
}

void loop() {
  // Baca keadaan pin A saat ini
    currentStateA = digitalRead(ENC_A);

  // Jika terjadi perubahan pada pin A
    if (currentStateA != lastStateA) {
    // Jika pin B berbeda dengan A, rotasi CCW (berlawanan jarum jam)
    if (digitalRead(ENC_B) != currentStateA) {
        counter--;
        currentDir = "CCW";
    } else {
      // Jika sama, rotasi CW (searah jarum jam)
        counter++;
        currentDir = "CW";
    }
    
    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.println(counter);
    }

  // Ingat keadaan terakhir pin A
    lastStateA = currentStateA;
}
