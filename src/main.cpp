#include <Arduino.h>
#include "EncMotor.h"
#include "variable.h"

  float x_pos = 0;
  float y_pos = 0;

  float WHEEL_PPR_1 = 384;
  float WHEEL_PPR_2 = 384;
  float WHEEL_PPR_3 = 384;
  float WHEEL_PPR_4 = 384;

  EncoderMotor intEncFL(2,3); 
  EncoderMotor intEncFR(4,5);
  EncoderMotor intEncBL(6,7); 
  EncoderMotor intEncBR(8,9);

  const double WHEEL_RADIUS = 0.05;
// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(9600);  
}
int clock_ms()

void loop() {

}


void controlCalculation(){
  while(1){
    // Mendapatkan Internal Encoder Pulse
    rotInFL = intEncFL.getPulses(1); 
    rotInFR = intEncFR.getPulses(1);
    rotInBL = intEncBL.getPulses(1);
    rotInBR = intEncBR.getPulses(1);
    // Membaca Encoder Ex (opsional)

    // Menambahkan pulse ke akumulasi
    motorPulseFL += rotInFL;
    motorPulseFR += rotInFR;
    motorPulseBL += rotInBL;
    motorPulseBR += rotInBR;
    // Hitung posisi x dan y berdasarkan pulse encoder
    x_pos += (((-rotInFL * 2 * PI * WHEEL_RADIUS )/ WHEEL_PPR_1) + 
              ((rotInFR * 2 * PI * WHEEL_RADIUS )/ WHEEL_PPR_2) - 
              ((rotInBL * 2 * PI * WHEEL_RADIUS) / WHEEL_PPR_3) +  
              ((rotInBR * 2 * PI * WHEEL_RADIUS) / WHEEL_PPR_4))*0.353553391;
    y_pos += (((rotInFL * 2 * PI * WHEEL_RADIUS) / WHEEL_PPR_1) + 
              ((rotInFR * 2 * PI * WHEEL_RADIUS )/ WHEEL_PPR_2) - 
              ((rotInBL * 2 * PI * WHEEL_RADIUS) / WHEEL_PPR_3) -  
              ((rotInBR * 2 * PI * WHEEL_RADIUS )/ WHEEL_PPR_4))*0.353553391;

    // Update posisi lokomosi berdasarkan encoder
    cur_locomotion_L -= temp_cur_locomotion_L;
    cur_locomotion_R -= temp_cur_locomotion_R;
    cur_locomotion_B -= temp_cur_locomotion_B;

    // Sampling time
    t_speed = clock_ms();
    control_period = (t_speed - last_timer_speed)/1000;

    // Hitung kecepatan tiap roda
    locomotion_FL_vel = rotInFL * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR_1 * control_period);
    locomotion_FR_vel = rotInFR * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR_2 * control_period);
    locomotion_BL_vel = rotInBL * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR_3 * control_period);
    locomotion_BR_vel = rotInBR * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR_4 * control_period);

    // Kontrol PID untuk menentukan pwm

    // eksekusi pwm

  }
}
// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}