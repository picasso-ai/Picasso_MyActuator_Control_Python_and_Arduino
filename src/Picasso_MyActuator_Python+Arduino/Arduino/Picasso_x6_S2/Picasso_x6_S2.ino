//Required Files: Gyems_Teensy41.cpp, Gyems_Teensy41.h, Serial_Isra.cpp, Serial_Isra.h

#include "Gyems_Teensy41.h"
#include <FlexCAN_T4.h>
#include "Serial_Isra.h"

/*Canbus Setup*/
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
CAN_message_t msgR;
/*Canbus Setup*/

/*Motor Setup*/
uint32_t Motor_ID1 = 1; // Motor Can Bus ID, left leg, loadcell: port 1, positive current = flexion
uint32_t ID_offset = 0x140;
double Gear_ratio = 36;
int CAN_ID = 3;
double Pgain = 7.5;    //P gain of torque control
double Igain = 0.7;    //I gain of torque control
double Dgain = 0;      //D gain of torque control
double MaxPIDout = 10; //Max torque control PID output (Unit Current A, inner loop is current controller)
double Cur_command_L = 0;
Gyems_Teensy41 m1(Motor_ID1, CAN_ID, Gear_ratio);
/*Motor Setup*/

/*Isra Serial Class Setup*/
Serial_Isra Serial_Isra;
/*Isra Serial Class Setup*/

/*Serial Send/Recieve*/
size_t Send_Length = 9;
char Send[9] = {0x30, 0x31, 0x32, 0x32, 0x33, 0x33, 0x33, 0x33, 0x33};
char SerialData2[6] = {0xff, 0xee, 0xdd, 0x77, 0x77, 0x33};
/*Serial Send/Recieve*/

/*Motor Send/Recieve Variables*/
uint16_t position_int = 0x0000;
uint16_t speed_int = 0x0000;
uint16_t current_int = 0x0000;
uint16_t Cur_command_L_uint = 0x0000;
/*Motor Send/Recieve Variables*/

/*Time Control*/
unsigned long Delta_T1 = 20000;
unsigned long t_i, t_pr1;
double HZ = 1.0 / (Delta_T1 / 100000.0);
unsigned long input_delay_control=2000;
/*Time Control*/

/*Position Conversion Variables*/
float position_delta = 0;
float position1 = 0;
/*Position Conversion Variables*/

void setup() {
  delay(3000);
  Serial.begin(115200);
  Serial7.begin(115200);
  Serial_Isra.INIT();
  initial_CAN();
  m1.init_motor();
  reset_motor_angle();

  Serial.print("Serial HZ: ");
  Serial.println(HZ);

  Serial.println("SETUP DONE");
  delay(100);
}

void loop() {
  Serial_Isra.READ2();

  //Assign Recieved Current Command
  Cur_command_L_uint = (Serial_Isra.SerialData2[4] << 8 | Serial_Isra.SerialData2[3]);
  Cur_command_L = Serial_Isra.uint_to_float(Cur_command_L_uint, -10, 10, 16);

  //Send Current Command to Motor
  m1.send_current_command_for36(Cur_command_L);

  t_i = micros();

  if (t_i - t_pr1  > Delta_T1) {
    t_pr1 = t_i;

    receive_CAN_data();
    Wait(input_delay_control);

    //Read Motor Angle/Speed
    m1.read_multi_turns_angle_for36();
    m1.receive_CAN_data();
    Wait(input_delay_control);

    //Read Motor Current
    m1.read_motor_current_for36();
    receive_CAN_data();
    Wait(input_delay_control);

    /*Send Motor Variables to Python*/

    /*Convert Encoder Position to degrees WIP */
    uint16_t position_delta_uint = m1.encoder;
    position_delta = Serial_Isra.uint_to_float(position_delta_uint, 0, 360, 16);
    position1 = position1 + position_delta;
    if (position1 > 360) {
      position1 = position1 - 360;
    }
    /*Convert Encoder Position to degrees WIP */

    /*Assign other Motor Variables*/
    float speed1 = m1.speed_value;
    float current = m1.iq_A;
    /*Assign other Motor Variables*/

    /*Convert Motor Variables to uint and then to high and low bytes*/
    position_int = Serial_Isra.float_to_uint(position1, 0, 360, 16);
    speed_int = Serial_Isra.float_to_uint(speed1, -25, 25, 16);
    current_int = Serial_Isra.float_to_uint(current, -10, 10, 16);

    Send[2] = current_int >> 8;
    Send[3] = current_int & 0xFF;
    Send[4] = speed_int >> 8;
    Send[5] = speed_int & 0xFF;
    Send[6] = position_int >> 8;
    Send[7] = position_int & 0xFF;
    /*Convert Motor Variables to uint and then to high and low bytes*/

    /*Send Motor Variables on Serial Bus*/
    Serial_Isra.WRITE(Send, Send_Length);
    /*Send Motor Variables on Serial Bus*/

    /*Send Motor Variables to Python*/

    /*Print Statements for Debug*/
    Serial.print("Current Command:  ");
    Serial.print(Cur_command_L);
    Serial.print(" Current:  ");
    Serial.print(m1.iq_A);
    Serial.print(" Position: ");
    Serial.print(position1);
    Serial.print(" Speed: ");
    Serial.println(m1.speed_value);
    /*Print Statements for Debug*/
  }
}


void reset_motor_angle() {
  for (int i = 0; i < 20; i++)
  {
    m1.read_multi_turns_angle();
    delay(10);
    receive_CAN_data();
    m1.motorAngle_offset = m1.motorAngle_raw;
    delay(10);
  }
}

void receive_CAN_data() {
  //  if (Can3.read(msgR))
  //  {
  Can3.read(msgR);
  if (msgR.id == (ID_offset + Motor_ID1))
  {
    m1.DataExplanation(msgR);
  }
  //  }
}

void initial_CAN()
{
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
  Serial.println("Can bus setup done...");
  delay(200);
}

void Wait(unsigned long delay_control)
{
  unsigned long Time_start = micros();
  unsigned long Time_Delta = delay_control;
  unsigned long Time_Control = 0;
  
  do {
    Time_Control = micros()-Time_start;
  }
  while (Time_Control < Time_Delta);

}
