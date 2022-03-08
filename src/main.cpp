/*
    Description: Use LEGO PLUS Module to drive the LEGO motor to rotate and monitor the encoder value.
*/
#include <Arduino.h>
#include <M5Stack.h>
#include <Wire.h>
#include "Free_Fonts.h" 
#include "utility/CommUtil.h"

/* Unit identifiers */
#define SLAVE_ADDR        0x56
#define MOTOR_ADDR_BASE   0x00
#define ENCODER_ADDR_BASE 0x08

/* Visualization variables */
#define STEP_V  20
#define FRONT   4
#define X_LOCAL 60
#define Y_LOCAL 80
#define XF  30
#define YF  30

/* Serial Protocol */
#define MESSAGE_SIZE      7
#define MESSAGE_HDR_TAIL  0x0A
#define CMD_SET_INTENSITY 0x10
#define CMD_FULL_STOP     0xFF
#define CMD_SET_INTENSITY_M1 0x01
#define CMD_SET_INTENSITY_M2 0x02
#define CMD_SET_INTENSITY_M3 0x03
#define CMD_SET_INTENSITY_M4 0x04
#define ONE_SECOND  1000

/* MOTORs IDs*/
#define MOTOR1 0x00
#define MOTOR2 0x01
#define MOTOR3 0x02
#define MOTOR4 0x03

/* Motor speeds */
#define MOTORS_NUMBERS     4
#define MOTOR_ON_THRESHOLD 100

/* Motors variable definition */
int16_t Speed[MOTORS_NUMBERS] = {0};
CommUtil Util;


/*************************************************
Function:MotorRun
Description: Motor forward and reverse API
Input:
      n: Motor 0 to motor 3
      Speed: Speed value from 0 to +255,when speed=0,The motor stopped. 
  
Return: Successful return 1
Others: 
*************************************************/
int32_t MotorRun(uint8_t motor_index, int16_t Speed){
    if(motor_index>3)
         return 0;
  
    if(Speed < 0)
        Speed = 0;
  
    if(Speed > 255)
        Speed = 255;
       
    Util.writeBytes(SLAVE_ADDR, MOTOR_ADDR_BASE+motor_index*2, (uint8_t *)&Speed, 2);
    
    return 1;
}

/*************************************************
Function:header
Description:The UI title
Input:
Return: 
Others: 
*************************************************/
void header(const char *string, uint16_t color){

    M5.Lcd.fillScreen(color);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLUE);
    M5.Lcd.fillRect(0, 0, 320, 30, TFT_BLUE);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString(string, 160, 3, 4);
   
}


/*************************************************
Function:motor_demo
Description:Press the button to control motor rotation,
            and coded values are displayed in real time.
Input:
Return: 
Others:A -> Speed--   B -> Speed=0   C -> Speed++  
*************************************************/
void update_screen(bool update_speed=false){

  //M5.update();

  if(update_speed){

    for(byte idx=0; idx<MOTORS_NUMBERS; idx++){
        M5.Lcd.setCursor(X_LOCAL, Y_LOCAL + YF*idx , FRONT);
        M5.Lcd.printf("M%d: %d      \n", idx, Speed[idx]);
    }

  }

}

/* Serial port -> Check if data corresponds to the coded HEADER or TAIL */
bool is_valid_header(byte data){

  if( (data ==  MESSAGE_HDR_TAIL) )
      return true;
  else return false;

}

/* Serial port -> DEcode to serial msg payload to take actions */
byte decode_Serial_msg(byte msg_data[MESSAGE_SIZE]){
  
  /* Set all motors intensities */
  if( (msg_data[1] == CMD_SET_INTENSITY) ){
    
    for(byte idx=0; idx<MOTORS_NUMBERS; idx++){
      MotorRun(idx, msg_data[idx+2]);
      Speed[idx] = msg_data[idx+2];
    }
    return 0;

  /* Full stop */
  }else if( (msg_data[1] == 0xFF) ){
    
    for(byte idx=0; idx<MOTORS_NUMBERS; idx++){
      MotorRun(idx, 0);
      Speed[idx] = 0;
    }
    return 0;

  /* Set motor 1 intensity */
  }else if( (msg_data[1] == CMD_SET_INTENSITY_M1) ){
    
    MotorRun(0, msg_data[2]);
    Speed[0] = msg_data[2];
    return 0;

  /* Set motor 2 intensity */
  }else if( (msg_data[1] == CMD_SET_INTENSITY_M2) ){
    
    MotorRun(1, msg_data[3]);
    Speed[1] = msg_data[3];
    return 0;

  /* Set motor 3 intensity */
  }else if( (msg_data[1] == CMD_SET_INTENSITY_M3) ){
    
    MotorRun(2, msg_data[4]);
    Speed[2] = msg_data[4];
    return 0;

  /* Set motor 4 intensity */
  }else if( (msg_data[1] == CMD_SET_INTENSITY_M4) ){
    
    MotorRun(3, msg_data[5]);
    Speed[3] = msg_data[5];
    return 0;

  }else{

    return 1;

  }

}

void serial_feedback_msg(byte flg_msg, byte data_buff[MESSAGE_SIZE], byte bytes_read){

  switch (flg_msg) {
  case 0:
    // Serial.print("> [Ok] Data Received: ");
    Serial.print("OK");
    break;
  case 10:
    // Serial.print("> [Error] Invalid HEADER or FOOTER; Data received: ");
    Serial.print("E1");
    break;
  case 1:
    // Serial.print("> [Error] Invalid motor value [0-3]; Data received: ");
    Serial.print("E2");
    break;
  case 55:
    // Serial.print("> [Error] MSG length, 4-bytes needed; Data received: ");
    Serial.print("E3");
    break;
  default:
    // Serial.print("> [Error Not defined] Data Received: ");
    Serial.print("ER");
    break;
  }
  //Serial.println("");
  Serial.flush();

}


void setup() {

  /* Init the M5 hardware */
  M5.begin();
  M5.Power.begin();

  /* I2C initialization */
  Wire.begin();

  /* Serial begin */
  Serial.begin(115200);
  Serial.setTimeout(ONE_SECOND);

  /* Screen */
  M5.Lcd.fillScreen(TFT_BLACK);               
  header("PROMPT", TFT_BLACK);
  M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);
  update_screen(true);

}

void loop() {

    byte serial_buff[MESSAGE_SIZE];
    byte bytes_read=0;
    byte error_flg=255;

    /* Check Serial port */
    if(Serial.available() > 0){
      bytes_read = Serial.readBytes(serial_buff, MESSAGE_SIZE);
      if( (bytes_read == MESSAGE_SIZE) ){
        if( is_valid_header(serial_buff[0]) && is_valid_header(serial_buff[MESSAGE_SIZE-1]) ){
          error_flg = decode_Serial_msg(serial_buff);
        }else {
          error_flg = 10;
        }
      }else{
        error_flg = 55;
      }
      serial_feedback_msg(error_flg, serial_buff, bytes_read);
    }

    update_screen(error_flg==0);

}