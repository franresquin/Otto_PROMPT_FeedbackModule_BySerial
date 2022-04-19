
//#include <M5Stack.h>
#include <M5Core2.h>
#include <Wire.h>
#include "Free_Fonts.h"
#include "utility/CommUtil.h"

/* Unit identifiers */
#define DRIVER1_ADDR        0x56
#define DRIVER2_ADDR        0xA6
#define MOTOR_ADDR_BASE   0x00

/* Visualization variables */
#define STEP_V  20
#define FRONT   4
#define X_LOCAL 60
#define Y_LOCAL 80
#define XF  30
#define YF  30
#define OFFSET 120

/* Serial Protocol */
#define MESSAGE_SIZE      7
#define MESSAGE_HDR_TAIL  0x0A
#define CMD_SET_INTENSITY_M1_4 0x10
#define CMD_SET_INTENSITY_M5_8 0x11
#define CMD_FULL_STOP     0xFF
#define CMD_SET_INTENSITY_M1 0x01
#define CMD_SET_INTENSITY_M2 0x02
#define CMD_SET_INTENSITY_M3 0x03
#define CMD_SET_INTENSITY_M4 0x04
#define CMD_SET_INTENSITY_M5 0x05
#define CMD_SET_INTENSITY_M6 0x06
#define CMD_SET_INTENSITY_M7 0x07
#define CMD_SET_INTENSITY_M8 0x08
#define ONE_SECOND  1000


/* Motor speeds */
#define MOTORS_NUMBERS 8

/* Motors variable definition */
int16_t motor_speed[MOTORS_NUMBERS] = {0};
CommUtil Util;


uint8_t get_motor_index(uint8_t motor_id){
  uint8_t mindex = 0;

  if( (motor_id > MOTORS_NUMBERS) | (motor_id <= 0) ) return 0;

  if(motor_id > 4) mindex = motor_id-5;
  else mindex = motor_id-1;

  return (MOTOR_ADDR_BASE+mindex*2);
  
}
/*************************************************
Function:MotorRun
Description: Motor forward and reverse API
Input:
      n: Motor 0 to motor 3
      Speed: Speed value from 0 to +255,when speed=0,The motor stopped. 
  
Return: Successful return 1
Others: 
*************************************************/
//int32_t MotorRun(uint8_t motor_index, int16_t Speed){
  int32_t MotorRun(uint8_t i2c_address, uint8_t motor_id, int16_t Speed){
    
    uint8_t motor_index=get_motor_index(motor_id);
    // if(motor_index>3)
    //      return 0;

    if(Speed < 0)
        Speed = 0;
  
    if(Speed > 255)
        Speed = 255;
       
    //Util.writeBytes(i2c_address, MOTOR_ADDR_BASE+motor_index*2, (uint8_t *)&Speed, 2);
    Util.writeBytes(i2c_address, motor_index, (uint8_t *)&Speed, 2);
    
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
        if(idx<4){
          M5.Lcd.setCursor(X_LOCAL, Y_LOCAL + YF*idx , FRONT);
          M5.Lcd.printf("M%d: %d       \n", idx+1, motor_speed[idx]);
        }else{
          M5.Lcd.setCursor(X_LOCAL+OFFSET, Y_LOCAL + YF*(idx-4), FRONT);
          M5.Lcd.printf("M%d: %d       \n", idx+1, motor_speed[idx]);
        }
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
  
  /* Set motors 1 to 4 intensities */
  if( (msg_data[1] == CMD_SET_INTENSITY_M1_4) ){
    
    for(byte idx=0; idx<4; idx++){
      MotorRun(DRIVER1_ADDR, idx+1, msg_data[idx+2]);
      motor_speed[idx] = msg_data[idx+2];
    }
    return 0;

  /* Set motors 4 to 8 intensities */
  }else if( (msg_data[1] == CMD_SET_INTENSITY_M5_8) ){

    for(byte idx=4; idx<MOTORS_NUMBERS; idx++){
      MotorRun(DRIVER2_ADDR, idx+1, msg_data[(idx-4)+2]);
      motor_speed[idx] = msg_data[(idx-4)+2];
    }
    return 0;
  
  /* Full stop */
  }else if( (msg_data[1] == 0xFF) ){
    
    for(byte idx=0; idx<MOTORS_NUMBERS; idx++){
      uint8_t address=0;
      //MotorRun(idx, 0);
      if(idx < 4) address = DRIVER1_ADDR;
      else address = DRIVER2_ADDR;
      MotorRun(address, idx+1, 0);
      motor_speed[idx] = 0;
    }
    return 0;

  /* Set motor 1 intensity */
  }else if( (msg_data[1] == CMD_SET_INTENSITY_M1) ){
    
    //MotorRun(0, msg_data[2]);
    //Speed[0] = msg_data[2];
    MotorRun(DRIVER1_ADDR, 1, msg_data[2]);
    motor_speed[0] = msg_data[2];
    return 0;

  /* Set motor 2 intensity */
  }else if( (msg_data[1] == CMD_SET_INTENSITY_M2) ){
    
    //MotorRun(1, msg_data[3]);
    //Speed[1] = msg_data[3];
    MotorRun(DRIVER1_ADDR, 2, msg_data[3]);
    motor_speed[1] = msg_data[3];
    return 0;

  /* Set motor 3 intensity */
  }else if( (msg_data[1] == CMD_SET_INTENSITY_M3) ){
    
    //MotorRun(2, msg_data[4]);
    //Speed[2] = msg_data[4];
    MotorRun(DRIVER1_ADDR, 3, msg_data[4]);
    motor_speed[2] = msg_data[4];
    return 0;

  /* Set motor 4 intensity */
  }else if( (msg_data[1] == CMD_SET_INTENSITY_M4) ){
    
    //MotorRun(3, msg_data[5]);
    //Speed[3] = msg_data[5];
    MotorRun(DRIVER1_ADDR, 4, msg_data[5]);
    motor_speed[3] = msg_data[5];
    return 0;
  
  /* Set motor 5 intensity */
  }else if( (msg_data[1] == CMD_SET_INTENSITY_M5) ){

    MotorRun(DRIVER2_ADDR, 5, msg_data[2]);
    motor_speed[4] = msg_data[2];
    return 0;

  /* Set motor 6 intensity */
  }else if( (msg_data[1] == CMD_SET_INTENSITY_M6) ){

    MotorRun(DRIVER2_ADDR, 6, msg_data[3]);
    motor_speed[5] = msg_data[3];
    return 0;

  /* Set motor 7 intensity */
  }else if( (msg_data[1] == CMD_SET_INTENSITY_M7) ){

    MotorRun(DRIVER2_ADDR, 7, msg_data[4]);
    motor_speed[6] = msg_data[4];
    return 0;

  /* Set motor 7 intensity */
  }else if( (msg_data[1] == CMD_SET_INTENSITY_M8) ){

    MotorRun(DRIVER2_ADDR, 8, msg_data[5]);
    motor_speed[7] = msg_data[5];
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
  //M5.Power.begin();

  /* I2C initialization */
  //Wire.begin();
  Wire.begin(/*SDA*/21, /*SCK*/22);

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

    //update_screen(error_flg==0);

}
