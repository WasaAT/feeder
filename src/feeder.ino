#include <ESP32TimerInterrupt.h>
#include <ESP32TimerInterrupt.hpp>

#include <ESP32Servo.h>
#include <iarduino_RTC.h> 
#include "BluetoothSerial.h"

#include "stdlib.h"

#define SERVO_MIN_POS           100
#define SERVO_MAX_POS           174
#define SERVO_START_POS         SERVO_MAX_POS

#define FEEDER_OPEN_POS         SERVO_MIN_POS
#define FEEDER_CLOSE_POS        SERVO_MAX_POS

#define SUCCESS                 0x00
#define ERR_BT_CONECT           0x01

#define CMD_NOP                 0x00
#define CMD_TEST                0x01
#define CMD_SERVO               0x02
#define CMD_GET_TIME            0x03
#define CMD_SET_TIME            0x04

#define ACK_STR                 0x01

#define LED_BUILTIN             2
#define SERVO_PIN               13

iarduino_RTC  external_time(RTC_DS1307);
BluetoothSerial SerialBT;
int pos = 0;

struct {
  unsigned long feeder_close_time;
  unsigned char error;
  unsigned char servo_pos;
  Servo         drive;
} farm;

void portion(unsigned portion_val)
{
  
}

void setup()
{
  farm.servo_pos = SERVO_START_POS;
  if (!SerialBT.begin("servo_test"))
    farm.error = ERR_BT_CONECT;
 
  pinMode(LED_BUILTIN, OUTPUT);
  
  farm.drive.attach(SERVO_PIN);
  farm.drive.write(farm.servo_pos);
  
  farm.feeder_close_time = 0;

  Serial.begin(9600);
  external_time.begin();
//  external_time.settime(0, 30, 12, 25, 10, 22, 2);
}

void SendStr(const char* str)
{
  SerialBT.write(0xff);
  SerialBT.write(ACK_STR);
  SerialBT.write(strlen(str));
  SerialBT.write((uint8_t*) str, strlen(str));
}

unsigned char waiting_cmd = CMD_NOP;

void elabor_serial_bt()
{
  if(waiting_cmd == CMD_NOP)
  {
    while(SerialBT.available() >= 2)
    {
      if(!SerialBT.read())
      {
        waiting_cmd = SerialBT.read();
        break;
      }
    }
  }
  
  char buffer[16];

  switch(waiting_cmd)
  {
    case CMD_GET_TIME   : SendStr(external_time.gettime("d-m-Y, H:i:s, D")); waiting_cmd = CMD_NOP; break; //itoa(external_time.gettimeUnix(), buffer, 10); SendStr(buffer); 
    case CMD_SET_TIME   : 
    {
      if(SerialBT.available() >= 4)
      {
        unsigned time = 0;
        for(unsigned idx = 0; idx < 4; ++idx)
          *(((unsigned char*) &time) + idx) = SerialBT.read();
          //time |= ((unsigned) (((unsigned char*) &time) + idx)) << idx*8;
        //utoa(time, buffer, 10); SendStr(buffer);
        external_time.settimeUnix(time);
        SendStr(external_time.gettime("d-m-Y, H:i:s, D")); //itoa(external_time.gettimeUnix(), buffer, 10); SendStr(buffer);
        waiting_cmd = CMD_NOP;
      }
      //SendStr(external_time.gettime("d-m-Y, H:i:s, D")); waiting_cmd = CMD_NOP;
      break;
    }
    //case CMD_TIME   : itoa(external_time.gettimeUnix(), buffer, 10); SendStr(buffer); waiting_cmd = CMD_NOP; break;
    case CMD_TEST   : 
    {
      farm.servo_pos = FEEDER_OPEN_POS;
      farm.feeder_close_time = millis() + 2000; // TODO: checking overflaw millis()
      SendStr("CMD_TEST done!");
      waiting_cmd = CMD_NOP;
      break;
    }
    case CMD_SERVO  : 
    {
      if(SerialBT.available())
      {
        farm.servo_pos = SerialBT.read();
        waiting_cmd = CMD_NOP;
      }
      break;
    }
    default: waiting_cmd = CMD_NOP;
  }
}

void loop() {
  
  while(farm.error)
  {
    digitalWrite(LED_BUILTIN, pos ? HIGH : LOW);
    pos = pos ? 0 : 1;
    delay(500);
  }
  
  if(farm.feeder_close_time && (millis() >= farm.feeder_close_time))
  {
    farm.servo_pos = FEEDER_CLOSE_POS;
    farm.feeder_close_time = 0; // TODO: checking overflaw millis()
  }

  farm.drive.write(farm.servo_pos);
  
  if(SerialBT.available())
    elabor_serial_bt();
}
