#include <Preferences.h>

#include <ESP32TimerInterrupt.h>
#include <ESP32TimerInterrupt.hpp>

#include <ESP32Servo.h>
#include <iarduino_RTC.h> 
#include "BluetoothSerial.h"

#include "stdlib.h"

#define ALARMS_AMOUNT           6

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
#define CMD_SET_ALARM           0x05
#define CMD_GET_ALARMS          0x06

#define ACK_STR                 0x01
#define ACK_ALARMS              0x06

#define LED_BUILTIN             2
#define SERVO_PIN               13

struct alarm { 
  unsigned char hour;
  unsigned char min;
  bool          worked;
};

struct alarm    alarms[ALARMS_AMOUNT];
iarduino_RTC    external_time(RTC_DS1307);
BluetoothSerial SerialBT;
Preferences     preferences;

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
  char str[32];
  farm.servo_pos = SERVO_START_POS;
  if (!SerialBT.begin("servo_test"))
    farm.error = ERR_BT_CONECT;
 
  pinMode(LED_BUILTIN, OUTPUT);
  
  farm.drive.attach(SERVO_PIN);
  farm.drive.write(farm.servo_pos);
  
  farm.feeder_close_time = 0;
  preferences.begin("feeder", true);
  for(int idx = ALARMS_AMOUNT - 1; idx >= 0; --idx)
  {
    for(unsigned hour = 0; hour <= 1; ++hour)
    {
      sprintf(str, "%s_%d", hour ? "hour" :  "min", idx);
      (hour ? alarms[idx].hour : alarms[idx].min) = preferences.getUChar(str, 24);
    }
    if(alarms[idx].hour < 24)
      sprintf(str, "%d - %02d:%02d", idx);
    else
      sprintf(str, "%d - off", idx);
    
    alarms[idx].worked = false;
  }
  preferences.end();
  external_time.begin();
  SendStr("Feeder started, with alarms:");
}

void SendStr(const char* str)
{
  SerialBT.write(0xff);
  SerialBT.write(ACK_STR);
  SerialBT.write(strlen(str));
  SerialBT.write((uint8_t*) str, strlen(str));
}

void CheckAlarm()
{
  for(unsigned idx = 0; idx < ALARMS_AMOUNT; ++idx)
  {
    if(alarms[idx].hour == external_time.Hours && alarms[idx].min == external_time.minutes)
    {
      if(!alarms[idx].worked)
      {
        farm.servo_pos = FEEDER_OPEN_POS;
        farm.feeder_close_time = millis() + 2000;
        alarms[idx].worked = true;
        SendStr("Opened!");
      }
    }
    else
      alarms[idx].worked = false;
  }
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
  
  char buffer[32];

  switch(waiting_cmd)
  {
    case CMD_GET_TIME   : SendStr(external_time.gettime("d-m-Y, H:i:s, D")); waiting_cmd = CMD_NOP; break; //itoa(external_time.gettimeUnix(), buffer, 10); SendStr(buffer);
    case CMD_GET_ALARMS :
    {
      SerialBT.write(0xff);
      SerialBT.write(ACK_ALARMS);
      SerialBT.write(ALARMS_AMOUNT);
      for(unsigned idx = 0; idx < ALARMS_AMOUNT; ++idx)
      {
        SerialBT.write(alarms[idx].hour);
        SerialBT.write(alarms[idx].min);
      }
      // SendStr("ACK_ALARMS sended.");
      waiting_cmd = CMD_NOP;
      break;
    }
    case CMD_SET_ALARM  :
    {
      if(SerialBT.available() >= 3)
      {
        unsigned char idx = SerialBT.read(), hour = SerialBT.read(), min = SerialBT.read();
        if(idx < ALARMS_AMOUNT)
        {
          alarms[idx].hour = hour;
          alarms[idx].min = min;
          alarms[idx].worked = false;
          preferences.begin("feeder", false);
          sprintf(buffer, "hour_%d", idx);
          preferences.putUChar(buffer, hour);
          sprintf(buffer, "min_%d", idx);
          preferences.putUChar(buffer, min);
          preferences.end();
          sprintf(buffer, "Alarm %d (%02d:%02d) setted!", idx, hour, min);
          SendStr(buffer);
        }
        else
          SendStr("ERROR! CMD_SET_ALARM unavailable alarm idx");
        waiting_cmd = CMD_NOP;
      }
      break;
    }
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
      char str[16];
      for(int idx = ALARMS_AMOUNT - 1; idx >= 0 ; --idx)
      {
        if(alarms[idx].hour < 24)
          sprintf(str, "%d - %02d:%02d", idx, alarms[idx].hour, alarms[idx].min);
        else
          sprintf(str, "%d - off", idx);

        SendStr(str);
      }
      
      /*
      farm.servo_pos = FEEDER_OPEN_POS;
      farm.feeder_close_time = millis() + 2000; // TODO: checking overflaw millis()
      SendStr("Opened!");
      */
      //SendStr("CMD_TEST done! :)");
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

  external_time.gettime();

  if(farm.feeder_close_time)
  {
    if(millis() >= farm.feeder_close_time)
    {
      farm.servo_pos = FEEDER_CLOSE_POS;
      SendStr("Closed!");
      farm.feeder_close_time = 0; // TODO: checking overflaw millis()
    }
  }
  else
    CheckAlarm();

  farm.drive.write(farm.servo_pos);
  
  if(SerialBT.available())
    elabor_serial_bt();
}
