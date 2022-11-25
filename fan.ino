#include <EEPROM.h>

#define prop_pin PD6
#define lift_pin PD5
#define FS_ADDR 0x01
int fanSpeed;

char rx_byte = 0;
String input = "";

void setupFan(uint8_t pin_name) {
  pinMode(3, OUTPUT);
  EEPROM.get(FS_ADDR, fanSpeed);
  if(fanSpeed < 1) fanSpeed = 255;
  analogWrite(pin_name, fanSpeed);
}



void fanWork(uint8_t pin_name, uint8_t max_fan) {
  if (Serial.available() > 0) {    // is a character available?
    rx_byte = Serial.read();       // get the character
  
    // check if a number was received
    if ((rx_byte >= '0') && (rx_byte <= '9')) {
      input.concat(rx_byte);
      
    }
    else if (rx_byte == '\n') {
      Serial.print("Received: ");
      Serial.println(input);
      if(input.toInt() < max_fan) {
        fanSpeed = input.toInt();
        EEPROM.put(FS_ADDR, fanSpeed);
      } else {
        Serial.println("Invalid Number");
      }
      input = "";
    }
    else {
      Serial.println("Not a number.");
    }
  } // end: if (Serial.available() > 0)
  analogWrite(pin_name, fanSpeed);
}

// function to control speed of fan depending on IMU
// int turnFanSpeed() {
//   if getYaw() <

//   return fan_speed;
// }
