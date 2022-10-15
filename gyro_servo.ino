#include <Servo.h>
#include <Wire.h>

// Accelerometer variables
long acc_x, acc_y, acc_z;
float g_x, g_y, g_z;

// Gyro variables
long gyro_x = 0, gyro_y = 0, gyro_z = 0;
long prev_gyro_x = 0, prev_gyro_y = 0, prev_gyro_z = 0;
long gyro_callibrate_x = 0, gyro_callibrate_y = 0, gyro_callibrate_z = 0;
float ang_vel_x, ang_vel_y, ang_vel_z; 
long roll = 0, pitch = 0, yaw = 0; // x, y, z

// Time values
float elapsed_time = 0, current_time = 0, previous_time = 0;

// Initialize Servo
Servo hover_servo;

// Servo yaw value
long convert_yaw;

void setup() {
  Serial.begin(9600); //starts the serial at 9600 baud rate
  gpio_init();
  DDRC|=(1<<PC1); // Set pin to output
  DDRB|=(1<<PB5)|(1<<PB0); // set the pin as output
  pinMode(PB5, OUTPUT);
  pinMode(PD3, OUTPUT);
    
  // IMU
  Wire.begin();
  setupIMU(); // initialize connection to imu
  callibrateGyro(); // make sure gyro is properly calibrated
  current_time = millis();

  // SERVO
  hover_servo.attach(9, 1000, 2000); // set servo to go from 0 to 180 degrees
  hover_servo.write(1500); // set servo to neutral position facing forward
}

void loop() {
  updateAccel();
  updateGyro();
  LEDL();
  moveServo();
  printCurrent();
  delay(1000);
}


// Setup IMU
void setupIMU() {
  Wire.beginTransmission(0b1101000); // starts communication
  // Manage power
  Wire.write(0x6B); // access power management register
  Wire.write(0x00000000); // set sleep = 0
  Wire.endTransmission(); // ends communication to power management register

  /*
    configure gyro sensitivity (+- 250, 500, 1000, 2000 deg/s)
      FS_SEL = 0  -> +- 250 deg/s
      FS_SEL = 1  -> +- 500 deg/s
      FS_SEL = 2  -> +- 1000 deg/s
      FS_SEL = 3  -> +- 2000 deg/s
  */
  Wire.beginTransmission(0b1101000); // starts communication
  Wire.write(0x1B); // Communicates with gyro configuration register
  Wire.write(0b00000000); // sets to 0 -> +-250 deg/s
  Wire.endTransmission();

  /*
    configure accelerometer sensitivity (+- 2, 4, 8, 16g)
      AFS_SEL = 0  -> +- 2g
      AFS_SEL = 1  -> +- 4g
      AFS_SEL = 2  -> +- 8g
      AFS_SEL = 3  -> +- 16g
  */
  Wire.beginTransmission(0b1101000); // starts communication
  Wire.write(0x1C); // Communicates with accelerometer sensitivity register
  Wire.write(0b00000000); // sets to 0 -> +-2g
  Wire.endTransmission();
}

// ACCELEROMETER

void updateAccel() {
  getAcceleration();
  getG();
}

// get raw acceleration values
void getAcceleration() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B); // access register for gyro readings
  Wire.endTransmission();

  Wire.requestFrom(0b1101000, 6);

  while (Wire.available() < 6); // wait until all 6 wires are available

  acc_x = Wire.read() <<8|Wire.read();
  acc_y = Wire.read() <<8|Wire.read();
  acc_z = Wire.read() <<8|Wire.read();
}

// convert values to acceleration in terms of g
/*
  dividing factor MUST be changed depending on which FS is picked:
  +- 2g -> 16384.0
  +- 4g -> 8192.0
  +- 8g -> 4096.0
  +- 16g -> 2048.0
*/
void getG() {
  g_x = acc_x / 16384.0;
  g_y = acc_y / 16384.0;
  g_z = acc_z / 16384.0;
}


// GYRO

// callibrate gyro
void callibrateGyro() {
  for (int i = 0; i < 5000; i++) {
    getGyro(); // need current gyro values to calibrate
    gyro_callibrate_x = gyro_callibrate_x + gyro_x;
    gyro_callibrate_y = gyro_callibrate_y + gyro_y;
    gyro_callibrate_z = gyro_callibrate_z + gyro_z;
  }
  gyro_callibrate_x = gyro_callibrate_x / 5000;
  gyro_callibrate_y = gyro_callibrate_y / 5000;
  gyro_callibrate_z = gyro_callibrate_z / 5000;
}

// update gyro data
void updateGyro() {
  prev_gyro_x = gyro_x;
  prev_gyro_y = gyro_y;
  prev_gyro_z = gyro_z;
  previous_time = current_time;
  current_time = millis();

  // loop over gyro function
  getGyro();
  getAngVelocity();
  getAngle();
}

// method to get the readings from gyro
void getGyro() {  
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43); // access register for gyro readings
  Wire.endTransmission();

  Wire.requestFrom(0b1101000, 6); // Request readings from bits 43-48
  while (Wire.available() < 6); // while loop AKA awaits all 6 bits
  // store 2 bytes into each parameter
  gyro_x = Wire.read()<<8|Wire.read();
  gyro_y = Wire.read()<<8|Wire.read();
  gyro_z = Wire.read()<<8|Wire.read();
}

// method to get angular velocity (GYROSCOPE)
/*
  dividing factor MUST be changed depending on which FS is picked:
  +- 250 -> 131.0
  +- 500 -> 65.5
  +- 1000 -> 32.8
  +- 2000 -> 16.4
*/
void getAngVelocity() {
  ang_vel_x = gyro_x / 131.0;
  ang_vel_y = gyro_y / 131.0;
  ang_vel_z = gyro_z / 131.0;
}

void getAngle() {
  elapsed_time = current_time - previous_time;
  roll = ((elapsed_time)*(gyro_x + prev_gyro_x - (2 * gyro_callibrate_x))) * (1/(1000 * 2 * 131.0));
  pitch = ((elapsed_time)*(gyro_y + prev_gyro_y - (2 * gyro_callibrate_y))) * (1/(1000 * 2 * 131.0));
  yaw = ((elapsed_time)*(gyro_z + prev_gyro_z - (2 * gyro_callibrate_z))) * (1/(1000 * 2 * 131.0));
}

// function to move servo depending on yaw
/*
  IMPORTANT: servo cannot move in negative angle values. We want range yaw +- 90 degrees, so convert yaw value to match.
*/
void moveServo() {
  // convert yaw value
  convert_yaw = yaw + 90;

  // check if yaw is within the range -90 <= yaw <= 90
  if (-90 <= yaw <= 90) {
    hover_servo.write(convert_yaw);
  }
}

// control LED_BUILTIN = LEDL
void LEDL() {
  if (yaw < -90 || yaw > 90) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

// PRINTING
void printCurrent() {
  Serial.println("---- CURRENT VALUES ----");
  Serial.println("Angular displacement according to axis:");
  Serial.print("roll: ");
  Serial.print(roll);
  Serial.print(" deg, pitch: ");
  Serial.print(pitch);
  Serial.print(" deg, yaw: ");
  Serial.print(yaw);
  Serial.println(" deg\n");

  Serial.println("Acceleration:");
  Serial.print("x: ");
  Serial.print(g_x);
  Serial.print(" g, y: ");
  Serial.print(g_y);
  Serial.print(" g, z: ");
  Serial.print(g_z);
  Serial.println(" g\n");

  Serial.print("Servo angle:");
  Serial.print(hover_servo.read());
  Serial.println(" deg\n\n");
  }
