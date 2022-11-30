

void setup() {
  // fan TO FINISH

  // TODO: replace with the following two lines:
  pinMode(lift_pin, 0x1);
  analogWrite(lift_pin, 255);cd 
  pinMode(prop_pin, 0x1);
  //setupFan(lift_pin);
  //setupFan(prop_pin);

  Serial.begin(9600); //starts the serial at 9600 baud rate
  gpio_init();

  // IMU
  Wire.begin();
  setupIMU(); // initialize connection to imu
  callibrateGyro(); // make sure gyro is properly calibrated
  current_time = millis();

  // SERVO
  hover_servo.attach(9, 1000, 2000); // set servo to go from 0 to 180 degrees
  hover_servo.write(1500); // set servo to neutral position facing forward
}

void backFanCTL(double turnAngle) {\
// Try setting a turn maximum. It may help us not lose momentum.
  if(turnAngle > 30) {
    return;
  }
  // Then do progressive speed decrease when turning
  // for a turn of 30 degrees we would have a speed set to 85.
  analogWrite(prop_pin, 255 / (turnAngle*10))
}

void loop() {
  // need to add to use yaw to control degree of rotation to avoid over turning!!

  // FAN
  fanWork(lift_pin);
  // fanWork(prop_pin);

  // get analog readings
  read_left = stableRead(ir_left);
  read_right = stableRead(ir_right);
  
  // get distance in cm
  distance_left = getDistance(read_left);
  distance_right = getDistance(read_right);

  // Check for location
  prop_control = turn(distance_left, distance_right);
  backFanCTL(prop_control)

  // check for yaw -> if isCentered returns false, then moveServo
  // move servo according to angle
  moveServo(prop_control);

  // console
  Serial.print("LHS: ");
  Serial.print(distance_left);
  Serial.print(" cm, RHS: ");
  Serial.print(distance_right);
  Serial.println(" cm");


  delay(500);
}

// function to keep the hovercraft hovering
bool isHovering() {
  updateAccel();

  // needs to hover above 0.3 cm (3mm)
  if (g_z < 0.3) {

  } else if (g_z > 0.035) { // control height to not waste battery
    
  }
}

// function to track yaw to see whether to rotate the fan or not
bool isCentered() {
  updateGyro();

  // check yaw to see if the hovercraft is centered enough or not
  // if deviation is too small, then returns true
}
