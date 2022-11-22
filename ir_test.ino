
// Include the library:

// Define model and input pin:
#define ir_left PC1
#define ir_right PC2

// Create variable to store the distance:
float distance_left;
float distance_right;

float read_left;
float read_right;

float prop_control = 90;


void setup() {
  // Begin serial communication at a baudrate of 9600:
  Serial.begin(9600);
  gpio_init();

}

void loop() {
  // get analog readings
  read_left = analogRead(ir_left);
  read_right = analogRead(ir_right);
  
  // get distance in cm
  distance_left = getDistance(read_left);
  distance_right = getDistance(read_right);

  // Check for location
  prop_control = turn(distance_left, distance_right);

  // console
  Serial.print("LHS: ");
  Serial.print(distance_left);
  Serial.print(" cm, RHS: ");
  Serial.print(distance_right);
  Serial.println(" cm");
  Serial.print("Rotation angle: ");
  Serial.print(prop_control);
  Serial.println(" degrees");

  delay(1000);
}

float turn(float adcReadLeft, float adcReadRight) {
  float turn = 90.00;
  // TODO:: flip the signs if the servo reacts opposite to expected.
  if( adcReadLeft > adcReadRight) {
    turn = 90.00 + ( ( adcReadLeft - adcReadRight ) / adcReadRight) * 90.00;
  }
  else if(adcReadLeft < adcReadRight) {
    turn = 90.00 - ( ( adcReadLeft - adcReadRight ) / adcReadRight) * 90.00;
  }
  return turn;
}

float getDistance(float reading) {
  float distance = 9462/(reading - 16.92);

	if (distance > 150) return 151;
	else if (distance < 20) return 19;
	else return distance;
}