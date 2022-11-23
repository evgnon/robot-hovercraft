
// Include the library:

// Define model and input pin:
#define ir_left PC2
#define ir_right PC1

// Create variable to store the distance:
float distance_left;
float distance_right;

float read_left;
float read_right;

float prop_control = 90;

float turn(float adcReadLeft, float adcReadRight) {
  float turn;

  // // check if difference between both distances is larger than 5 cm
  // if (((adcReadLeft - adcReadRight) < -5.00) || ((adcReadLeft - adcReadRight) > 5.00)) {

    if (adcReadLeft > adcReadRight) {
      return (1-(adcReadRight / adcReadLeft)) * 90.00;
    }
    else if (adcReadLeft < adcReadRight) {
      return -((1-(adcReadLeft / adcReadRight))* 90.00);
    }
  //}
}
