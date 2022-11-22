#include <Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>

#define F_CPU 16000000UL

// Averaging filter
// If you use an ultrasonic sensor (US) in analog mode, this value MUST be set to 1 as per the sensor's manufacturer application notes.
// If you wish to filter the values from US sensor, you should use median or mode filter.
// If you use an IR sensor, you can set it to a reasonable value, something between 4 and 10 should work well.
#define ADC_sample_max 4

// Distance threshold for IR sensor. Change it for the US one.
// Note that IR and US sensors have different distance-voltage curves.
// You can use the "ADC_DEBUG" option (see below) to get the corresponding ADC reading.
#define DIST_TH 76
// 1.5/5*255

// Threshold for the distance readings variation
#define DELTA 5

//un-comment the line below to enable printing of ADC readings through the serial port
// #define ADC_DEBUG
// COM port settings: 9600, 8-N-1, None
// Note 1: for this connection, the flow control must be set to "None".
// Note 2: Use any terminal software to display the data (e.g. Hyperterminal)
// Note 3: Arduino IDE must be closed before launching the terminal, 
// and the terminal must be closed before launching Arduino IDE. I.e. only one software should access the serial port.
// Note 4: Since unconnected ADC inputs are floating, the displayed values of unconnected channels are random.
//***
#define DELAY_20ms(x)   TIMSK1&=~(1<<ICIE1);\
                      delay_ms=x;\
                      TIMSK1|=(1<<ICIE1);\
                      while (delay_ms<=x)

// SERVO
Servo hover_servo; //initialize servo

// IMU initialization
MPU6050 mpu(Wire);

//timer (mpu)
unsigned long timer = 0;
  
                  

static volatile uint8_t RX_buff, servo_idx, ADC_sample, V_batt;
static volatile uint16_t time, delay_ms, ADC_acc; 
volatile struct {
  uint8_t TX_finished:1;
  uint8_t sample:1;
  uint8_t mode:1;
  uint8_t stop:1;
//  uint8_t ADC_ready:1;
  uint8_t T1_ovf0:2;
  uint8_t T1_ovf1:2;
} flags;
const int ledPin =  LED_BUILTIN;

//setup command
void setup() {

  // setup
  Serial.begin(9600); //starts the serial at 9600 baud rate
  gpio_init();
  DDRC|=(1<<PC1); // Set pin to output
  DDRB|=(1<<PB5)|(1<<PB0); // set the pin as output
  pinMode(PB5, OUTPUT);

  // setup MPU
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  // calibrating YAW
  while (status != 0) {
    Serial.print(F("Calibrating offsets - DO NOT MOVE IMU"));
    delay(1000);
    mpu.calcOffsets();
    hover_servo.write(1500); // sets servo to neutral position
    Serial.println("Done!\n");
  }

  // Servo setup
  hover_servo.attach(9, 1000, 2000); // set servo to go from 0 degree to 2000 degree
  hover_servo.write(1500); // set servo to neutral position facing forward
  
}

#define TrigPin PB3
#define EchoPin PD2
#define MAX_DIST 40

// DEF SONAR
//NewPing sonar (TrigPin, EchoPin, MAX_DIST);

double UNIT_TO_VOLTAGE = 0.0049;

int getTurnDirection(){
  uint16_t adcReadLeft = analogRead(PC1); // port 8

  uint16_t adcReadRight = analogRead(PC2); // port 14
  int turn = -1;
  // TODO:: flip the signs if the servo reacts opposite to expected.
  if( adcReadLeft > adcReadRight) {
    turn =  ( ( adcReadLeft - adcReadRight ) / adcReadLeft) * 90
  }
  else if(adcReadLeft < adcReadRight) {
    turn = 90 + ( ( adcReadLeft - adcReadRight ) / adcReadRight) * (90)
  }
  // If it's lower than 10 degrees, we probably don't care.
  if(turn >= 10 ){
    return turn;
  }
  else {
    return 0;
  }
  
}

void loop() {
    // READING VOLTAGE FOR IR SENSOR
    read
  

  // Serial.print(UNIT_TO_VOLTAGE*value);
  // Serial.println(" V");
  
  // double distanceCm = get_gp2d12(value);
  // Serial.print(distanceCm - 40);
  // Serial.println(" cm");

  // if(distanceCm >40.00) {
  //   analogWrite(PB5,HIGH);
  // }
  // else{
  //   analogWrite(PB5,LOW);
  // }
  // _delay_ms(500);

  // IMU MPU6050 code

  /*
  little explanation: the new 0 keeps changing -> IT IS NORMAL!!! there's a constant recalibration done depending on current position of the hover craft. So we want the hovercraft to be able to move around by +- 90 degrees from CURRENT location.
  */
  mpu.update();
  if ((millis() - timer) > 10) {
    double current_yaw = mpu.getAngleZ();
    Serial.print("Z: ");
    Serial.println(current_yaw);

    //TODO: convert current yaw -> ms
    // if -90 < z < 90 degrees -> set servo to current_yaw value
    if (-90.00 <= current_yaw <= 90.00) {
      hover_servo.write(current_yaw);
    }
    Serial.print("Servo angle: ");
    Serial.println(hover_servo.read());
  
    timer = millis();
  }
  _delay_ms(50);
  
}
uint16_t adcToCm (uint16_t x) {
    // This formula was found at: https://wiki.dfrobot.com/Sharp_GP2Y0A21_Distance_Sensor__10-80cm___SKU_SEN0014_
    // I think it's in mm so  /10
    return  2.54 /( (1.983773565816502e-04 + (7.131755812731296e-04) * x + (1.538397242051181e-03) * x ^ 2 - (3.189883571136305e-04) * x ^ 3 + (2.894171139886596
e-05) * x ^ 4 - (1.204350042380299e-06) * x ^ 5 + (1.906621854559469e-08) * x ^ 6) )
}
