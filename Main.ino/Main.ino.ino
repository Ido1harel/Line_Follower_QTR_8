//Pololu sesnor libaries
#include <PololuQTRSensors.h>
#include <AFMotor.h>
//

//APDS color sensor libaries
#include <Wire.h>
#include <SparkFun_APDS9960.h>
//
// Global Variables for APDS color sensor
SparkFun_APDS9960 apds = SparkFun_APDS9960();
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;
//



AF_DCMotor motor1(1, MOTOR12_8KHZ ); // PIN 11 - create motor #1 pwm
AF_DCMotor motor2(2, MOTOR12_8KHZ ); // PIN 3 - create motor #2 pwm

// Change the values below to suit your robot's motors, weight, wheel type, etc.
#define KP .2
#define KD 5
#define M1_DEFAULT_SPEED 50
#define M2_DEFAULT_SPEED 50
#define M1_MAX_SPEED 70
#define M2_MAX_SPEED 70
#define MIDDLE_SENSOR 2
#define NUM_SENSORS  5      // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
#define DEBUG 0 // set to 1 if serial debug output needed

PololuQTRSensorsRC qtrrc((unsigned char[]) {  18,17,16,15,14} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  delay(1000);
  manual_calibration(); 
  set_motors(0,0);
  
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  
  // Start running the APDS-9960 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
    Serial.println(F("Light sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during light sensor init!"));
  }
}

// Checking the color by APDS sensor.
void loop()
{
  // Read the light levels (ambient, red, green, blue)
  if (  !apds.readAmbientLight(ambient_light) ||
        !apds.readRedLight(red_light) ||
        !apds.readGreenLight(green_light) ||
        !apds.readBlueLight(blue_light) ) {
    Serial.println("Error reading light values");
  } else {
    Serial.print("Ambient: ");
    Serial.print(ambient_light);
    Serial.print(" Red: ");
    Serial.print(red_light);
    Serial.print(" Green: ");
    Serial.print(green_light);
    Serial.print(" Blue: ");
    Serial.println(blue_light);
  }
  // Wait 0.5 second before next reading
  delay(500);
}


//Driving data
int lastError = 0;
int  last_proportional = 0;
int integral = 0;

//Driving data to Pololu sensor
void loop()
{
  unsigned int sensors[5];
  int position = qtrrc.readLine(sensors);
  int error = position - 2000;

  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  int leftMotorSpeed = M1_DEFAULT_SPEED + motorSpeed;
  int rightMotorSpeed = M2_DEFAULT_SPEED - motorSpeed;

  // set motor speeds using the two motor speed variables above
  set_motors(leftMotorSpeed, rightMotorSpeed);
}

void set_motors(int motor1speed, int motor2speed)
{
  if (motor1speed > M1_MAX_SPEED ) motor1speed = M1_MAX_SPEED; // limit top speed
  if (motor2speed > M2_MAX_SPEED ) motor2speed = M2_MAX_SPEED; // limit top speed
  if (motor1speed < 0) motor1speed = 0; // keep motor above 0
  if (motor2speed < 0) motor2speed = 0; // keep motor speed above 0
  motor1.setSpeed(motor1speed);     // set motor speed
  motor2.setSpeed(motor2speed);     // set motor speed
  motor1.run(FORWARD);  
  motor2.run(FORWARD);
}

void manual_calibration()
{
  int i;
  for (i = 0; i < 250; i++)  // the calibration will take a few seconds
  {
    qtrrc.calibrate(QTR_EMITTERS_ON);
    delay(20);
  }

  if (DEBUG) { // if true, generate sensor dats via serial output
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
  }
}



// An example of RBG leds code.
// from red to yellow
  for(int i = 0; i < 255; i++)
  {
    
    analogWrite(ledRed,255); 
    analogWrite(ledGreen,i);  
    analogWrite(ledBlue,0);
    delay(milliseconds);
      
  }

  // from yellow to cyan
  for(int i = 0; i < 255; i++)
  {
    
    analogWrite(ledRed,255-i); 
    analogWrite(ledGreen,255);  
    analogWrite(ledBlue,i); 
    delay(milliseconds);
      
  }

  // from cyan to blue
  for(int i = 0; i < 255; i++)
  {
    
    analogWrite(ledRed,0); 
    analogWrite(ledGreen,255-i);  
    analogWrite(ledBlue,255); 
    delay(milliseconds);
      
  }

  // from blue to magenta
  for(int i = 0; i < 255; i++)
  {
    
    analogWrite(ledRed,i); 
    analogWrite(ledGreen,0);  
    analogWrite(ledBlue,255); 
    delay(milliseconds);
      
  }

  // from magenta to red
  for(int i = 0; i < 255; i++)
  {
    
    analogWrite(ledRed,255); 
    analogWrite(ledGreen,0);  
    analogWrite(ledBlue,255 - i); 
    delay(milliseconds);
      
  }
