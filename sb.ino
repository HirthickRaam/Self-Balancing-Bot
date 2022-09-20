#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Motor A
int motor1Pin1 = 12;
int motor1Pin2 = 14;
int enable1Pin = 33;

// Motor B
int motor2Pin1 = 27;
int motor2Pin2 = 26;
int enable2Pin = 25;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
// resolution is 8, 2^8 = 256. So duty cycle should take values from 0 to 255. Map it accordingly to percentage.
int dutyCycle = 200;

float gyroX, gyroY, gyroZ;

//Gyroscope sensor deviation
float gyroXoffset = 0.07;
float gyroYoffset = 0.03;
float gyroZoffset = 0.01;

// PID constants
double kp = 1000;
double ki = 0;
double kd = 5000000;
float gyroAngle = 0;
unsigned long currentTime, previousTime, previousAccl;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

Adafruit_MPU6050 mpu;

void moveForward(int speedDuty)
{ // speedDuty is an int from 0 to 255

  // setting both motors to 'forward' configuration
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  ledcWrite(pwmChannel, speedDuty);
  //Serial.print("Forward with duty cycle: ");
  //Serial.println(speedDuty);
  // should maintain this signal until it starts rotating in the opposite direction

  //delay(100);
}

void moveBackward(int speedDuty)
{

  // setting both motors to 'forward' configuration
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  ledcWrite(pwmChannel, speedDuty);
  //Serial.print("Backward with duty cycle: ");
  //Serial.println(speedDuty);
  // should maintain this signal until it starts rotating in the opposite direction

  //delay(100);
}

void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  // will pause Zero, Leonardo, etc until serial console opens

  //Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin(0x69))
  {
    //Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  pinMode(enable1Pin, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // configure LED Control PWM functionalities -- setting up a PWM channel to create the signal (there are 16 PWM Channels from 0 to 15)
  // LEDC is used to generate PWM signals, but not necessarily only for LEDs
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled -- channeling the same PWM signal through enable1Pin and enable2Pin
  ledcAttachPin(enable1Pin, pwmChannel);
  ledcAttachPin(enable2Pin, pwmChannel);
}

void loop()
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;  
  mpu.getEvent(&a, &g, &temp);
  
  float gyroX_temp = g.gyro.x;
  if(abs(gyroX_temp) > gyroXoffset)  // If the value is greater than the offset, assume that it is a valid reading
  {
    gyroX += gyroX_temp/50.00;  // theta_x_final = theta_x_initial + w_x*t
  }
  
  

  // NOW NEED TO MAKE IT SELF-BALANCE

  int speed; // duty cycle 0 to 255 for motor

  // PID Control Algorithm

  // Calculate angle of inclination
  currentTime = micros();                             // get current time
  elapsedTime = (double)(currentTime - previousTime); // compute time elapsed from previous computation

 
  error = (gyroAngle - 0);                         // determine error
  cumError += error * elapsedTime;               // compute integral
  rateError = (error - lastError) / elapsedTime; // compute derivative

  double out = kp * error + ki * cumError + kd * rateError; // PID output

/*
  Serial.print("angle ");
  Serial.println(gyroAngle);
  Serial.print("out ");
  Serial.println(out);
*/
  lastError = error; // remember current error
  int base_speed = 200; 
  if (out > 0)
  {
    if (out >=55)
    out = 55;
    // -ve error means + y axis will tilt downwards
    speed = base_speed + (int)((+out));
    moveForward(speed); // along + y
  }
  else if(out==0)
  {
    moveForward(0);
  }
  else
  {
    if (out <= -55)
    out = -55;
    speed = base_speed + (int)(-out);
    moveBackward(speed); // along - y
  }
 

  previousTime = currentTime;
  previousAccl = a.acceleration.z;
}
