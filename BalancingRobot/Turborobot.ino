//-----------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------//
//-------------------------------------TURBOROBOT__TURBO_TEAM------------------------------------//
//-----------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------//

#include <Wire.h>
#include "Kalman.h"
#include "Turborobot.h"

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data


void setup() {
  Serial.begin(115200);
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2;       // Set I2C frequency to 400kHz
  
  /* Setup motor pins to output */
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  
  /*Config the PWM mode*/
  TCCR1A = _BV(COM1A1) | _BV(COM1B1);
  TCCR1B = _BV(WGM13) | _BV(CS10);   // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1H = (PWMVALUE >> 8);           // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz
  ICR1L = (PWMVALUE & 0xFF);
  
  TCCR4A = _BV(COM4A1) | _BV(COM4B1);
  TCCR4B = _BV(WGM43) | _BV(CS40);   // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR4H = (PWMVALUE >> 8);           // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz
  ICR4L = (PWMVALUE & 0xFF);
  /*----------------------------------*/
  /*-----------------------------------------------------------------------------------------------------------------*/
  
  i2cData[0] = 7;                            // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x05;                         // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00;                         // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00;                         // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true));        // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) {                  // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angles */
  while (i2cRead(0x3B, i2cData, 6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  // We then convert it to 0 to 2π and then from radians to degrees
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;

  // Set starting angles
  kalmanX.setAngle(accXangle); 
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  compAngleX = accXangle;
  compAngleY = accYangle;

  timer = micros();
}

void loop ()
{
  Update_Angle();
  
  if((layingdown && (pitch < 170 || pitch > 190)) || (!layingdown && (pitch < 135 || pitch > 225))) 
  {
    layingdown = 1;  // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
  }
 else 
  { 
    layingdown = 0; // It's no longer laying down
    PID(target_angle);        
  }
  int motor1 = analogRead(10);
  int motor2 = analogRead(11);
  Serial.print(motor1); Serial.print("\t");
  Serial.print(motor2); Serial.print("\t");
  Serial.print(pitch); Serial.print("\t");
  Serial.print("\r\n"); // \r : carriage return respectively
  delay(1);
}
/*----------------------------------------------------------*/

void PID(double restAngle)
{
  double error = (restAngle - pitch);
  /*-------------------------------------*/
  if (error > 0) // backward
  PID_Tuning(80,0,0);//PID_Tuning(80, 8.5, -50);
  else if (error < 0)// forward
  PID_Tuning(80,0,0);//PID_Tuning(80, 14.5, -50);
  /*-------------------------------------*/
  double pTerm = Kp * error;
  iTerm += Ki * error;
  double dTerm = Kd * (error - lastError);
  lastError = error;
  double PIDValue = pTerm + iTerm + dTerm;
  PIDValue = output_constrain(PIDValue, 800, -800);
  // Set motor speed
  if (PIDValue >= 0)
  {
    motor_left(backward, PIDValue);
    motor_right(backward, PIDValue);
  }
  else
  {
    motor_left(forward, PIDValue * (-1));
    motor_right(forward, PIDValue * (-1));    
  }
  //------------------- Print data--------------------------//
 
 
  Serial.print(PIDValue); Serial.print("\t");
  //Serial.print(PID_Forward); Serial.print("\t");
  //Serial.print(dTerm); Serial.print("\t");
  //Serial.print("\r\n"); // \r : carriage return respectively
  //delay(1);
}
//--------------------------------------------------------//
// Left motor controller. Using pin 11, 12 to PWM output--//
//--------------------------------------------------------//
void motor_left(int _direction, int _speed)
{
  int _PWM;
  int _PWM_scale;
  _PWM = (_speed + motor_offset_left);
  _PWM_scale = _PWM ; // synchronous the motor speed
  if (_direction)
  {
    OCR1AH = (_PWM_scale >> 8); 
    OCR1AL = (_PWM_scale & 0xFF);
    OCR1BH = (0 >> 8); 
    OCR1BL = (0 & 0xFF);
  }
  else if (!_direction)
  {
    OCR1AH = (0 >> 8); 
    OCR1AL = (0 & 0xFF);
    OCR1BH = (_PWM >> 8); 
    OCR1BL = (_PWM & 0xFF);
  }
}
//--------------------------------------------------------//
// Right motor controller. Using pin 6, 7 to PWM output---//
//--------------------------------------------------------//
void motor_right(int _direction, int _speed)
{
  int _PWM;
  int _PWM_scale;
  _PWM = (_speed   + motor_offset_right );
  _PWM_scale = _PWM * 0.9 ;  // synchronous the motor speed
   if (_direction)
    {
    OCR4AH = (0 >> 8); 
    OCR4AL = (0 & 0xFF);
    OCR4BH = (_PWM_scale >> 8); 
    OCR4BL = (_PWM_scale & 0xFF);
    }
  else if (!_direction)
    {
    OCR4AH = (_PWM >> 8); 
    OCR4AL = (_PWM & 0xFF);
    OCR4BH = (0 >> 8); 
    OCR4BL = (0 & 0xFF);
    }
}
void stopAndReset() {
  // Stop 2 motors and reset the storage
  motor_left(forward, 0);
  motor_right(forward, 0);  
  lastError = 0;
  iTerm = 0;
}
double output_constrain(double value, double max_value, double min_value)
{
  if (value >= max_value)
  value = max_value;
  else if (value < min_value)
  value = min_value;
  return value;
}
void PID_Tuning(double kp, double ki, double kd)
{
  Kp = kp;
  Ki = ki;
  Kd = kd;
}
