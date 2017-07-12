#include <Servo.h>
Servo xservo,yservo;

#include <Wire.h>
#include "Kalman.h"
Kalman kalmanX;
Kalman kalmanY;

uint8_t IMUAddress = 0x68; // MPU6050 Address

/* IMU Data */
int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

int moveX,moveY;
int mapX,mapY;
int correctionX,correctionY;

double accXangle;
double accYangle;
double gyroXangle = 9;      
double gyroYangle = 9;
double compAngleX = 90;    
double compAngleY = 90;
double kalAngleX;
double kalAngleY;
uint32_t timer;




// ----------  VOID SETUP START -------------- /
void setup() { 
  Serial.begin(115200);
  xservo.attach(10);
  yservo.attach(11);
  
  Wire.begin(); 
  i2cWrite(0x6B,0x00);           // Disable sleep mode 
  if(i2cRead(0x75,1)[0] != 0x68) {   // Read "WHO_AM_I" register
    Serial.print(F("MPU-6050 with address 0x"));
    Serial.print(IMUAddress,HEX);
    Serial.println(F(" is not connected"));
    while(1);
  }     
  kalmanX.setAngle(90);  // Set starting angle
  kalmanY.setAngle(90);
  timer = micros();
}

// ----------  VOID SETUP END -------------- /


// ---------------------- VOID LOOP START -------------- /
void loop() {
  /* Update all the values */
  uint8_t* data = i2cRead(0x3B,14); 
  accX = ((data[0] << 8) | data[1]);
  accY = ((data[2] << 8) | data[3]);
  accZ = ((data[4] << 8) | data[5]); 
  tempRaw = ((data[6] << 8) | data[7]); 
  gyroX = ((data[8] << 8) | data[9]);
  gyroY = ((data[10] << 8) | data[11]);
  gyroZ = ((data[12] << 8) | data[13]);
 
  /* Calculate the angls based on the different sensors and algorithm */
 
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;  
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;  
  double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = -((double)gyroY/131.0); 
  
  gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter 
  gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
  compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle); // Calculate the angle using a Complimentary filter
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter

  gyroYangle += gyroYrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter 
  gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
  compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle); // Calculate the angle using a Complimentary filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
  
  timer = micros();
  mapX = map(kalAngleX, 0, 200, 0, 179);     //calculate limitation of servo mechanical
  mapY = map(kalAngleY, 0, 200, 0, 179);     //calculate limitation of servo mechanical
 
 
// /////////////////////////////

    correctionX = 0;     // EDIT THIS VALUE FOR SERVO CORRECTION ANGLE// 0 = servo at 90 degrees 
    correctionY = 0;     // EDIT THIS VALUE FOR SERVO CORRECTION ANGLE// 0 = servo at 90 degrees

// ////////////////////////////


  moveX = 270 - (kalAngleX) + correctionX;
  moveY = 270 - (kalAngleY) + correctionY;
 
// ------- SEND TO SERIAL PRINT START ----- /

 // Serial.print(moveX);Serial.print("\t");
 // Serial.print("\n");

//  Serial.print(moveY);Serial.print("\t");
//  Serial.print("\n");
 
// ------- SEND TO SERIAL PRINT END ----- / 




// ------- SEND TO SERVO START ----- /

   xservo.write(moveX);   // Send signal to servo
   Serial.print("moveX= ");
   Serial.println(moveX);
   yservo.write(moveY);   // Send signal to servo
   Serial.print("moveY= ");
   Serial.println(moveY);
   delay(15);     // delay to allow servos to move (ms) 
  
// ------- SEND TO SERVO END ----- /
 
  delay(1); // The accelerometer's maximum samples rate is 1kHz
}
// ---------------------- VOID LOOP END -------------- /



// -- FUNCTIONS START --
void i2cWrite(uint8_t registerAddress, uint8_t data){
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();                       // Send stop
}

uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
  uint8_t data[nbytes]; 
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // Don't release the bus
  Wire.requestFrom(IMUAddress, nbytes);   // Send a repeated start and then release the bus after reading
  for(uint8_t i = 0; i < nbytes; i++)
  data[i] = Wire.read();
  return data;
}
// -- FUNCTIONS END --

// GYROCAM BY SAFT7.COM //
// MODIFIED TO HAVE Y-AXIS (ROLL) BY SHANGLOR //

// END
