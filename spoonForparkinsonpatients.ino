#include <Wire.h>
#include <Servo.h>

Servo myservoX;  // create servo object to control a servo
Servo myservoY;  // create servo object to control a servo

//variables to store values from mpu6050

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
float gyroX_cal, gyroY_cal, gyroZ_cal;
float angle_pitch, angle_roll;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
int acc_calibration_value = 1000;                            //Enter the accelerometer calibration value
float angle_acc;

long loop_timer;
int servoXpos=90;
int servoYpos=90;
int count = 0;

void setup(){
  Serial.begin(9600);

  myservoX.attach(9);   // connect servo at X axis(roll) to pin 9  
  myservoY.attach(10);  // connect servo at Y axis(pitch) to pin 10
  myservoX.write(90);   //set starting position of both servo motors at 90 degree
  myservoY.write(90);
  Wire.begin();
  setupMPU();       //set mpu6050 registers
  //delay(1000);
  pinMode(13,OUTPUT);
  Serial.println("caliberating MPU6050");  
  for(int i=0; i<2000; i++){
    if(i %125 == 0){Serial.print("."); }
    recordGyroRegisters();
    gyroX_cal += gyroX;
    gyroY_cal += gyroY;
    gyroZ_cal += gyroZ;
    delayMicroseconds(3700);
  }
  gyroX_cal /= 2000;
  gyroY_cal /= 2000;
  gyroZ_cal /= 2000;
  Serial.print("gyroX_cal: ");
  Serial.print(gyroX_cal);
  Serial.print("  gyroY_cal: ");
  Serial.print(gyroY_cal);
  Serial.print("  gyroZ_cal: ");
  Serial.print(gyroZ_cal);
  delay(500); //Short delay to allow the serial communication to finish
  //###########
  Serial.end(); //Closing the serial communication to save CPU resources
  //###########
  loop_timer = micros();
}

void loop(){
  recordAccelRegisters();
  recordGyroRegisters();
  
  gyroX -= gyroX_cal;
  gyroY -= gyroY_cal;
  gyroZ -= gyroZ_cal;
  //printData();
  angle_pitch += gyroX * 0.000122;
  angle_roll += gyroY * 0.000122;

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  //angle_pitch += angle_roll * sin(gyroZ * 0.000002131);               //If the IMU has yawed transfer the roll angle to the pitch angel
  //angle_roll -= angle_pitch * sin(gyroZ * 0.000002131);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //######################################################################################################################################
  //Due to the fact, that the datatype float can only handle 6-7 digits (in total) of precision it would be better to increase the constant factor by 1000
  //After the calculation the result gets divided by 1000 again:
  angle_pitch += angle_roll * sin((gyroZ * 0.002131)/1000.0);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin((gyroZ * 0.002131)/1000.0);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //angle_pitch = constrain(angle_pitch, -90.00, 90.00);
  //angle_roll = constrain(angle_roll, -90.00, 90.00);
  
  //The map function uses integer math and therefore might suppress fractions:
  //servoXpos = map(angle_roll, 90.00,-90.00,0,180);
  //servoYpos = map(angle_pitch, -90.00,90.00,0,180);
  
  //############################################################################################################################
  //More precise solution (uses Microseconds for the Servo.writeMicroseconds()-function, which is also faster and more precise):
  //500us translate to 0° and 2500° translate to 180° of servo angle:
  servoXpos = ((angle_roll - 90.0)*(2500.0 - 500.0) / (-90.0 - 90.0)) + 500.0;
  servoYpos = ((angle_pitch - (-90.0))*(2500.0 - 500.0) / (90.0 - (-90.0))) + 500.0;  
    
    
   count++;
   while(micros() - loop_timer < 8000);{
    if(count==1){
      if(servoXpos >=500.0 && servoXpos <=2500.0){ //setting 500us and 2500us as 0° and 180° limits
         //myservoX.write(servoXpos );
           myservoX.writeMicroseconds(servoXpos);
        }
     }
    if(count==2){
      count=0;
      if(servoYpos >=500.0 && servoYpos <=2500.0){ //setting 500us and 2500us as 0° and 180° limits
         //myservoY.write(servoYpos);
           myservoY.writeMicroseconds(servoYpos);
      }
     }
   }
  loop_timer += 8000;
  }
  
void setupMPU(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
 /* Wire.beginTransmission(0b1101000);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(); */
}

//function to read accelerometer values from mpu6050
void recordAccelRegisters(){
  Wire.beginTransmission(0b1101000);  
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
}

//function to read gyro values from mpu6050
void recordGyroRegisters(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}
