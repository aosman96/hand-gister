
/*
 * MPU6050 to Arduino Compementary Angle Finder
 * 
 * This sketch calculates the angle of an MPU6050 relative to the ground using a complementary filter.
 * It is designed to be easy to understand and easy to use. 
 * 
 * The following are your pin connections for a GY-521 breakout board to Arduino UNO or NANO:
 * MPU6050  UNO/NANO
 * VCC      +5v
 * GND      GND (duh)
 * SCL      A5
 * SDA      A4
 * 
 * Of note: this sketch uses the "I2C Protocol," which allows the Arduino to access multiple pieces of data
 * on the MPU6050 with only two pins.  Way cooler than needing 7 pins for the 7 pieces of data you can get
 * from your MPU6050.  The two wires are a data wire and a clock wire. The "Wire.h" library does most of this 
 * communication between the Arduino and the MPU6050. It requires that the Arduino use the A5 adn A4 pins.
 * Other Arduinos (Arduini?) may use different pins for the I2C protocol, so look it up on the Arduino.cc website. 
 * The I2C protocol works this way like a conversation between the Arduino and the MPU.  It goes something like this:
 * Arduino: "Hey 0x68." (0x68 is the MPU's address)
 * MPU6050: "wat"
 * Arduino: "Gimme yer x acceleration data." (except Arduino is a computer, so it calls that data "0x3B"
 * MPU6050: "Ugh fine. 16980"
 * Arduino: "K stop now"
 * MPU6050: "K"
 * 
 */
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
const int MPU_addr=0x68;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //These will be the raw data from the MPU6050.
uint32_t timer; //it's a timer, saved as a big-ass unsigned int.  We use it to save times from the "micros()" command and subtract the present time in microseconds from the time stored in timer to calculate the time for each loop.
double compAngleX, compAngleY; //These are the angles in the complementary filter
#define degconvert 57.2957786 //there are like 57 degrees in a radian.
int pitchvalue , rollvalue ;
int left = 4;
int right = 3;
int leftR =0 ,rightR =0 ;
int buzzer = 6;
int bumb = 5 ;
byte gotbyte = 0 ;
byte bytestored = 56;
/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 1;
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7,8);
/**********************************************************/
byte addresses[][6] = {"1Node","2Node"};              // Radio pipe addresses for the 2 nodes to communicate.
// Timers
byte tobesend ;
void setup() 
{
  Serial.begin(115200);
  //for transimitter 
  radio.begin();

  radio.enableAckPayload();                     // Allow optional ack payloads
  radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads
  
    radio.openWritingPipe(addresses[1]);        // Both radios listen on the same pipes by default, but opposite addresses
    radio.openReadingPipe(1,addresses[0]);      // Open a reading pipe on address 0, pipe 1
  /////////////
   Wire.begin();
  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(100);

  //setup starting angle
  //1) collect the data
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //2) calculate pitch and roll
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  //3) set the starting angle to this pitch and roll
  double gyroXangle = roll;
  double gyroYangle = pitch;
  double compAngleX = roll;
  double compAngleY = pitch;
  pinMode(left,INPUT);
  pinMode(right,INPUT);
  pinMode (buzzer,OUTPUT);
  //start a timer
  timer = micros();
 
}
void (*reset)(void)=0;
void loop()
{


//Now begins the main loop. 
  //Collect raw data from the sensor.
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  if(!rightR)
  leftR=digitalRead(left);
  if(!leftR)
  rightR=digitalRead(right);
  double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
  timer = micros(); //start the timer again so that we can calculate the next dt.

  //the next two lines calculate the orientation of the accelerometer relative to the earth and convert the output of atan2 from radians to degrees
  //We will use this data to correct any cumulative errors in the orientation that the gyroscope develops.
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  //The gyroscope outputs angular velocities.  To convert these velocities from the raw data to deg/second, divide by 131.  
  //Notice, we're dividing by a double "131.0" instead of the int 131.
  double gyroXrate = GyX/131.0;
  double gyroYrate = GyY/131.0;

  //THE COMPLEMENTARY FILTER
  //This filter calculates the angle based MOSTLY on integrating the angular velocity to an angular displacement.
  //dt, recall, is the time between gathering data from the MPU6050.  We'll pretend that the 
  //angular velocity has remained constant over the time dt, and multiply angular velocity by 
  //time to get displacement.
  //The filter then adds a small correcting factor from the accelerometer ("roll" or "pitch"), so the gyroscope knows which way is down. 
  compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch; 
  
  //W00T print dat shit out, yo
  Serial.print(compAngleY);Serial.print("\n");
 
 //tazbet el angle
 if(compAngleX > 60 || compAngleX <-60)
 {
    tobesend = bytestored;
    
  if(leftR && tobesend!=0  )
  {
    bytestored = tobesend ;
    Serial.print("leeft");
    Serial.println(tobesend);
  tobesend = tobesend|B01000000;
  
    Serial.print("leeft2");
    Serial.println(tobesend);
 
  }
   if(rightR && tobesend!=0 )
    {   
      bytestored = tobesend ;
      tobesend = tobesend|B10000000;
    }
    if( digitalRead(bumb) )
    {

      bytestored = tobesend ;
       tobesend = tobesend|B00000001;
       }
  
 }
else
{
  if(compAngleY>=-30 && compAngleY<0)
  tobesend =B00000000 ;
      
  else if(compAngleY<=30 && compAngleY>=0)
  tobesend =B00000000 ;

  else if(compAngleY >30 && compAngleY <=60 )
    tobesend=B00010000;
  
  else if(compAngleY < -30 && compAngleY >=-60 )
    tobesend=B00110000; 
  
   else if(compAngleY >60  )
    tobesend=B00011000;
  
  else if(compAngleY <-60   )
  tobesend=B00111000;

  
  if(leftR && tobesend!=0  )
  {

    Serial.print("leeft");
    Serial.println(tobesend);
  tobesend = tobesend|B01000000;
  
    Serial.print("leeft2");
    Serial.println(tobesend);
 
  }
   if(rightR && tobesend!=0 )
       tobesend = tobesend|B10000000;
    
    if( digitalRead(bumb) )
    {
       tobesend = tobesend|B00000001;
       }
                                  
   // digitalWrite(buzzer,rightR);
}
Serial.print(F("Now sending "));                         // Use a simple byte counter as payload
    Serial.println(tobesend);
/**************************************************************************************************/
 // Send data
  byte Byteack;                                           // Initialize a variable for the incoming response
    bool timeout = false;
    Serial.print(F("Now sending "));                         // Use a simple byte counter as payload
    Serial.println(tobesend);
  unsigned long sending_time = millis();
    radio.write(&tobesend,1) ;
    if (  radio.available()){                         // Send the counter variable to the other radio  
    radio.read( &gotbyte, 1 );                  // Read it, and display the response time
   }        
      if(gotbyte == 1)
       digitalWrite(buzzer,1);
       else
      digitalWrite(buzzer,0);
}



