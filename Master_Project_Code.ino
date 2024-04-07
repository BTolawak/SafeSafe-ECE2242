#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display
Servo myservo;
int pos = 0;    // variable to store the servo position

//assigning pin numbers
int butt1= 8;//buttons
int butt2= 9;
int butt3= 10;

int buzzer = 3;
int soundSensor = 2;
int trigSonic = A0;
int echoSonic = A1;
int servo = 11;
int esp = A2;


//assigning pin numbers to LEDs
int led1= A3;
int led2= 6;

unsigned long previousMillis = 0;        // stores last time LED was updated
unsigned long previousMillis2 = 0;
const long interval = 500;              // interval at which to blink (milliseconds)
bool alarm1 = false;
bool alarm2 = false;
int ledState = 0; //holds last led state, helps to control blinking

String password= "111"; //holds password
String pwTry= ""; //holds current password attempt
int tryCount=0; //counts number of password attempts
int tryLength=0; //counts length of current password attempt, will max out at 3

//array to hold current button states
int currentStates[3]={0,0,0}; 

int nextNum;// next number to be printed
bool lock= true;// state of lock
bool start= true;// determines whether to print opening message


//gyro variables
Adafruit_MPU6050 mpu;

// Thresholds for changes to consider (might need tuning)
float accelThreshold = 5; // Acceleration change threshold
float gyroThreshold = 5.0; // Gyroscope change threshold

// Variables to store the last readings
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
float lastGyroX = 0, lastGyroY = 0, lastGyroZ = 0;

//ultra sonic variables
long duration;
int distance;





void setup() {


 // Wire.begin(); // Join the bus as a master or slave
 // Wire.setClock(50000); // Lower the I2C clock speed to 50kHz


  pinMode(buzzer, OUTPUT);
  pinMode(trigSonic, OUTPUT);
  pinMode(servo, OUTPUT);
  pinMode(esp, OUTPUT);

  pinMode(butt1, INPUT);
  pinMode(butt2, INPUT);
  pinMode(butt3, INPUT);
  pinMode(soundSensor, INPUT);
  pinMode(led1, INPUT);
  pinMode(led2, INPUT);





  // setting up LCD
  lcd.setCursor(0, 0);
  lcd.backlight();
  lcd.display();
  myservo.attach(11);  // attaches the servo on pin 11 to the servo object

    // Initialize Serial Communication
  Serial.begin(9600);
  lcd.init();
  lcd.clear();  
  delay(200);       
  lcd.backlight();      // Make sure backlight is on
  // lcd.print("Booting");
  // delay(500);
  // lcd.print(".");
  // delay(500);
  // lcd.print(".");
  // delay(500);
  // lcd.print(".");
  // delay(500);
  // lcd.clear();
  // lcd.print("Welcome to your");
  // lcd.setCursor(1,1); 
  // lcd.print("SafeSafe");
  // delay(1000);
  // lcd.clear();


  //setupt motion detection
  // mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  // mpu.setMotionDetectionThreshold(1);
  // mpu.setMotionDetectionDuration(20);
  // mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  // mpu.setInterruptPinPolarity(true);
  // mpu.setMotionInterrupt(true);

  // Serial.println("");


  lockSafe();

  delay(1000);
}


// lcd.cursor();

void loop() {
  // put your main code here, to run repeatedly:
  if (start){
    lcd.print("Enter Password:");
    lcd.setCursor(0,1);
    start= false;
  }
  
  
  updateCurrentStates();
  if(findInput()!=-1){ //if a button is pressed, then we print that button
    nextNum = findInput();
    lcd.print(nextNum); //printing current try to lcd
    pwTry.concat(nextNum); //updating pwTry


  }

  if (lock){
    
    // checkGyro();
    // ultraSonic();


    if (pwTry.length() == 3){//once try length equals the 3 we will compare to the password
      delay(1000);
      lcd.clear();
      if (pwTry == password){//if try is correct we open the safe
        noTone(buzzer);     // Stop sound...
        ledOff();
        lcd.print("Safe Opened");
        pwTry="";
        lock = false;
        unlockSafe();
        delay(1000);
        lcd.clear();
        delay(500);
        lcd.print("Enter any 3");
        lcd.setCursor(0,1);
        lcd.print("digits to lock");
        start= false;
        delay(2000);
        lcd.clear();
        tryCount=0;
        alarm1 = false;
      } else {
        lcd.print("Incorrect,");
        lcd.setCursor(0,1); 
        lcd.print("try again");
        delay(2000);
        lcd.clear();
        pwTry="";
        tryCount++;
        start = true;
      }

    }
  }
  if (!lock){
  
    if (pwTry.length() == 3){//once try length equals to 3 we will compare to the password
      delay(100);
      lockSafe();
      lock = true;
      lcd.clear();
      delay(100);
      lcd.print("Safe Locked");
      delay(1000);
      lcd.clear();
      start= true;
      delay(100);
      pwTry="";
      }

    }


  if (tryCount >= 3){
    alarm1 = true;
  }
  if (tryCount == 5){
    buzz();
    digitalWrite(esp,HIGH);
    delay(300);
    digitalWrite(esp,LOW);
  }
  if (alarm1){
    ledBlink();
  } 





  
  delay(500);
}




//checks the state of each button, and stores them in an array
void updateCurrentStates(){
  currentStates[0] = digitalRead(butt1);
  currentStates[1] = digitalRead(butt2);
  currentStates[2] = digitalRead(butt3);
  
}

//method to determine which button was true. returns the index of the button that is true, if none are true, method returns -1
int findInput(){
  for (int i = 0; i<3; i++){
    if (currentStates[i]==1){
      return (i+1);
    }

  }
  return -1;
}

// methods to lock and unlock safe
void lockSafe(){
myservo.write(180);
}
void unlockSafe(){
myservo.write(90);
}


// //level 1 alarm
//void alarm1(){
//  delay(100);
//  lcd.clear();
//  delay(100);
//  lcd.print("First Warning");
  
  
//}


//level 2 alarm
//void alarm2(){
//  
//}

void ledOn(){
  digitalWrite(led1,HIGH);
   digitalWrite(led2,HIGH);
}

void ledOff(){
  digitalWrite(led1,LOW);
  digitalWrite(led2,LOW);
}


void ledBlink(){
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Toggle the LED state
    ledState = !ledState; // This flips the state
    digitalWrite(led1, ledState ? HIGH : LOW); // Update the LED state
    digitalWrite(led2, ledState ? HIGH : LOW);
  }
}

//activates buzzer
void buzz(){
  tone(buzzer, 1000); // Send 1KHz sound signal...
  // delay(500);        // ...for 1 sec
  // noTone(buzzer);     // Stop sound...
  // delay(1000);        // ...for 1sec
}

// bool checkGyro(){
//   if(mpu.getMotionInterruptStatus()) {
//     /* Get new sensor events with the readings */
//     sensors_event_t a, g, temp;
//     mpu.getEvent(&a, &g, &temp);

//     // Check if changes exceed the thresholds
//     if(abs(a.acceleration.x - lastAccelX) > accelThreshold || abs(a.acceleration.y - lastAccelY) > accelThreshold ||
//        abs(a.acceleration.z - lastAccelZ) > accelThreshold || abs(g.gyro.x - lastGyroX) > gyroThreshold ||
//        abs(g.gyro.y - lastGyroY) > gyroThreshold || abs(g.gyro.z - lastGyroZ) > gyroThreshold) {
       
//         // Print the values
//         Serial.print("AccelX:");
//         Serial.print(a.acceleration.x);
//         Serial.print(", AccelY:");
//         Serial.print(a.acceleration.y);
//         Serial.print(", AccelZ:");
//         Serial.print(a.acceleration.z);
//         Serial.print(", GyroX:");
//         Serial.print(g.gyro.x);
//         Serial.print(", GyroY:");
//         Serial.print(g.gyro.y);
//         Serial.print(", GyroZ:");
//         Serial.print(g.gyro.z);
//         Serial.println("");

//         // Update last readings
//         lastAccelX = a.acceleration.x;
//         lastAccelY = a.acceleration.y;
//         lastAccelZ = a.acceleration.z;
//         lastGyroX = g.gyro.x;
//         lastGyroY = g.gyro.y;
//         lastGyroZ = g.gyro.z;

//         espSignal(100);
//     }
//   }

// }


void ultraSonic(){
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

  // Clears the trigPin
  digitalWrite(trigSonic, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigSonic, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigSonic, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoSonic, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  }
}


void espSignal(int time){
  digitalWrite(esp, HIGH);
  delay(time);
  digitalWrite(esp, LOW);
}






