#define LOG_INPUT false
#define LOG_OUTPUT false
#define LOG_DMP false
#define LOG_LOOP false
#define PID_LOG false
#define LOG_WIFI false

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
//#include <WiFiUdp.h>

#include "AsyncUDP.h"

#include "PID_v1.h"
#include "LMotorController.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

const char* ssid = "HomeSweetHome";
const char* password = "allUneedisLOvE";

WebServer server(80);

AsyncUDP udp;

// Multicast declarations
IPAddress ipBroadcast(255, 255, 255, 255);
unsigned int portBroadcast = 61565; // port to broadcast on must be between1024...65000
bool gotFirstCommand = false;
unsigned long lastTimeIpSent = 0;
char ipAddressBuff[255];
const int led = 13;
bool needResetFIFO = false;

#define MIN_ABS_SPEED 1

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[1300]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

double originalSetpoint = 246.5;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
int moveState=0; //0 = balance; 1 = back; 2 = forth
double Kp = 54;
double Kd = 2;
double Ki = 100;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 1;
double motorSpeedFactorRight = 1;

bool operating = true;
double leftShift = 0.0;
double rightShift = 0.0;
int minAbsSpeed = 20;

// PINOUTS

// mcu6050 (gravity): 
// VCC
// GND
// D1
// D2
// nc
// nc
// nc
// D8

int INTERRUPT_PIN = 27;

//MOTOR CONTROLLER
int ENA = 4;  //D4
int IN1 = 12; //D5
int IN2 = 14; //D6 
int IN3 = 32; //D7
int IN4 = 33; //D0
int ENB = 17;  //RX

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

//timers
long time1Hz = 0;
long time5Hz = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setupWifi(){
  Serial.println("WIFI SETUP");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }

  IPAddress ip = WiFi.localIP();
  sprintf(ipAddressBuff, "mrBRObot:IP=%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

  Serial.print("UDP BROADCAST: ");
  Serial.println(ipAddressBuff);
  Serial.println("Server setup");

  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });
  
  server.on("/ctrl", ctrlHandleFunc);
  server.on("/", ctrlHandleFunc);
    
  server.begin();
  Serial.println("HTTP server started");
  Serial.println("  --= SETUP DONE =--");


}

void setup()
{
  Serial.begin(115200);

  setupWifi();
  delay(3000);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));

        pinMode(INTERRUPT_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID

       // analogWriteRange(255);
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}


void loop()
{
  server.handleClient(); //this is required for handling the incoming requests

    if (needResetFIFO){
      needResetFIFO = false;
      mpu.resetFIFO();
      fifoCount = 0;
      mpu.setDMPEnabled(true);
    }
    
    if (!gotFirstCommand){
      unsigned long now = millis();
            
      if (now >= lastTimeIpSent + 2000){      
        Serial.println("Sending IP addres via UDP"); 
        lastTimeIpSent = now;
        sendIpAddressViaUDP();
      }
    }
  
      #if LOG_LOOP
      Serial.print(F("."));
      #endif

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

        #if LOG_DMP
        Serial.println(">");
        Serial.print(F("DMPREADY "));
        Serial.print(F("fc: "));
        Serial.print(fifoCount);
        Serial.print(F(" ps: "));
        Serial.print(packetSize);
        Serial.print(F(" mi: "));
        Serial.println(mpuInterrupt);
        #endif

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //no mpu data - performing PID calculations and output to motors
        pid.Compute();

        double lSpeed = output + leftShift;
        double rSpeed = output + rightShift;
        #if LOG_OUTPUT
        Serial.print(F("output "));
        Serial.print(output);
        Serial.print(" lshift: ");
        Serial.print(leftShift);
        Serial.print(" lshift: ");
        Serial.print(rightShift);
        Serial.print(" l: ");
        Serial.print(lSpeed);
        Serial.print(" r: ");
        Serial.println(rSpeed);
        #endif

        if (operating) {
          motorController.move(lSpeed, rSpeed, minAbsSpeed);
        } else {
          motorController.move(0, 0, 0);
        }       
    }
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        fifoCount = 0;

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[1] * 180/M_PI + 180;

        #if LOG_INPUT
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            Serial.print("\t");
            Serial.println(input);
            Serial.print("\t");
        #endif
   }
}

///// WIFI
void ctrlHandleFunc(){
   #if LOG_WIFI
   Serial.println("GOT request:");
   #endif
   mpu.setDMPEnabled(false);
   gotFirstCommand = true;
   if (server.args() > 0){
    for (int i = 0; i < server.args();i++){

        String key = server.argName(i);
        String val = server.arg(i);

        #if LOG_WIFI
        Serial.print("Name: "); Serial.println(key);
        Serial.print("Value: "); Serial.println(val);
        #endif
        
        handleParam(key, val);
      }
   }
   server.send(200, "text/html", "OK");
   #if LOG_WIFI
   Serial.println("END request");
   #endif
   needResetFIFO = true;
}

String KP = "kp";
String KI = "ki";
String KD = "kd";
String LEFT_SHIFT = "lshift";
String RIGHT_SHIFT = "rshift";
String TARGET = "target";
String LEFT_COEF = "lcoef";
String RIGHT_COEF = "rcoef";
String MINSTEP = "minstep";
String SAMPLETIME = "sampletime";
String OPERATE = "operate";

void handleParam(String key, String val){
  val.trim();
  
  if (val.length() == 0) {
    #if LOG_WIFI    
    Serial.print(key); Serial.print(" -> "); Serial.println("DROPPED EMPTY"); 
    #endif
    return;
  }
    
  if (KP.equalsIgnoreCase(key)){
    Kp = val.toFloat();
    pid.SetTunings(Kp, Ki, Kd);
    #if LOG_WIFI
    Serial.print("Kp -> "); Serial.println(Kp); 
    #endif
  } else if (KI.equalsIgnoreCase(key)){
    Ki = val.toFloat();
    pid.SetTunings(Kp, Ki, Kd);
    #if LOG_WIFI
    Serial.print("Ki -> "); Serial.println(Ki); 
    #endif
  } else if (KD.equalsIgnoreCase(key)){
    Kd = val.toFloat();
    pid.SetTunings(Kp, Ki, Kd);
    #if LOG_WIFI
    Serial.print("Kd -> "); Serial.println(Kd); 
    #endif
  } else if (SAMPLETIME.equalsIgnoreCase(key)){
    int stime = val.toInt();
    if (stime >= 1){
       pid.SetSampleTime(stime);
    }
    #if LOG_WIFI
    Serial.print("stime -> "); Serial.println(stime); 
    #endif
  } else if (LEFT_SHIFT.equalsIgnoreCase(key)){
    leftShift = val.toFloat();
    #if LOG_WIFI
    Serial.print("leftShift -> "); Serial.println(leftShift); 
    #endif
  } else if (RIGHT_SHIFT.equalsIgnoreCase(key)){
    rightShift = val.toFloat();
    #if LOG_WIFI
    Serial.print("rightShift -> "); Serial.println(rightShift); 
    #endif
  } else if (TARGET.equalsIgnoreCase(key)){
    setpoint = val.toFloat();
    #if LOG_WIFI
    Serial.print("originalSetpoint -> "); Serial.println(setpoint); 
    #endif
  } else if (RIGHT_COEF.equalsIgnoreCase(key)){
    motorSpeedFactorRight = val.toFloat();
    #if LOG_WIFI
    Serial.print("motorSpeedFactorRight -> "); Serial.println(motorSpeedFactorRight); 
    #endif
  } else if (LEFT_COEF.equalsIgnoreCase(key)){
    motorSpeedFactorLeft = val.toFloat();
    #if LOG_WIFI
    Serial.print("motorSpeedFactorLeft -> "); Serial.println(motorSpeedFactorLeft); 
    #endif
  } else if (MINSTEP.equalsIgnoreCase(key)){
    minAbsSpeed = val.toInt();
    #if LOG_WIFI
    Serial.print("minAbsSpeed -> "); Serial.println(minAbsSpeed); 
    #endif
  } else if (OPERATE.equalsIgnoreCase(key)){
    operating = val.toInt() != 0;
    #if LOG_WIFI
    Serial.print("operating -> "); Serial.println(operating); 
    #endif
  } else {
    #if LOG_WIFI
    Serial.print(key); Serial.print(" -> "); Serial.println("IGNORED"); 
    #endif
  }
}

/// UPD
void sendIpAddressViaUDP() {
  udp.broadcastTo("mrBRObot:IP=", 1234);
//  Udp.beginPacket(ipBroadcast, portBroadcast);
//  Udp.write("mrBRObot:IP=");
//  Udp.write(ipAddressBuff);
//  Udp.endPacket(); 
}
