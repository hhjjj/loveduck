// MPU-9250 sensor
// foremost define!!!!!
#define SENSORS_MPU9250_ATTACHED

#include "simple-OSC.h"
#include "Sensors.h"
#include "neopixel.h"
#include "SparkIntervalTimer.h"

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

// FLORA neopixel
#define PIXEL_PIN D2
#define PIXEL_COUNT 4
#define PIXEL_TYPE WS2812B

// MODE
typedef enum
{
  MODE_STANDBY = 0,
  MODE_PATIENCE,
  MODE_LOVELY,
  MODE_CRUSH,
  MODE_SYNC,

} LOVEDUCK_MODES;

LOVEDUCK_MODES duck_mode;

// SYNC Sub-Mode
typedef enum
{
  SUBMODE_STANDBY = 0,
  SUBMODE_SYNCLEFT,
  SUBMODE_SYNCRIGHT,
  SUBMODE_SYNCDOWN,
  SUBMODE_SYNCJUMP,

} SYNC_MODES;

SYNC_MODES sync_submode;

// pin declaration
const uint8_t neopixelDataPin = D2;
const uint8_t buzzerPin = D3;
const uint8_t intPin = D4;
const uint8_t touch_A_Pin = D5;
const uint8_t touch_B_Pin = D6;
const uint8_t ledPin = D7;

const uint8_t FSRAnalogPin = A0;
const uint8_t whisperAnalogPin = A1;
// const uint8_t pot_B_Pin = A2;
// const uint8_t pot_A_Pin = A3;
const uint8_t piezoAnalogPin = A4;
const uint8_t tearAnalogPin = A5;

// sensor values
volatile int FSRValue;
volatile int piezoValue;
volatile int tearValue;
volatile int whisperValue;

volatile float ax, ay, az, gx, gy, gz, mx, my, mz, azimuth;

volatile bool touch_A_OK;
volatile bool touch_B_OK;
volatile bool accel_OK;
volatile bool gyro_OK;
volatile bool FSR_OK;
volatile bool whisper_OK;
volatile bool piezo_OK;
volatile bool tear_OK;

volatile bool sendOSC_OK;

volatile bool sendAck_OK;

// Timer
IntervalTimer Timer;

// udp
UDP udp;

// outbound ip
IPAddress outIP(192, 168, 100, 10);        // braodcast ip addess
unsigned int outPort = 8000;                // port

// neopixel
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

void initPort();
void initWifi();
void initBoard();
void setMode(LOVEDUCK_MODES mode);
void checkOSCMsg();
void sendOSCMsg();

void checkSensors();

void checkAccel();
void checkGyro();
void checkMagneto();
void checkFSR();
void checkTouch_A();
void checkTouch_B();
void checkPiezo();
void checkTear();
void checkWhisper();

void PING(OSCMessage &inMessage);
void setModeToStandby(OSCMessage &inMessage);
void setModeToPatience(OSCMessage &inMessage);
void setModeToLovely(OSCMessage &inMessage);
void setModeToCrush(OSCMessage &inMessage);
void setModeToSync(OSCMessage &inMessage);

void sendAck();

// change lover_name and static ip for members
// "kh": ip 192, 168, 100, 101
// "hc": ip 192, 168, 100, 102
// "hj": ip 192, 168, 100, 103
// "sy": ip 192, 168, 100, 104
// "yj": ip 192, 168, 100, 105

const String lover_name = "sy";

void setup()
{
  initPort();
  initWifi();
  initBoard();

  int i;
  for (i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, strip.Color(0,0,0));
  }
  strip.show();


  tone(buzzerPin, 4000,100);
  delay(600);

  for (i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, strip.Color(255,255,255));
  }
  strip.show();



  tone(buzzerPin, 4000,100);
  delay(600);
  for (i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, strip.Color(0,0,0));
  }
  strip.show();

}

void loop()
{
  // check incoming OSC messages and
  // set current MODE
  checkOSCMsg();
  checkSensors();
  if(sendOSC_OK == true)
  {
    sendOSCMsg();
    sendOSC_OK = false;
  }

  if(sendAck_OK == true)
  {
    sendAck();
    sendAck_OK = false;
  }

	Particle.process();
}


void initPort()
{
  pinMode(neopixelDataPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(intPin, INPUT);
  pinMode(touch_A_Pin, INPUT);
  pinMode(touch_B_Pin, INPUT);
  pinMode(ledPin, OUTPUT);

  // neopiex led
  strip.begin();
  strip.show();
}

void initWifi()
{

  // static IP setting
  // change lover_name and static ip for members
  // "kh": ip 192, 168, 100, 101
  // "hc": ip 192, 168, 100, 102
  // "hj": ip 192, 168, 100, 103
  // "sy": ip 192, 168, 100, 104
  // "yj": ip 192, 168, 100, 105
  IPAddress myAddress(192, 168, 100, 104);
  IPAddress netmask(255, 255, 255, 0);
  IPAddress gateway(192, 168, 100, 1);
  IPAddress dns(192, 168, 100, 1);
  WiFi.setStaticIP(myAddress, netmask, gateway, dns);
  WiFi.useStaticIP();

  // wifi gogo!
  WiFi.connect();

  while(!WiFi.ready()){
    Particle.process();
    delay(500);
  }
  Particle.process();
}

void initBoard()
{
  touch_A_OK = false;
  touch_B_OK = false;
  accel_OK = false;
  gyro_OK = false;
  FSR_OK = false;
  whisper_OK = false;
  piezo_OK = false;
  tear_OK = false;

  sendOSC_OK = false;
  sendAck_OK = false;

  // serial is used for debug only
  // Serial.begin(115200);

  // udp begin for OSC communication
  udp.begin(8001);

  // set i2c and init MPU-9250
  Wire.setSpeed(CLOCK_SPEED_400KHZ);
  Wire.begin();
  Sensors::initialize();

  setMode(MODE_STANDBY);

  // setup and start the timer using TIMER6
  // to avoid timer confliction
  // run update() every 100ms
  Timer.begin(update, 100*2, hmSec, TIMER6);

}

void update()
{
  //Serial.println("update");

  // send OSCMsg every 100 ms
  sendOSC_OK = true;


}
void setMode(LOVEDUCK_MODES mode)
{
  duck_mode = mode;
}

void sendOSCMsg()
{
  if (touch_A_OK == true)
  {
    delay(1);
    touch_A_OK = false;
  }

  if (touch_B_OK == true)
  {
    delay(1);
    touch_B_OK = false;
  }

  if (accel_OK == true)
  {
    pixelOn();
    OSCMessage outMessage("/accel");
    outMessage.addInt(1);
    outMessage.send(udp,outIP,outPort);
    delay(1);
    pixelOff();
    accel_OK = false;
  }

  if (gyro_OK == true)
  {
    OSCMessage outMessage("/gyro");
    outMessage.addInt(1);
    outMessage.send(udp,outIP,outPort);
    delay(1);
    gyro_OK = false;
  }

  if (FSR_OK == true)
  {
    OSCMessage outMessage("/force");
    outMessage.addInt(1);
    outMessage.send(udp,outIP,outPort);
    delay(1);
    FSR_OK = false;
  }

  if (whisper_OK == true)
  {
    pixelOn();
    OSCMessage outMessage("/wind");
    outMessage.addInt(1);
    outMessage.send(udp,outIP,outPort);
    delay(1);
    pixelOff();
    whisper_OK = false;
  }

  if (piezo_OK == true)
  {
    pixelOn();
    OSCMessage outMessage("/piezo");
    outMessage.addInt(1);
    outMessage.send(udp,outIP,outPort);
    delay(1);
    pixelOff();
    piezo_OK = false;
  }

  if (tear_OK == true)
  {
    delay(1);
    tear_OK = false;
  }

}

void checkOSCMsg()
{
  int size = 0;
  OSCMessage inMessage;
  if ( ( size = udp.parsePacket()) > 0)
  {
        while (size--)
      {
          inMessage.fill(udp.read());
      }
      if( inMessage.parse())
      {
          inMessage.route("/ping", PING);
          inMessage.route("/standby", setModeToStandby);
          inMessage.route("/patience", setModeToPatience);
          inMessage.route("/cute", setModeToLovely);
          inMessage.route("/crush", setModeToCrush);
          inMessage.route("/sync", setModeToSync);
      }
  }
}

void checkSensors()
{
  switch (duck_mode) {
    case MODE_STANDBY:
      // checkAccel();
      // checkGyro();
      // checkFSR();
      // checkWhisper();
      // checkPiezo();
      //Serial.println("STANDBY");
      break;

    case MODE_PATIENCE:
      //Serial.println("PATIENCE");
      break;

    case MODE_LOVELY:
      //Serial.println("LOVELY");
      break;

    case MODE_CRUSH:
      //Serial.println("CRUSH");
      break;

    case MODE_SYNC:
      //Serial.println("SYNC");
      break;

    default:
      break;
  }

}

void checkAccel()
{

  Accelerometer *accelerometer = Sensors::getAccelerometer();
  if(accelerometer) {
      Vector3 a = accelerometer->getAcceleration();
      ax = a.x;
      ay = a.y;
      az = a.z;
      //Serial.printlnf("Acceleration (m/s^2)  %+7.3f, %+7.3f, %+7.3f", a.x, a.y, a.z);

      if (ax > 18.0 || ay > 18.0 || az > 18.0) // 9.8-10 is gravity.
      {
        accel_OK = true;
      }

      if (ax < -18.0 || ay < -18.0 || az < -18.0) // 9.8-10 is gravity.
      {
        accel_OK = true;
      }

  }

}

void checkGyro()
{
  Gyroscope *gyroscope = Sensors::getGyroscope();
  if(gyroscope) {
      Vector3 g = gyroscope->getRotation();
      gx = g.x;
      gy = g.y;
      gz = g.z;
      //Serial.printlnf("Rotation (rad/s)      %+7.3f, %+7.3f, %+7.3f", g.x, g.y, g.z);
      if (gx > 4.3 || gy > 4.3 || gz > 4.3) // 4.3 is almost maximum
      {
        gyro_OK = true;
      }

      if (gx < -4.3 || gy < -4.3 || gz < -4.3) // 4.3 is almost maximum
      {
        gyro_OK = true;
      }


  }
}

void checkMagneto()
{
  Magnetometer *magnetometer = Sensors::getMagnetometer();
  if(magnetometer) {
      Vector3 m = magnetometer->getMagneticField();

      mx = m.x;
      my = m.y;
      mz = m.z;
      // Serial.printlnf("Magnetic Field (uT)   %+7.3f, %+7.3f, %+7.3f", m.x, m.y, m.z);

      azimuth = magnetometer->getAzimuth(); // value 0 - 360

      //Serial.printlnf("Azimuth (deg)         %+7.3f", azimuth);

  }
}

void checkFSR()
{
  FSRValue = analogRead(FSRAnalogPin);
  //Serial.println(FSRValue);
  if(FSRValue > 3000)
  {
    FSR_OK = true;
  }
}

void checkTouch_A()
{
  if (digitalRead(touch_A_Pin) == HIGH)
  {
    //Serial.println("touch A");
    touch_A_OK = true;
  }
}

void checkTouch_B()
{
  if (digitalRead(touch_B_Pin) == HIGH)
  {
    //Serial.println("touch B");
    touch_B_OK = true;
  }
}

void checkPiezo()
{
  piezoValue = analogRead(piezoAnalogPin);
  //Serial.println(piezoValue);
  if(piezoValue > 500)
  {
    piezo_OK = true;
  }
}

void checkTear()
{
  tearValue = analogRead(tearAnalogPin);
  //Serial.println(tearValue);
  if(tearValue > 2000)
  {
    tear_OK = true;
  }
}

void checkWhisper()
{
  whisperValue = analogRead(whisperAnalogPin);
  //Serial.println(whisperValue);
  if(whisperValue > 1000)
  {
    whisper_OK = true;
  }
}

void PING(OSCMessage &inMessage)
{
    //Serial.println("/ping");
//Do something
  sendAck_OK = true;
}

void setModeToStandby(OSCMessage &inMessage)
{
    //Serial.println("/standby");
    setMode(MODE_STANDBY);
//Do something
}

void setModeToPatience(OSCMessage &inMessage)
{
    //Serial.println("/patience");
    setMode(MODE_PATIENCE);
//Do something
}

void setModeToLovely(OSCMessage &inMessage)
{
    //Serial.println("/lovely");
    setMode(MODE_LOVELY);
//Do something
}

void setModeToCrush(OSCMessage &inMessage)
{
    //Serial.println("/crush");
    setMode(MODE_CRUSH);
//Do something
}

void setModeToSync(OSCMessage &inMessage)
{
    //Serial.println("/sync");
    setMode(MODE_SYNC);
//Do something
}

void sendAck()
{
  //Serial.println("ping received!");
  OSCMessage outMessage("/ok");
  outMessage.addString(lover_name);
  outMessage.send(udp,outIP,outPort);
  delay(1);
}

void pixelOn()
{
  int i;
  for (i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, strip.Color(255,0,0));
  }
  strip.show();
}

void pixelOff()
{
  int i;
  for (i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, strip.Color(0,0,0));
  }
  strip.show();
}
