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
  MODE_LOVELY,
  MODE_SYNC,
  MODE_CRUSH,
  MODE_TRUST,

} LOVEDUCK_MODES;

LOVEDUCK_MODES duck_mode;

typedef enum
{
  JUMP_STANDBY = 0,
  JUMP_FREEFALL,

} JUMP_MODES;

JUMP_MODES jump_mode;

typedef enum
{
  DOWN_STANDBY = 0,
  DOWN_STILL,
  DOWN_OK,

} DOWN_MODES;

DOWN_MODES down_mode;


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

volatile float tilt_angle;

volatile bool touch_A_OK;
volatile bool touch_B_OK;
volatile bool accel_OK;
volatile bool gyro_OK;
volatile bool FSR_OK;
volatile bool whisper_OK;
volatile bool piezo_OK;
volatile bool tear_OK;

volatile bool gofwd_OK;
volatile bool goback_OK;

volatile bool syncTimeout;
volatile bool syncTimerStart;

volatile bool sendOSC_OK;

volatile bool sendAck_OK;

volatile bool ledOn_Start;
volatile int ledOnCount;
volatile int redValue, blueValue, greenValue;
const int updatePeriod = 100; // 100 ms

// Timer
IntervalTimer Timer;

// sync mode timer
IntervalTimer syncTimer;

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

void readAccel();
void readGyro();
void readMagneto();
void checkFSR();
void checkTouch_A();
void checkTouch_B();
void checkPiezo();
void checkTear();
void readWhisper();

void checkSyncLeft(OSCMessage &inMessage);
void checkSyncRight(OSCMessage &inMessage);
void checkSyncDown(OSCMessage &inMessage);
void checkSyncJump(OSCMessage &inMessage);

void checkSyncTimeout();

void PING(OSCMessage &inMessage);
void setModeToStandby(OSCMessage &inMessage);
void setModeToTrust(OSCMessage &inMessage);
void setModeToLovely(OSCMessage &inMessage);
void setModeToCrush(OSCMessage &inMessage);
void setModeToSync(OSCMessage &inMessage);
void setMainPlayer(OSCMessage &inMessage);
void setRawDataDump(OSCMessage &inMessage);

void disableMainPlayer();

void sendAck();
void dumpRawData();


// change lover_name and static ip for members
// "kh": ip 192, 168, 100, 101
// "hc": ip 192, 168, 100, 102
// "hj": ip 192, 168, 100, 103
// "sy": ip 192, 168, 100, 104
// "yj": ip 192, 168, 100, 105
// left: ip 192, 168, 100, 106
// right: ip 192, 168, 100, 107

const String lover_name = "yj";
String mainPlayerAddr = "/main/" + lover_name;
float mainPlayerFWDVal;

String rawDataLoverAddr = "/dumpdata/" +lover_name;
volatile bool rawdata_OK;

void setup()
{
  initPort();
  initWifi();
  initBoard();
  jump_mode = JUMP_STANDBY;
  down_mode = DOWN_STANDBY;
  sync_submode = SUBMODE_STANDBY;

  setMode(MODE_STANDBY);
  delay(500);
  tone(buzzerPin, 4000,100);
  delay(100);

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

    if (rawdata_OK == true)
    {
      digitalWrite(ledPin, HIGH);
      dumpRawData();
      digitalWrite(ledPin, LOW);
    }

    sendOSC_OK = false;
  }

  if(sendAck_OK == true)
  {
    sendAck();
    sendAck_OK = false;
  }

  if(syncTimerStart == true)
  {
    syncTimer.end();
    syncTimer.begin(checkSyncTimeout, 700*2, hmSec, TIMER7);
    syncTimeout = false;

    syncTimerStart = false;
  }


  // check led for main player
  if (mainPlayerFWDVal > 1.5)
  {
    redValue = 255;
    greenValue = 0;
    blueValue = 0;
  }
  else
  {
    redValue = 0;
    greenValue = 255;
    blueValue = 0;
  }

  if(ledOn_Start)
  {
    // prev led off
    // current led on
    // check when to turn off

    int i;
    for (i = 0; i < strip.numPixels(); i++)
    {
      strip.setPixelColor(i, strip.Color(redValue,greenValue,blueValue));
    }
    strip.show();


    ledOn_Start = false;
  }
  else
  {
    if(ledOnCount == 0)
    {
      pixelAllOff();
      ledOnCount = -1;
    }
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
  // left: ip 192, 168, 100, 106
  // right: ip 192, 168, 100, 107

  IPAddress myAddress(192, 168, 100, 105);
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

  syncTimeout = true;
  syncTimerStart = false;

  gofwd_OK = false;
  goback_OK = false;

  sendOSC_OK = false;
  sendAck_OK = false;

  rawdata_OK = false;

  tilt_angle = 0.0;

  ledOn_Start = false;
  ledOnCount = 0;
  redValue = 0;
  greenValue = 0;
  blueValue = 0;

  // serial is used for debug only
  // Serial.begin(115200);

  // udp begin for OSC communication
  udp.begin(8001);

  // set i2c and init MPU-9250
  Wire.setSpeed(CLOCK_SPEED_400KHZ);
  Wire.begin();
  Sensors::initialize();

  strip.setBrightness(100);

  mainPlayerFWDVal = 1.0;

  // setup and start the timer using TIMER6
  // to avoid timer confliction
  // run update() every 100ms
  Timer.begin(update, updatePeriod*2, hmSec, TIMER6);

  syncTimer.begin(checkSyncTimeout, 700*2, hmSec, TIMER7);
  syncTimer.end();

}

void update()
{
  //Serial.println("update");

  // send OSCMsg every 100 ms
  sendOSC_OK = true;

  if(ledOnCount > 0)
  {
    ledOnCount--;
  }
  else
  {
    ledOnCount = 0;
  }


}

void checkSyncTimeout()
{
  syncTimeout = true;
}

void setMode(LOVEDUCK_MODES mode)
{
  duck_mode = mode;
  tone(buzzerPin, 4000,100);
  pixelAllOn(255, 255, 255,500);
}

void sendOSCMsg()
{

  switch (duck_mode) {
    case MODE_STANDBY:

      break;

    case MODE_TRUST:
      // for sy(seyoon only!)
      if(digitalRead(touch_A_Pin) == HIGH)
      {
        // check whether accelerometer is available!!
        // or badvalue will go out!!!!
        if(gofwd_OK == true)
        {
          pixelAllOn(redValue,greenValue,blueValue,100);
          String addr = "/gofwd/" + lover_name;
          OSCMessage outMessage(addr);
          outMessage.addFloat(tilt_angle);
          outMessage.send(udp,outIP,outPort);
          delay(1);

          gofwd_OK = false;
        }

      }

      if(goback_OK == true)
      {
        OSCMessage outMessage("/goback");
        outMessage.addFloat(1.0);
        outMessage.send(udp,outIP,outPort);
        delay(1);

        goback_OK = false;
      }

      break;

    case MODE_LOVELY:
      if(gofwd_OK == true)
      {
        pixelAllOn(redValue,greenValue,blueValue,100);
        String addr = "/gofwd/" + lover_name;
        OSCMessage outMessage(addr);
        outMessage.addFloat(mainPlayerFWDVal);
        outMessage.send(udp,outIP,outPort);
        delay(1);
        gofwd_OK = false;
      }

      if(goback_OK == true)
      {
        OSCMessage outMessage("/goback");
        outMessage.addFloat(1.0);
        outMessage.send(udp,outIP,outPort);
        delay(1);

        goback_OK = false;
      }

      break;

    case MODE_CRUSH:

      if(gofwd_OK == true)
      {
        pixelAllOn(redValue,greenValue,blueValue,100);
        String addr = "/gofwd/" + lover_name;
        OSCMessage outMessage(addr);
        outMessage.addFloat(mainPlayerFWDVal);
        outMessage.send(udp,outIP,outPort);
        delay(1);
        gofwd_OK = false;
      }

      if(goback_OK == true)
      {
        OSCMessage outMessage("/goback");
        outMessage.addFloat(1.0);
        outMessage.send(udp,outIP,outPort);
        delay(1);

        goback_OK = false;
      }

      break;

    case MODE_SYNC:
      if (gofwd_OK == true)
      {
        pixelAllOn(redValue,greenValue,blueValue,100);
        String addr = "/gofwd/" + lover_name;
        OSCMessage outMessage(addr);
        outMessage.addFloat(mainPlayerFWDVal);
        outMessage.send(udp,outIP,outPort);
        delay(1);
        gofwd_OK = false;
      }


      if(goback_OK == true)
      {

        OSCMessage outMessage("/goback");
        outMessage.addFloat(1.0);
        outMessage.send(udp,outIP,outPort);
        delay(1);

        goback_OK = false;
      }
      break;

    default:
      break;
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
          inMessage.route("/trust", setModeToTrust);
          inMessage.route("/cute", setModeToLovely);
          inMessage.route("/crush", setModeToCrush);
          inMessage.route("/sync", setModeToSync);
          inMessage.route("/syncleft", checkSyncLeft);
          inMessage.route("/syncright", checkSyncRight);
          inMessage.route("/syncdown", checkSyncDown);
          inMessage.route("/syncjump", checkSyncJump);

          inMessage.route(mainPlayerAddr, setMainPlayer);
          inMessage.route(rawDataLoverAddr, setRawDataDump);

      }
  }
}

void checkSyncLeft(OSCMessage &inMessage)
{
  if(inMessage.getFloat(0) > 0.5)
  {
    sync_submode = SUBMODE_SYNCLEFT;
    syncTimerStart = true;
  }
}

void checkSyncRight(OSCMessage &inMessage)
{
  if(inMessage.getFloat(0) > 0.5)
  {
    sync_submode = SUBMODE_SYNCRIGHT;
    syncTimerStart = true;
  }
}

void checkSyncDown(OSCMessage &inMessage)
{
  if(inMessage.getFloat(0) > 0.5)
  {
    sync_submode = SUBMODE_SYNCDOWN;
    syncTimerStart = true;
  }
}

void checkSyncJump(OSCMessage &inMessage)
{
  if(inMessage.getFloat(0) > 0.5)
  {
    sync_submode = SUBMODE_SYNCJUMP;
    syncTimerStart = true;
  }
}

void checkSensors()
{
  switch (duck_mode) {
    case MODE_STANDBY:
      // readAccel();
      // readGyro();
      // checkFSR();
      // readWhisper();
      // checkPiezo();
      //Serial.println("STANDBY");

      // TODO re-init variables!!!!!!!!!!!!!!!!!!!!!!!!!
      break;

    case MODE_TRUST:
      //Serial.println("PATIENCE");
      if (digitalRead(touch_A_Pin) == HIGH) // for sy(seyoon only!)
      {
        readAccel();
        if (accel_OK == true)
        {
          if (az > 0.0)
          {
            if (ax > 10.0) ax = 10.0;
            if (ax < 0.0) ax = 0.0;
            // tilt_angle = map(ax, 0.0, 10.0, 90.0, 0.0); // map only works on integer
            tilt_angle = ax * (-9.0) + 90.0;
          }
          else
          {
            tilt_angle = 0.0;
          }
        }
        gofwd_OK = true;
        accel_OK = false;
      }
      readWhisper();
      if(whisperValue > 1000)
      {
        gofwd_OK = false;
        goback_OK = true;
      }

      break;

    case MODE_LOVELY:
      //Serial.println("LOVELY");
      //readAccel();
      readGyro();
      if (gyro_OK == true)
      {
        if (gx > 3.0 || gy > 3.0 || gz > 3.0) // 4.3 is almost maximum
        {
          gofwd_OK = true;
        }

        if (gx < -3.0 || gy < -3.0 || gz < -3.0) // 4.3 is almost maximum
        {
          gofwd_OK = true;
        }
        gyro_OK = false;
      }

      readWhisper();
      if(whisperValue > 1000)
      {
        gofwd_OK = false;
        goback_OK = true;
      }

      break;

    case MODE_CRUSH:
      //Serial.println("CRUSH");
      // detect jump using az(head) or ax(back, shoulder)
      // 9.8 -> 0 -> 30
      readAccel();
      if(accel_OK == true)
      {

        // DETECT ONE JUMP
        if (abs(az) < 1.0  || abs(ax) < 1.0)
        {
          jump_mode = JUMP_FREEFALL;
        }

        if (jump_mode == JUMP_FREEFALL)
        {
          if (abs(az) > 25.0  || abs(ax) > 25.0)
          {
            jump_mode = JUMP_STANDBY;
            gofwd_OK = true;
          }
        }

        // detect AIR TIME -> NOT WORKING yet
        // if (abs(az) < 1.0  || abs(ax) < 1.0)
        // {
        //   gofwd_OK = true;
        // }

        accel_OK = false;
      }

      readWhisper();
      if(whisperValue > 1000)
      {
        gofwd_OK = false;
        goback_OK = true;
      }

      break;

    case MODE_SYNC:
      //Serial.println("SYNC");
      readAccel();
      readGyro();

      switch(sync_submode)
      {
        case SUBMODE_STANDBY:
          syncTimer.end();
        break;

        case SUBMODE_SYNCLEFT:
          // if timeout is false and checkLeft is true
          // gofwd_OK = true;
          if(syncTimeout == false)
          {
            if (gyro_OK == true)
            {
              if( gz > 5 || gx > 5)
              {
                gofwd_OK = true;
                syncTimeout = true;
              }
              gyro_OK = false;
            }
          }
        break;

        case SUBMODE_SYNCRIGHT:
          if(syncTimeout == false)
          {
            if (gyro_OK == true)
            {
              if( gz < -5 || gx < -5)
              {
                gofwd_OK = true;
                syncTimeout = true;
              }
              gyro_OK = false;
            }
          }
        break;

        case SUBMODE_SYNCDOWN:
          if(syncTimeout == false)
          {
            if(accel_OK == true)
            {

              if ((abs(az) < 11.0 && abs(az) > 9.0)  || (abs(ax) < 11.0 && abs(ax) > 9.0))
              {
                down_mode = DOWN_STILL;
              }

              if (down_mode == DOWN_STILL)
              {
                // head
                // abs(ax) ~= 2
                // az: 9.8 -> 0


                // shoulder & back
                // ax: -9.8 -> -20
                // abs(az) ~= 2

                // head
                if (abs(ax) < 2 && abs(az)< 5.0)
                {
                  down_mode = DOWN_STANDBY;
                  gofwd_OK = true;
                  syncTimeout = true;
                }
                else
                // shoulder & back
                if(ax < -20 && abs(az) < 2)
                {
                  down_mode = DOWN_STANDBY;
                  gofwd_OK = true;
                  syncTimeout = true;
                }

                // if (abs(az) < 5.0  || abs(ax) < 5.0)
                // {
                //   down_mode = DOWN_STANDBY;
                //   gofwd_OK = true;
                //   syncTimeout = true;
                // }
              }
              accel_OK = false;
            }
          }
        break;

        case SUBMODE_SYNCJUMP:
          if(syncTimeout == false)
          {
            if(accel_OK == true)
            {

              if (abs(az) < 1.0  || abs(ax) < 1.0)
              {
                jump_mode = JUMP_FREEFALL;
              }

              if (jump_mode == JUMP_FREEFALL)
              {
                if (abs(az) > 20.0  || abs(ax) > 20.0)
                {
                  jump_mode = JUMP_STANDBY;
                  gofwd_OK = true;
                  syncTimeout = true;
                }
              }
              accel_OK = false;
            }
          }
        break;

        default:
        break;

      }


      readWhisper();
      if(whisperValue > 1000)
      {
        gofwd_OK = false;
        goback_OK = true;
      }

      break;

    default:
      break;
  }

}

void readAccel()
{
  Accelerometer *accelerometer = Sensors::getAccelerometer();
  if(accelerometer) {
      Vector3 a = accelerometer->getAcceleration();
      ax = a.x;
      ay = a.y;
      az = a.z;
      //Serial.printlnf("Acceleration (m/s^2)  %+7.3f, %+7.3f, %+7.3f", a.x, a.y, a.z);
      accel_OK = true;
  }
}

void readGyro()
{
  Gyroscope *gyroscope = Sensors::getGyroscope();
  if(gyroscope) {
      Vector3 g = gyroscope->getRotation();
      gx = g.x;
      gy = g.y;
      gz = g.z;
      //Serial.printlnf("Rotation (rad/s)      %+7.3f, %+7.3f, %+7.3f", g.x, g.y, g.z);
      gyro_OK = true;
  }
}

void readMagneto()
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

void readWhisper()
{
  whisperValue = analogRead(whisperAnalogPin);
  // //Serial.println(whisperValue);
  // if(whisperValue > 1000)
  // {
  //   whisper_OK = true;
  // }
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
    if(inMessage.getFloat(0)> 0.5)
    {
      gofwd_OK = false;
      goback_OK = false;
      setMode(MODE_STANDBY);
    }
//Do something
}

void setModeToTrust(OSCMessage &inMessage)
{
    //Serial.println("/trust");
    if(inMessage.getFloat(0)> 0.5)
    {
      gofwd_OK = false;
      goback_OK = false;
      setMode(MODE_TRUST);
    }

//Do something
}

void setModeToLovely(OSCMessage &inMessage)
{
    //Serial.println("/lovely");
    if(inMessage.getFloat(0)> 0.5)
    {
      gofwd_OK = false;
      goback_OK = false;
      setMode(MODE_LOVELY);
    }
//Do something
}

void setModeToCrush(OSCMessage &inMessage)
{
    //Serial.println("/crush");
    if(inMessage.getFloat(0)> 0.5)
    {
      gofwd_OK = false;
      goback_OK = false;
      setMode(MODE_CRUSH);
    }
//Do something
}

void setModeToSync(OSCMessage &inMessage)
{
    //Serial.println("/sync");
    if(inMessage.getFloat(0)> 0.5)
    {
      gofwd_OK = false;
      goback_OK = false;
      setMode(MODE_SYNC);
    }
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

void setMainPlayer(OSCMessage &inMessage)
{
  float value = inMessage.getFloat(0);
  //Serial.println(value);
  if (value > 0.5)
  {
    mainPlayerFWDVal = 2.0;
  }
  else
  {
    mainPlayerFWDVal = 1.0;
  }

  tone(buzzerPin, 4000,100);
}

void disableMainPlayer()
{
  mainPlayerFWDVal = 1.0;
  //TODO led off!

}

void setRawDataDump(OSCMessage &inMessage)
{
  float value = inMessage.getFloat(0);
  if (value > 0.5)
  {
    rawdata_OK = true;
  }
  else
  {
    rawdata_OK = false;
  }
}

void dumpRawData()
{
  // String addr = rawDataLoverAddr;
  String addr = "/rawdata/" + lover_name;
  readAccel();
  readGyro();
  readMagneto();
  delay(1);
  readWhisper();

  //send OSC Msg only when the above read values are available
  OSCMessage outMessageA(addr);
  outMessageA.addString("ax");
  outMessageA.addFloat(ax);
  outMessageA.addString("ay");
  outMessageA.addFloat(ay);
  outMessageA.addString("az");
  outMessageA.addFloat(az);
  outMessageA.send(udp,outIP,outPort);
  delay(1);

  OSCMessage outMessageB(addr);
  outMessageB.addString("gx");
  outMessageB.addFloat(gx);
  outMessageB.addString("gy");
  outMessageB.addFloat(gy);
  outMessageB.addString("gz");
  outMessageB.addFloat(gz);
  outMessageB.send(udp,outIP,outPort);
  delay(1);

  OSCMessage outMessageC(addr);
  outMessageC.addString("d");
  outMessageC.addFloat(azimuth);
  outMessageC.addString("w");
  outMessageC.addInt(whisperValue);
  outMessageC.send(udp,outIP,outPort);
  delay(1);
}


void pixelAllOn(int r, int g, int b, int ms)
{
  redValue = r;
  greenValue = g;
  blueValue = b;
  ledOnCount = (int)(ms / updatePeriod);
  if (ledOnCount < 0) ledOnCount = 0;
  ledOn_Start = true;
}

void pixelAllOff()
{
  int i;
  for (i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, strip.Color(0,0,0));
  }
  strip.show();

}
