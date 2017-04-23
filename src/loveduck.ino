/*
    MPU-9250TestOSC2

    mpu-9250 with particle photon.
    library from https://github.com/loopj/i2c-sensor-hal
*/

#define SENSORS_MPU9250_ATTACHED

#include "simple-OSC.h"
#include "Sensors.h"

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

#define LED             D7      //indicator, Grove - LED is connected with D4 of Arduino
#define intPin 			D4  // These can be changed, 2 and 3 are the Arduinos ext int pins
UDP udp;

IPAddress outIP(192, 168, 100, 255);        // braodcast ip addess
unsigned int outPort = 8000;                // port

bool ledState = LOW;
unsigned long lastLEDStateChangedTimer;
unsigned long last9250DataUpdatedTimer;
unsigned long lastOSCMessageSentTimer;

float ax, ay, az, gx, gy, gz, mx, my, mz, azimuth, temperature;


void setup() {
  Serial.begin(115200);
    // static IP setting
	IPAddress myAddress(192, 168, 100, 101);
	IPAddress netmask(255, 255, 255, 0);
	IPAddress gateway(192, 168, 100, 1);
	IPAddress dns(192, 168, 100, 1);
	WiFi.setStaticIP(myAddress, netmask, gateway, dns);

	WiFi.useStaticIP();
	WiFi.connect();

	while(!WiFi.ready()){
		Particle.process();
		delay(500);
	}
	Particle.process();

	udp.begin(8001);

	pinMode(LED, OUTPUT);

	Serial.println("OSC address patterns... : ");

	Wire.begin();
	Sensors::initialize();

}

void loop() {
	if(millis() - last9250DataUpdatedTimer > 50){
		Accelerometer *accelerometer = Sensors::getAccelerometer();
	    if(accelerometer) {
	        Vector3 a = accelerometer->getAcceleration();
	        ax = a.x;
	        ay = a.y;
	        az = a.z;
	        // Serial.printlnf("Acceleration (m/s^2)  %+7.3f, %+7.3f, %+7.3f", a.x, a.y, a.z);
	    }

	    Gyroscope *gyroscope = Sensors::getGyroscope();
	    if(gyroscope) {
	        Vector3 g = gyroscope->getRotation();
	        gx = g.x;
	        gy = g.y;
	        gz = g.z;
	        // Serial.printlnf("Rotation (rad/s)      %+7.3f, %+7.3f, %+7.3f", g.x, g.y, g.z);
	    }

	    Magnetometer *magnetometer = Sensors::getMagnetometer();
	    if(magnetometer) {
	        Vector3 m = magnetometer->getMagneticField();

	        mx = m.x;
	        my = m.y;
	        mz = m.z;
	        // Serial.printlnf("Magnetic Field (uT)   %+7.3f, %+7.3f, %+7.3f", m.x, m.y, m.z);

	        azimuth = magnetometer->getAzimuth();
	        // Serial.printlnf("Azimuth (deg)         %+7.3f", azimuth);
	    }

	    Thermometer *thermometer = Sensors::getThermometer();
	    if(thermometer) {
	        float t = thermometer->getTemperature();
	        temperature = t;
	        // Serial.printlnf("Temperature (C)       %+7.3f", t);
	    }
	    last9250DataUpdatedTimer = millis();
	}

	if(millis() - lastOSCMessageSentTimer > 10){
		sendAccOSCMessage("/accel/demo");
		// count++;
		sendOSCMessage("/demo", 1);

		lastOSCMessageSentTimer = millis();
	}

	Particle.process();

	// LED status
	if(WiFi.ready())	blinkingLED(50);
	else 				blinkingLED(200);
}

void blinkingLED(int interval){
	if(millis() - lastLEDStateChangedTimer > interval){
		ledState = !ledState;
		lastLEDStateChangedTimer = millis();
	}
	digitalWrite(LED, ledState);
}

void sendAccOSCMessage(String addrPattern){
	OSCMessage message(addrPattern);
	message.addString(float2String(ax));
	message.addString(float2String(ay));
	message.addString(float2String(az));

	message.addString(float2String(gx));
	message.addString(float2String(gy));
	message.addString(float2String(gz));

	message.addString(float2String(mx));
	message.addString(float2String(my));
	message.addString(float2String(mz));
	message.addString(float2String(azimuth));

	message.addString(float2String(temperature));

	message.send(udp, outIP, outPort);
	// delay(10);
}

void sendOSCMessage(String addrPattern, int value){
	OSCMessage message(addrPattern);
	message.addString(int2String(value));
	message.send(udp, outIP, outPort);
	// delay(10);
}

String int2String(int num){
	char message[64] = "";
	sprintf(message, "%d", num);
	return message;
}

String double2String(unsigned long num){
	char message[64] = "";
	sprintf(message, "%lu", num);
	return message;
}

String float2String(float num){
	char message[64] = "";
	sprintf(message, "%f", num);
	return message;
}
