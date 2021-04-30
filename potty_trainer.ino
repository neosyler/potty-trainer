#include <ArduinoHttpClient.h>
#include <WiFiNINA.h>
#include <CapacitiveSensor.h>
#include <Servo.h>
#include <FastLED.h>
#include "arduino_secrets.h"

// WiFi variables
char wifiSsid[] = SECRET_SSID;
char wifiPass[] = SECRET_PASS;


int wifiPort = 80;
char wifiServerAddress[] = "173.231.220.42";

WiFiClient wifi;
HttpClient client = HttpClient(wifi, wifiServerAddress, wifiPort);

// Named constants for LED light strip pins
#define REDPIN   8
#define GREENPIN 6
#define BLUEPIN  9

// Servo
Servo myservo;
const int servoPin = 7;
const int servoLidOpen = 130;
const int servoLidClose = 70;

// Ultrasonic (uss) sensor pins
const int ussPingPin = 2;
const int ussEchoPin = 1;
long ussDuration, ussInches, ussCm;
int personDetectedThreshold = 36; // 3 feet away
int personDetected = 0;

// Water level (wll) sensor
const int wllPin = A1;
const int wllPower = 3;
const int wllWaterLevelHighMin = 80;
int wllValue = 0;
int toiletFlushed = 0;

// Capacitive Sensor
CapacitiveSensor capSensor = CapacitiveSensor(4, 5);
const int capLedPin = 0;
int capThreshold = 11000;
int faucetOn = 0;

// Conditions
int personEnteredBathroom = 0;
int personUsedToilet = 0;
int personFlushedToilet = 0;
int personUsedFaucet = 0;
int personLeftBathroom = 0;

const int waitPeriodMins = 5;

int msgConnectionSent = 0;

int toiletFlushCounter = 0;

void setup() {
  // initialize serial and wait for port to open
  Serial.begin(9600);
//  while (!Serial);

  // set up ultrasonic sensor
  pinMode(ussPingPin, OUTPUT);
  pinMode(ussEchoPin, INPUT);

  // set up water level sensor
  pinMode(wllPower, OUTPUT);
  digitalWrite(wllPower, LOW);

  // set up capacitive sensor
  pinMode(capLedPin, OUTPUT);

  // set up servo
  myservo.attach(servoPin);
  myservo.write(servoLidOpen);

  // LED light strip configuration
  pinMode(REDPIN,   OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN,  OUTPUT);

  // Flash the "hello" color sequence: R, G, B, black.
  showColorBars();  

  // attempt to connect to Wifi network:
  initiateWifiConnection();
}

void initiateWifiConnection() {
  Serial.println("connecting...");
  while (WiFi.begin(wifiSsid, wifiPass) != WL_CONNECTED) {
    // unsuccessful, retry in 4 seconds
    Serial.print("failed ... ");
    delay(4000);
    Serial.print("retrying ... ");
  }

  // were connected, so print out the data:
  Serial.println("You're connected to the network");
  Serial.println("----------------------------------------");
  printData();
  Serial.println("----------------------------------------");
}

void loop() {
  if (msgConnectionSent == 0) {
    sendText("Potty Trainer initiated.", msgConnectionSent);
  }

  //    readCapSensor();
  //    readWaterLevel();
  //    readUltrasonicSensor();
  runLogic();
}

void runLogic() {
  if (personEnteredBathroom == 1) {
    if (personDetected == 1) {
      personLeftBathroom = 1;
      myservo.write(servoLidOpen);
    }

    if (personLeftBathroom == 1 && toiletFlushed == 1 && faucetOn == 1) {
      // person flushed and washed their hands, all conditions met = SUCCESS
      int flag = 0;
      sendText("[1] Person used bathroom, flushed and washed their hands. Success!", flag);
      reset();
    } else if (personLeftBathroom == 1 && toiletFlushed == 1 && faucetOn == 0) {
      // person flushed but forgot to wash hands
      int flag = 0;
      sendText("[2] Person used bathroom and flushed the toilet, but may have forgotten to wash their hands.", flag);
      reset();
    } else if (personLeftBathroom == 1 && toiletFlushed == 0 && faucetOn == 1) {
      // person washed hands, but may have forgotten to flush
      int flag = 0;
      sendText("[3] Person used bathroom, but may have forgotten to flush.", flag);
      reset();
    } else if (toiletFlushed == 1 && faucetOn == 1) {
      // they flushed and washed
      showAnalogRGB( CRGB::Green );
      readUltrasonicSensor();
      myservo.write(servoLidOpen);
    } else if (toiletFlushed == 1) {
      // they flushed, but haven't washed
      showAnalogRGB( CRGB::Red );
      readCapSensor();
      readUltrasonicSensor();
      myservo.write(servoLidClose);
    } else if (faucetOn == 1) {
      // they washed, but haven't flushed
      showAnalogRGB( CRGB::Blue );
      readUltrasonicSensor();
      myservo.write(servoLidOpen);
    } else {
      readWaterLevel();
      readCapSensor();
    }
  } else if (personDetected == 1) {
    showAnalogRGB( CRGB::White );
    personEnteredBathroom = 1;
    personDetected = 0;
  } else {
    // person not detected, read distance sensor
    showAnalogRGB( CRGB::Black );
    readUltrasonicSensor();
  }
}

long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void printData() {
  Serial.println("Board Information:");

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println();
  Serial.println("Network Information:");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

}

void readCapSensor() {
  // store the value reported by the sensor in a variable
  long sensorValue = capSensor.capacitiveSensor(30);

  // if the value is greater than the threshold
  if (sensorValue > capThreshold) {
    // turn the LED on
    digitalWrite(capLedPin, HIGH);
    faucetOn = 1;
    Serial.print("Faucet is on: ");
    Serial.println(sensorValue);
  }
  // if it's lower than the threshold
  else {
    // turn the LED off
    digitalWrite(capLedPin, LOW);
  }
}

int readWaterLevel() {
  digitalWrite(wllPower, HIGH);   // Turn the sensor ON
  delay(10);                      // wait 10 milliseconds
  wllValue = analogRead(wllPin);  // Read the analog value form sensor
  digitalWrite(wllPower, LOW);    // Turn the sensor OFF

  //    Serial.print("Water level: ");
  //    Serial.println(wllValue);

  if (wllValue < wllWaterLevelHighMin) {
    // toilet was flushed
    toiletFlushCounter += 1;

    if (toiletFlushCounter > 5) {
      toiletFlushed = 1;
      toiletFlushCounter = 0;
      
      Serial.print("Toilet was flushed: ");
      Serial.println(wllValue);
    }
  }

  return wllValue;                // send current reading
}

void readUltrasonicSensor() {
  digitalWrite(ussPingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ussPingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ussPingPin, LOW);
  ussDuration = pulseIn(ussEchoPin, HIGH);
  ussInches = microsecondsToInches(ussDuration);
  ussCm = microsecondsToCentimeters(ussDuration);
  Serial.print("Distance: ");
  Serial.println(ussInches);

  if (ussInches < personDetectedThreshold) {
    personDetected = 1;
    Serial.print("Person detected: ");
    Serial.println(ussInches);
  }

  delay(100);
}

void reset() {
  personDetected = 0;
  toiletFlushed = 0;
  faucetOn = 0;
  personEnteredBathroom = 0;
  personFlushedToilet = 0;
  personUsedFaucet = 0;
  personLeftBathroom = 0;

  // turn the LED off
  digitalWrite(capLedPin, LOW);

  // reset servo
  myservo.write(servoLidOpen);

  // reset lights
  showAnalogRGB( CRGB::Black );

  delay(1000 * waitPeriodMins);
}

void sendText(String body, int &flag) {
  String contentType = "application/x-www-form-urlencoded";
  String postData = "To=2699863561&From=+12063171908&Body=" + body;

  Serial.println(body);

  Serial.println("Sending post request");
  client.post("/api/potty-trainer", contentType, postData);
  Serial.println("Finished request.");

  // read the status code and body of the response
  int statusCode = client.responseStatusCode();
  String response = client.responseBody();

  if (statusCode < 200) {
    flag = 0;
  } else {
    flag = 1;
  }
}


// showAnalogRGB: this is like FastLED.show(), but outputs on
// analog PWM output pins instead of sending data to an intelligent,
// pixel-addressable LED strip.
//
// This function takes the incoming RGB values and outputs the values
// on three analog PWM output pins to the r, g, and b values respectively.
void showAnalogRGB( const CRGB& rgb)
{
  analogWrite(REDPIN,   rgb.r );
  analogWrite(GREENPIN, rgb.g );
  analogWrite(BLUEPIN,  rgb.b );
}

// showColorBars: flashes Red, then Green, then Blue, then Black.
// Helpful for diagnosing if you've mis-wired which is which.
void showColorBars()
{
  showAnalogRGB( CRGB::Red );   delay(500);
  showAnalogRGB( CRGB::Green ); delay(500);
  showAnalogRGB( CRGB::Blue );  delay(500);
  showAnalogRGB( CRGB::Black ); delay(500);
}

void turnOffLEDs() {
  showAnalogRGB( CRGB::Black );
}
