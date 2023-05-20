 #include <AltSoftSerial.h>
#include <TinyGPS++.h>
#include<Wire.h>
#include <math.h>
#include <SoftwareSerial.h>



#define Relay 13

#define buzzer A4

const String EMERGENCY_PHONE = "+919360321408";
#define rxPin 4
#define txPin 5

SoftwareSerial sim800(rxPin, txPin);
AltSoftSerial neogps;
TinyGPSPlus gps;
//gps(9,8)

String sms_status, sender_number, received_date, msg;
String latitude, longitude;
#define BUZZER 12
#define BUTTON 11
#define xPin A1
#define yPin A2
#define zPin A3

byte updateflag;

int eyeblinkPin = 10;
int speakerPin = 9;
int eyeblinkState = 0;
int lastEyeblinkState = 0;
int eyeblinkCount = 0;

static const int sensorPin = 10;                    // sensor input pin 
int SensorStatePrevious = LOW;                      // previousstate of the sensor

unsigned long minSensorDuration = 3000; // Time we wait before  the sensor active as long 
unsigned long minSensorDuration2 = 6000;
unsigned long SensorLongMillis;                // Time in ms when the sensor was active
bool SensorStateLongTime = false;                  // True if it is a long active

const int intervalSensor = 50;                      // Time between two readings sensor state
unsigned long previousSensorMillis;                 // Timestamp of the latest reading

unsigned long SensorOutDuration;                  // Time the sensor is active in ms

//// GENERAL ////

unsigned long currentMillis;  

int xaxis = 0, yaxis = 0, zaxis = 0;
int deltx = 0, delty = 0, deltz = 0;
int vibration = 2;
int devibrate = 75;
int magnitude = 0;
int sensitivity = 20;

double angle;
boolean impact_detected = false;
unsigned long time1;
unsigned long impact_time;
unsigned long alert_delay = 30000;

        // Variabele to store the number of milleseconds since the Arduino has started


void setup()
{

  pinMode(eyeblinkPin, INPUT);
  pinMode(speakerPin, OUTPUT);
  sim800.begin(9600);

   Serial.begin(9600);                 // Initialise the serial monitor

  pinMode(sensorPin, INPUT);          // set sensorPin as input
  Serial.println("Press button");
  pinMode(Relay,OUTPUT);
  pinMode(buzzer,OUTPUT);

  Serial.begin(9600);
  sim800.begin(9600);
  neogps.begin(9600);
  pinMode(BUZZER, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  sms_status = "";
  sender_number = "";
  received_date = "";
  msg = "";
  sim800.println("AT");
  delay(1000);
  sim800.println("ATE1");
  delay(1000);
  sim800.println("AT+CPIN?");
  delay(1000);
  sim800.println("AT+CMGF=1");
  delay(1000);
  sim800.println("AT+CNMI=1,1,0,0,0");
  delay(1000);
  time1 = micros();
  xaxis = analogRead(xPin);
  yaxis = analogRead(yPin);
  zaxis = analogRead(zPin);
}



void loop()
{

eyeblinkState = analogRead(eyeblinkPin);

  if (eyeblinkState < 400) {
    eyeblinkCount++;
    delay(1000);
  }

  if (eyeblinkCount >= 2) {
    sim800.println("AT+CMGF=1");
    delay(1000);
    sim800.println("AT+CMGS=\"9360321408\""); //replace with your phone number
    delay(1000);
    sim800.println("Alert: sleep detected while driving!"); //message to send
    sim800.write(0x1A);
    delay(5000);
    tone(speakerPin, 1000, 2000);
    sim800.println("ATD9360321408;"); //replace with your phone number
    delay(5000);
    eyeblinkCount = 0;
  }
   currentMillis = millis();    // store the current time
  readSensorState();           // read the sensor state

  if (micros() - time1 > 1999) Impact();
  if (updateflag > 0)
  {
    updateflag = 0;
    Serial.println("Impact detected!!");
    Serial.print("Magnitude:");
    Serial.println(magnitude);

    getGps();
    digitalWrite(BUZZER, HIGH);
    impact_detected = true;
    impact_time = millis();


  }
  if (impact_detected == true)
  {
    if (millis() - impact_time >= alert_delay) {
      digitalWrite(BUZZER, LOW);
      makeCall();
      delay(1000);
      sendAlert();
      impact_detected = false;
      impact_time = 0;
    }
  }

  if (digitalRead(BUTTON) == LOW) {
    delay(200);
    digitalWrite(BUZZER, LOW);
    impact_detected = false;
    impact_time = 0;
  }
  while (sim800.available()) {
    parseData(sim800.readString());
  }
  while (Serial.available())  {
    sim800.println(Serial.readString());
  }
}

void readSensorState() {

  // If the difference in time between the previous reading is larger than intervalsensor
  if(currentMillis - previousSensorMillis > intervalSensor) {
    
    // Read the digital value of the sensor (LOW/HIGH)
    int SensorState = digitalRead(sensorPin);    

    // If the button has been active AND
    // If the sensor wasn't activated before AND
    // IF there was not already a measurement running to determine how long the sensor has been activated
    if (SensorState == LOW && SensorStatePrevious == HIGH && !SensorStateLongTime) {
     SensorLongMillis = currentMillis;
       SensorStatePrevious = LOW;
      
      Serial.println("Button pressed");
    }

    // Calculate how long the sensor has been activated
   SensorOutDuration = currentMillis - SensorLongMillis;

    // If the button is active AND
    // If there is no measurement running to determine how long the sensor is active AND
    // If the time the sensor has been activated is larger or equal to the time needed for a long active
    if (SensorState == LOW && !SensorStateLongTime && SensorOutDuration >= minSensorDuration) {
      SensorStateLongTime = true;
      digitalWrite(Relay,HIGH);
      Serial.println("Button long pressed");
    }
    if (SensorState == LOW && SensorStateLongTime && SensorOutDuration >= minSensorDuration2) {
     SensorStateLongTime = true;
      digitalWrite(buzzer,HIGH);
      delay(1000);
      Serial.println("Button long pressed");
    }
      
    // If the sensor is released AND
    // If the sensor was activated before
    if (SensorState == HIGH && SensorStatePrevious == LOW) {
      SensorStatePrevious = HIGH;
      SensorStateLongTime = false;
      digitalWrite(Relay,LOW);
      digitalWrite(buzzer,LOW);
      Serial.println("Button released");

  
    }
    
    // store the current timestamp in previousSensorMillis
   previousSensorMillis = currentMillis;
    lastEyeblinkState = eyeblinkState;

  }

}


void Impact()
{
  time1 = micros();
  int oldx = xaxis;
  int oldy = yaxis;
  int oldz = zaxis;

  xaxis = analogRead(xPin);
  yaxis = analogRead(yPin);
  zaxis = analogRead(zPin);


  vibration--;
  Serial.print("Vibration = 50 ");
  Serial.println(vibration);
  if (vibration < 0) vibration = 0;
  if (vibration > 0) return;
  deltx = xaxis - oldx;
  delty = yaxis - oldy;
  deltz = zaxis - oldz;


  magnitude = sqrt(sq(deltx) + sq(delty) + sq(deltz));
  if (magnitude >= sensitivity) //impact detected
  {
    updateflag = 1;
    vibration = devibrate;
  }
  else
  {
    magnitude = 60;
  }
}


void parseData(String buff) {
  Serial.println(buff);

  unsigned int len, index;
  index = buff.indexOf("\r");
  buff.remove(0, index + 2);
  buff.trim();
  if (buff != "OK") {
    index = buff.indexOf(":");
    String cmd = buff.substring(0, index);
    cmd.trim();

    buff.remove(0, index + 2);
    if (cmd == "+CMTI") {
      index = buff.indexOf(",");
      String temp = buff.substring(index + 1, buff.length());
      temp = "AT+CMGR=" + temp + "\r";
      sim800.println(temp);
    }
    else if (cmd == "+CMGR") {
      if (buff.indexOf(EMERGENCY_PHONE) > 1) {
        buff.toLowerCase();
        if (buff.indexOf("get gps") > 1) {
          getGps();
          String sms_data;
          sms_data = "GPS Location Data\r";
          sms_data += "http://maps.google.com/maps?q=loc:";
          sms_data += latitude + "," + longitude;

          sendSms(sms_data);
        }
      }
    }
  }
  else {
  }
}

void getGps()
{
  // Can take up to 60 seconds
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 2000;) {
    while (neogps.available()) {
      if (gps.encode(neogps.read())) {
        newData = true;
        break;
      }
    }
  }

  if (newData)
  {
    latitude = String(gps.location.lat(), 6);
    longitude = String(gps.location.lng(), 6);
    newData = false;
  }
  else {
    Serial.println("No GPS data is available");
    latitude = "";
    longitude = "";
  }

  Serial.print("Latitude= "); Serial.println(latitude);
  Serial.print("Lngitude= "); Serial.println(longitude);
}


void sendAlert()
{
  String sms_data;
  sms_data = "Accident Alert!!\r";
  sms_data += "http://maps.google.com/maps?q=loc:";
  sms_data += latitude + "," + longitude;

  sendSms(sms_data);
}

void makeCall()
{
  Serial.println("calling....");
  sim800.println("ATD" + EMERGENCY_PHONE + ";");
  delay(20000); //20 sec delay
  sim800.println("ATH");
  delay(1000); //1 sec delay
}

void sendSms(String text)
{
  //return;
  sim800.print("AT+CMGF=1\r");
  delay(1000);
  sim800.print("AT+CMGS=\"" + EMERGENCY_PHONE + "\"\r");
  delay(1000);
  sim800.print(text);
  delay(100);
  sim800.write(0x1A);
  delay(1000);
  Serial.println("SMS Sent Successfully.");
}


boolean SendAT(String at_command, String expected_answer, unsigned int timeout) {

  uint8_t x = 0;
  boolean answer = 0;
  String response;
  unsigned long previous;
  while ( sim800.available() > 0) sim800.read();

  sim800.println(at_command);

  x = 0;
  previous = millis();

  do {
    if (sim800.available() != 0) {
      response += sim800.read();
      x++;
      if (response.indexOf(expected_answer) > 0) {
        answer = 1;
        break;
      }
    }
  }
  while ((answer == 0) && ((millis() - previous) < timeout));
  Serial.println(response);
  return answer;
}
