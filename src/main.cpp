#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>

#include <DallasTemperature.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

#include <PID_v1.h>


/* --------------- initialize the LCD --------------- */
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
uint8_t degree[8] = {0x7,0x5,0x7,0x0,0x0,0x0,0x0,0x0};


/* --------------- initialize DS18B20 --------------- */
// Data wire is connected to GPIO32
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
double sensVal;


/* ---- initialize PID Controller */
#define RELAY_PIN 3

//Define Variables we'll be connecting to
double Setpoint, Output;

//Specify the links and initial tuning parameters
double Kp=7, Ki=5, Kd=2;
uint16_t sampleTime = 700;
PID myPID(&sensVal, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

uint16_t WindowSize = 5000;
uint16_t timeFactor = 5;
uint16_t upLimit = WindowSize/timeFactor;
uint32_t windowStartTime;





void setup() {
  // Debugging output
  Serial.begin(115200);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.setBacklight(0x7);
  lcd.createChar(0,degree);
  lcd.print("Hello, world!");


  // Start up the DS18B20 library
  sensors.begin();


  // PID
  pinMode(RELAY_PIN,OUTPUT);

  windowStartTime = millis();
  //initialize the variables we're linked to
  Setpoint = 85;
  Output = 1000;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, upLimit);
  myPID.SetSampleTime(sampleTime);                                                

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

}

uint8_t i=0;
uint32_t lastMsg = millis();
uint32_t sensPoll = millis();

void loop() {
  /* ---------- read Temperature sensor ---------- */
   if ((millis() - sensPoll > sampleTime) || (millis()-sensPoll < 0)) {     // intervall 5 seconds
    sensPoll = millis();

    // call sensors.requestTemperatures() to issue a global temperature 
    // request to all devices on the bus
    //Serial.print("Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    sensVal = sensors.getTempCByIndex(0);
    //Serial.println("DONE");
    //Serial.print("Temperature for the device 1 (index 0) is: ");
    //Serial.println(sensVal);
    Serial.println(String(Output) + "," + String(sensVal));
  }



  /* ---------- handle PID ---------- */
  myPID.Compute();
  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  double relayTime = Output*timeFactor;
  if (relayTime < millis() - windowStartTime) digitalWrite(RELAY_PIN, LOW);          // for normaly open relay 
  else digitalWrite(RELAY_PIN, HIGH);



  /* ---------- update Secons counter on LCD ---------- */
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis()/1000);


  /* ---------- hnadle LCD buttons ---------- */
  uint8_t buttons = lcd.readButtons();

  if (buttons) {
    lcd.clear();
    lcd.setCursor(0,0);
    if (buttons & BUTTON_UP) {
      lcd.print("UP ");
    }
    if (buttons & BUTTON_DOWN) {
      lcd.print("DOWN ");
    }
    if (buttons & BUTTON_LEFT) {
      lcd.print("LEFT ");
    }
    if (buttons & BUTTON_RIGHT) {
      lcd.print("RIGHT ");
    }
    if (buttons & BUTTON_SELECT) {
      lcd.print("SELECT ");
    }
  }

/* ------------------------------ Non Blocking LCD Update ------------------------------ */
  if ((millis() - lastMsg > 5000) || (millis()-lastMsg < 0)) {     // intervall 5 seconds
    lastMsg = millis();

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(sensors.getTempCByIndex(0));
    lcd.write(0);
    lcd.print("C");
  }

}