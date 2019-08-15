#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>

#include <DallasTemperature.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>


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
}

uint8_t i=0;
uint32_t lastMsg;

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis()/1000);

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

// ------------------------------ Non Blocking Sensor Read and LCD Display ------------------------------
  if ((millis() - lastMsg > 5000) || (millis()-lastMsg < 0)) {     // intervall 5 seconds
    lastMsg = millis();

  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  Serial.print("Temperature for the device 1 (index 0) is: ");
  Serial.println(sensors.getTempCByIndex(0));

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(sensors.getTempCByIndex(0));
  lcd.write(0);
  lcd.print("C");
  }

}