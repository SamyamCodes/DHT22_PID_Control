#include <DHT22.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

#define DHTPIN 2          // Pin connected to the DHT22 sensor
#define HEATER_PIN 3      // Pin for controlling the relay or LED (PWM)
#define POTPIN A0         // Pin connected to the potentiometer

// Initialize the DHT22 sensor
DHT22 dht22(DHTPIN); 
LiquidCrystal_I2C lcd(0x27, 16, 2);

// PID Control Variables
double Setpoint = 22.0;  // Initial setpoint temperature in Celsius
double Input, Output;
double Kp = 2, Ki = 5, Kd = 1;  // PID tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  Serial.println("\nTest sensor DHT22");

  lcd.init();
  lcd.backlight();
  
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(POTPIN, INPUT);
  
  // Initialize PID
  myPID.SetMode(AUTOMATIC);  // Set PID mode to automatic
  myPID.SetOutputLimits(0, 255);  // Output range for PWM
}

void loop() {
  Serial.println("=================================");
  Serial.println("Sample DHT22...");
  
  // Read temperature and humidity from DHT22
  float temp = dht22.getTemperature();
  float hum = dht22.getHumidity();
  
  if (dht22.getLastError() != dht22.OK) {
    Serial.print("Last error: ");
    Serial.println(dht22.getLastError());
    return;  // Skip the rest of the loop if there's an error
  }

  // Read potentiometer value and map it to the temperature range
  int potValue = analogRead(POTPIN);
  Setpoint = map(potValue, 0, 1023, 20, 40);  // Adjust the temperature range as needed
  
  // Compute the PID output
  Input = temp;
  myPID.Compute();
  
  // Control the heater (or relay) based on PID output
  if (temp >= Setpoint) {  
    digitalWrite(HEATER_PIN, LOW);  // Turn off the heater (or relay) immediately
  } else {
    analogWrite(HEATER_PIN, Output);  // Control the heater power
  }

  // Display temperature and setpoint on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Setpoint: ");
  lcd.print(Setpoint);
  lcd.print(" ");
  lcd.print((char)223); // Degree symbol
  lcd.print("C");
  
  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(temp, 1);
  lcd.print(" ");
  lcd.print((char)223); // Degree symbol
  lcd.print("C");

  lcd.setCursor(0, 2);
  lcd.print("Output: ");
  lcd.print(Output);

  // Print debug information to the Serial Monitor
  Serial.print("Desired Temperature: ");
  Serial.print(Setpoint);
  Serial.print("C, Actual Temperature: ");
  Serial.print(temp, 1);
  Serial.println("C");
  
  delay(2000); // Delay 2 seconds
}
