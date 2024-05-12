#include <SoftwareSerial.h> // Include the SoftwareSerial library to communicate over Bluetooth
SoftwareSerial BTSerial(10, 11); // RX, TX pins for Bluetooth communication

#define MOTION_SENSOR_PIN  8 // The Arduino Nano pin connected to the OUTPUT pin of motion sensor
int motion_state   = LOW; // current state of pin
int prev_motion_state  = LOW; // previous state of pin

#define led 6
#include "DHT.h"
#define DHTPIN1 2     // what pin we're connected to
#define DHTPIN2 9     // what pin we're connected to
#define DHTTYPE DHT11   // DHT 11
DHT dht1(DHTPIN1, DHTTYPE, 6);
DHT dht2(DHTPIN2, DHTTYPE, 6);

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

#include <Servo.h>
#define SERVO_PIN 3 // Arduino Nano pin D9 connected to the signal pin of servo motor
Servo servo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
#define SENSOR_PIN A0  // Analog pin connected to the MQ5 sensor
#define VOLTAGE_RANGE_MIN 0.1  // Minimum sensor voltage (in volts) corresponding to 200 ppm LPG
#define VOLTAGE_RANGE_MAX 4.0  // Maximum sensor voltage (in volts) corresponding to 10000 ppm LPG

int fan = 11;
float h1, t1, f1 , h2, t2, f2, hi1, hi2, sensorVoltage, lpgConcentration;

// Define Bluetooth command codes
#define GAS_LEAKAGE_COMMAND 'G'
#define GAS_NORMAL_COMMAND 'N'
#define MOTION_DETECTED_COMMAND 'M'

void sendBluetoothNotification(char command) {
  BTSerial.write(command); // Send command over Bluetooth
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);            // Initialize the Serial to communicate with the Serial Monitor.
  BTSerial.begin(9600); // Initialize Bluetooth serial communication
  pinMode(MOTION_SENSOR_PIN, INPUT); // set arduino pin to input mode to read value from OUTPUT pin of sensor
  Serial.println("DHTxx test!");
  dht1.begin();
  dht2.begin();
  lcd.init();                      // initialize the lcd
  lcd.backlight();
  lcd.setCursor(0, 0);
  servo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  pinMode (fan, OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(fan,LOW);
}

void loop() {
  h1 = dht1.readHumidity();
  t1 = dht1.readTemperature();
  f1 = dht1.readTemperature(true);

  h2 = dht2.readHumidity();
  t2 = dht2.readTemperature();
  f2 = dht2.readTemperature(true);
  if (isnan(h1) || isnan(t1) || isnan(f1)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  if (isnan(h2) || isnan(t2) || isnan(f2)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  hi1 = dht1.computeHeatIndex(f1, h1);
  hi2 = dht2.computeHeatIndex(f2, h2);
  lcd.setCursor(0, 0);
  lcd.print("Temp in:        ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(t1);
  lcd.print("c");
  delay(2000);

  lcd.setCursor(0, 0);
  lcd.print("Temp out: ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(t2);
  lcd.print("c");
  delay(2000);

  sensorVoltage;
  lpgConcentration;
  sensorVoltage = analogRead(SENSOR_PIN) * (5.0 / 1023.0);  // Convert analog value to voltage
  lpgConcentration = map(sensorVoltage, VOLTAGE_RANGE_MIN, VOLTAGE_RANGE_MAX, 200, 10000);
  lcd.setCursor(0, 0);
  lcd.print("Gas : ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(lpgConcentration);
  lcd.print("ppm");
  delay(2000);

  if (lpgConcentration > 1000)
  {
    servo.write(0);  digitalWrite(fan, HIGH);
    lcd.setCursor(0, 0);
    lcd.print("Fan on");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    delay(2000);
    sendBluetoothNotification(GAS_LEAKAGE_COMMAND); // Send gas leakage notification
  }
  else
  {
    servo.write(180);  digitalWrite(fan, LOW);
    lcd.setCursor(0, 0);
    lcd.print("Fan off");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    delay(2000);
    sendBluetoothNotification(GAS_NORMAL_COMMAND); // Send gas normal notification
  }

  prev_motion_state = motion_state; // store old state
  motion_state = digitalRead(MOTION_SENSOR_PIN);   // read new state
  if (prev_motion_state == LOW && motion_state == HIGH) {   // pin state change: LOW -> HIGH
    lcd.setCursor(0, 0);
    lcd.print("Motion detected");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    delay(2000);
    digitalWrite(led, HIGH);
    delay(2000);
    sendBluetoothNotification(MOTION_DETECTED_COMMAND); // Send motion detection notification
  }
  else if (prev_motion_state == HIGH && motion_state == LOW) {  // pin state change: HIGH -> LOW
    lcd.setCursor(0, 0);
    lcd.print("Motion stopped");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    delay(2000);
    digitalWrite(led, LOW);
  }

}
