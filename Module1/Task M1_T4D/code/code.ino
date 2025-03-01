#include <avr/interrupt.h>
#include <avr/io.h>

#define PIR_PIN 2        // PIR motion sensor
#define LED_PIN 7        // LED
#define TMP36_PIN A0     // TMP36 temperature sensor (analog)
#define MOISTURE_PIN A1  // Soil moisture sensor (analog)

// Interrupt flags for each sensor
volatile bool motionDetected = false;
volatile bool timerTriggered = false;
volatile bool drySoilDetected = false;
volatile bool highTempDetected = false;

unsigned long ledTurnOnTime = 0;

// Threshold values
float temperatureThreshold = 30.0;  // Temperature threshold in Celsius
int moistureThreshold = 300;        // Dry soil threshold (adjust as needed)

// Timer interrupt fires every X milliseconds
#define TIMER_INTERVAL_MS 5000  // Set timer interval

void setup() {
  Serial.begin(9600);
  Serial.println("System Initializing...");

  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // LED Test
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  Serial.println("LED Test Complete.");

  // Enable Pin Change Interrupts (PCINT) for PIR
  PCICR |= (1 << PCIE2);    // Enable PCINT for PORTD
  PCMSK2 |= (1 << PIR_PIN); // Enable interrupt for PIR sensor

  // Setup Timer Interrupt (Timer1)
  cli();  // Disable global interrupts
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);  // CTC Mode, Prescaler = 1024
  OCR1A = (F_CPU / 1024) * TIMER_INTERVAL_MS / 1000 - 1;  // Set compare match
  TIMSK1 |= (1 << OCIE1A);  // Enable Timer1 Compare Interrupt
  sei();  // Enable global interrupts

  Serial.println("System Ready. Waiting for motion, dry soil, or high temperature...");
}

// Pin Change Interrupt Service Routine (PCINT) for PIR sensor
ISR(PCINT2_vect) {
  if (digitalRead(PIR_PIN) == HIGH) {
    motionDetected = true;
  }
}

// Timer Interrupt Service Routine (fires every X milliseconds)
ISR(TIMER1_COMPA_vect) {
  timerTriggered = true;
}

void loop() {
  // Read TMP36 sensor
  float voltage = analogRead(TMP36_PIN) * (5.0 / 1023.0);
  float temperatureC = (voltage - 0.5) * 100.0;
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  // Check if temperature is above threshold
  if (temperatureC > temperatureThreshold) {
    Serial.println("High Temperature Detected!");
    highTempDetected = true;
  }

  // Read Soil Moisture Sensor
  int moistureLevel = analogRead(MOISTURE_PIN);
  Serial.print("Soil Moisture Level: ");
  Serial.println(moistureLevel);

  // Check if soil is dry
  if (moistureLevel < moistureThreshold) {
    Serial.println("Dry Soil Detected!");
    drySoilDetected = true;
  }

  // Handle separate alerts
  if (motionDetected) {
    Serial.println("ALERT: Motion Detected!");
    digitalWrite(LED_PIN, HIGH);
    ledTurnOnTime = millis();
    motionDetected = false;
  }

  if (highTempDetected) {
    Serial.println("ALERT: High Temperature Detected!");
    digitalWrite(LED_PIN, HIGH);
    ledTurnOnTime = millis();
    highTempDetected = false;
  }

  if (drySoilDetected) {
    Serial.println("ALERT: Dry Soil Detected!");
    digitalWrite(LED_PIN, HIGH);
    ledTurnOnTime = millis();
    drySoilDetected = false;
  }

  if (timerTriggered) {
    Serial.println("ALERT: Timer Triggered!");
    digitalWrite(LED_PIN, HIGH);
    ledTurnOnTime = millis();
    timerTriggered = false;
  }

  // Turn off LED after 5 seconds
  if (ledTurnOnTime != 0 && millis() - ledTurnOnTime >= 5000) {
    digitalWrite(LED_PIN, LOW);
    Serial.println("LED Turned OFF");
    ledTurnOnTime = 0;
  }

  delay(1000);  // Small delay for better readability
}
