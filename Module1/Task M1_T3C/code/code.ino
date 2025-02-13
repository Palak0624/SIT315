#define PIR_PIN 2      // PIR motion sensor
#define LED_PIN 7      // LED
#define TMP36_PIN A0   // TMP36 temperature sensor (analog)

// Interrupt flag
volatile bool motionDetected = false;
unsigned long ledTurnOnTime = 0;

float temperatureThreshold = 30.0;  // Temperature threshold in Celsius

// Interrupt Service Routine for PIR sensor
void detectMotion() {
  motionDetected = true;
}

void setup() {
  Serial.begin(9600);
  Serial.println("System Initializing...");

  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Test LED
  Serial.println("Testing LED...");
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  Serial.println("LED test complete.");

  // Attach interrupt for PIR sensor
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), detectMotion, RISING);

  Serial.println("System Ready. Waiting for motion or high temperature...");
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
    motionDetected = true;  // Treat high temperature as a trigger
  }

  // If motion or high temperature is detected, turn on LED
  if (motionDetected) {
    Serial.println("Alert: Motion or High Temperature Detected!");
    digitalWrite(LED_PIN, HIGH);
    Serial.println("LED Turned ON");

    ledTurnOnTime = millis();  // Record LED on time

    // Reset flag
    motionDetected = false;
  }

  // Turn off LED after 5 seconds
  if (ledTurnOnTime != 0 && millis() - ledTurnOnTime >= 5000) {
    digitalWrite(LED_PIN, LOW);
    Serial.println("LED Turned OFF");
    ledTurnOnTime = 0;
  }

  delay(5000);  // Small delay to prevent serial spam
}
