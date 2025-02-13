#define PIR_PIN 2      // PIR motion sensor output pin
#define LED_PIN 7      // LED pin

volatile bool motionDetected = false;  // Flag for motion detection
volatile unsigned long ledTurnOnTime = 0; // Time when LED was turned on

// Interrupt Service Routine for motion detection
void detectMotion() {
  motionDetected = true;
}

void setup() {
  Serial.begin(9600);
  Serial.println("System Initializing...");

  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Test LED at startup
  Serial.println("Testing LED...");
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  Serial.println("LED test complete.");

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), detectMotion, RISING);

  Serial.println("Waiting for motion or sound...");
}

void loop() {
  if (motionDetected) {
    Serial.println("Alert: Motion Detected!");
    digitalWrite(LED_PIN, HIGH);
    Serial.println("LED Turned ON");

    ledTurnOnTime = millis(); // Record the LED on time

    // Reset flags
    motionDetected = false;
  }

  // Turn off LED after 5 seconds
  if (ledTurnOnTime != 0 && millis() - ledTurnOnTime >= 5000) {
    digitalWrite(LED_PIN, LOW);
    Serial.println("LED Turned OFF");
    ledTurnOnTime = 0; // Reset timer
  }
}
