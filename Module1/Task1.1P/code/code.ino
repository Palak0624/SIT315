// Define pins
const int pirSensorPin = 2;  // PIR Sensor digital output connected to pin 2
const int ledPin = 8;        // External LED connected to pin 8

void setup() {
    pinMode(pirSensorPin, INPUT);  // Set PIR sensor pin as input
    pinMode(ledPin, OUTPUT);       // Set LED pin as output
    Serial.begin(9600);            // Start Serial communication
}

void loop() {
    int motionDetected = digitalRead(pirSensorPin); // Read PIR sensor

    if (motionDetected == HIGH) { 
        digitalWrite(ledPin, HIGH);  // Turn External LED ON
        Serial.println("Motion detected! LED ON");
    } else {
        digitalWrite(ledPin, LOW);   // Turn External LED OFF
        Serial.println("No motion detected. LED OFF");
    }

    delay(5000); // Small delay for stability
}
