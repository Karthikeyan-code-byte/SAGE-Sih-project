#include "model_data.h"

// Sample inputs order: [ HR, SpO2, Temp, Gas_PPM, Humidity ]
// Adjust these variables to read from your real sensors (e.g., MAX30102, MQ-3, DHT22)
float raw_sensor_inputs[5] = {75.0f, 98.2f, 36.6f, 65.0f, 45.0f};

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for Serial Monitor

  Serial.println("\n=== Vicharak ShrikeFi ESP32-S3 ML Inference Initialized ===");
}

void loop() {
  // Execute real-time classification
  int regime = predict_sentinel_regime(raw_sensor_inputs);

  Serial.print("Current Status: ");
  if (regime == 0) {
    Serial.println("Regime 0 [STABLE / NORMAL]");
  } else if (regime == 1) {
    Serial.println("Regime 1 [RISK / ELEVATED]");
  } else if (regime == 2) {
    Serial.println("Regime 2 [CRITICAL / HAZARD]");
  }

  delay(2000); // Sample every 2 seconds
}