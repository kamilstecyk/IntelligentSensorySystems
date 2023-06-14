#include <Wire.h>
#include "DFRobotFermion.h"

DFRobotFermion fermion;
int heartRate;
RunningStatistics intervalStats;

#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);  // Wait for serial connection

  Wire.begin(SDA_PIN, SCL_PIN);  // Inicjalizacja komunikacji I2C z odpowiednimi pinami SDA i SCL
  fermion.begin();  // Initialize the Fermion sensor

}

void loop() {
  // Read heart rate from the sensor
  heartRate = fermion.getHeartRate();

  // Check if a valid heart rate is obtained
  if (heartRate > 0) {
    Serial.print("Heart Rate: ");
    Serial.println(heartRate);

    // Calculate HRV
    if (intervalStats.count() > 0) {
      unsigned long currentMillis = millis();
      unsigned long interval = currentMillis - previousMillis;
      previousMillis = currentMillis;
      intervalStats.push(interval);

      Serial.print("HRV: ");
      Serial.println(intervalStats.standardDeviation());
    } else {
      previousMillis = millis();
    }
  }

  delay(1000);  // Delay for 1 second before reading again

}
