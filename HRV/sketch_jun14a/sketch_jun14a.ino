/*!
  * @file  gainHeartbeatSPO2.ino
  * @n experiment phenomena: get the heart rate and blood oxygenation, during the update the data obtained does not change
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @licence     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V1.0
  * @date        2021-06-21
  * @get         from https://www.dfrobot.com
  * @url         https://github.com/DFRobot/DFRobot_BloodOxygen_S
*/
#include "DFRobot_BloodOxygen_S.h"
#include <vector>
#include <cmath>

#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef  I2C_COMMUNICATION
#define I2C_ADDRESS    0x57
  DFRobot_BloodOxygen_S_I2C MAX30102(&Wire ,I2C_ADDRESS);
#else
/* ---------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |
 * ---------------------------------------------------------------------------------------------------------------*/
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
SoftwareSerial mySerial(4, 5);
DFRobot_BloodOxygen_S_SoftWareUart MAX30102(&mySerial, 9600);
#else
DFRobot_BloodOxygen_S_HardWareUart MAX30102(&Serial1, 9600); 
#endif
#endif

// Constants
const int MAX_HR_BUFFER_SIZE = 10;
const int HR_SAMPLING_INTERVAL = 1000; // Sampling interval in milliseconds

int heartRateBuffer[MAX_HR_BUFFER_SIZE];
int bufferIndex = 0;
unsigned long previousMillis = 0;

// Function to calculate approximation of HRV from heart rate buffer
// Calculating HRV last 10 seconds
float calculateHRV()
{
  // Calculate the standard deviation of heart rate values
  float sum = 0;
  float mean = 0;

  for (int i = 0; i < MAX_HR_BUFFER_SIZE; i++)
  {
    sum += heartRateBuffer[i];
  }
  mean = sum / MAX_HR_BUFFER_SIZE;

  float sumSquaredDiff = 0;
  for (int i = 0; i < MAX_HR_BUFFER_SIZE; i++)
  {
    float diff = heartRateBuffer[i] - mean;
    sumSquaredDiff += diff * diff;
  }
  float variance = sumSquaredDiff / MAX_HR_BUFFER_SIZE;
  float standardDeviation = sqrt(variance);

  return standardDeviation;
}

void setup()
{
  Serial.begin(115200);
  while (false == MAX30102.begin())
  {
    Serial.println("init fail!");
    delay(1000);
  }
  Serial.println("init success!");
  Serial.println("start measuring...");
  MAX30102.sensorStartCollect();

  Serial.print("Delay because of bad values at the beginning.. Prepare your finger and put it on the sensor..");
  delay(3000);
}

// Co 4 sekundy następuje odczyt parametrów pacjenta z czujnika MAX30102
// Dodatkowo obliczamy HRV w następujący sposób:
// Po każdym odczycie, obliczamy interwał RR w milisekundach
// (jako 60 000 ms podzielone przez liczbę uderzeń serca na minutę). 
// Następnie dodajemy ten interwał RR do wektora rrIntervals. 
// Jeśli wektor ten zawiera co najmniej dwa interwały, obliczamy średnią 
//wartość RR oraz odchylenie standardowe (SDNN) tych interwałów. 

void loop()
{
   // Check if it's time to sample heart rate
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= HR_SAMPLING_INTERVAL)
  {
    previousMillis = currentMillis;

    MAX30102.getHeartbeatSPO2();

    Serial.print("SPO2 is : ");
    Serial.print(MAX30102._sHeartbeatSPO2.SPO2);
    Serial.println("%");

    Serial.print("heart rate is : ");
    Serial.print(MAX30102._sHeartbeatSPO2.Heartbeat);
    int heartRate = MAX30102._sHeartbeatSPO2.Heartbeat;
    Serial.println("Times/min");

    Serial.print("Temperature value of the board is : ");
    Serial.print(MAX30102.getTemperature_C());
    Serial.println(" ℃");

    // Store heart rate in the buffer
    heartRateBuffer[bufferIndex] = heartRate;
    bufferIndex++;

    // Check if the buffer is full
    if (bufferIndex >= MAX_HR_BUFFER_SIZE)
    {
      // Calculate HRV
      float hrv = calculateHRV();

      // Print the HRV value
      Serial.print("HRV: ");
      Serial.print(hrv);
      Serial.println("ms");

      // Reset buffer and index
      bufferIndex = 0;
    }

  }

  //The sensor updates the data every 4 seconds
  // delay(100);
  //Serial.println("stop measuring...");
  //MAX30102.sensorEndCollect();

}
