#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;

uint8_t effect = 64;

// Define the pins used for the GPS connection
SoftwareSerial mySerial(5,3); // RX, TX
Adafruit_GPS GPS(&mySerial);

// Set your target location coordinates
float targetLat = 42.29338;
float targetLon = -71.303532;
// float targetLat = 34.0522;  // Example latitude
// float targetLon = -118.2437; // Example longitude
float radius = 0.1; // Distance in kilometers within which the target is considered reached

void setup() {
  Serial.begin(9600);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  pinMode(LED_BUILTIN, OUTPUT);

  //vibration code
  Serial.begin(9600);
  drv.begin();
  drv.selectLibrary(1);
  // I2C trigger by sending 'go' command 
  // default, internal trigger when sending GO command
  drv.setMode(DRV2605_MODE_INTTRIG); 
}

void loop() {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;

    if (GPS.fix) {
      float distance = calcGeoDistance(GPS.latitudeDegrees, GPS.longitudeDegrees, targetLat, targetLon);

      Serial.print("Latitude: "); Serial.println(GPS.latitudeDegrees, 6);
      Serial.print("Longitude: "); Serial.println(GPS.longitudeDegrees, 6);
      Serial.print("Distance to target: "); Serial.println(distance);

      if (distance < radius) {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("Target Reached!");
        Serial.print("Effect #"); Serial.println(effect);
        // set the effect to play
        drv.setWaveform(0, effect);  // play effect 
        drv.setWaveform(1, 0);       // end waveform
        drv.go();
      } else {
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
    else {
      Serial.println("Looking for a fix...");
    }
  }
}

float calcGeoDistance(float lat1, float lon1, float lat2, float lon2) {
  float earthRadius = 6371; // Earth radius in kilometers
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(lat1)) * cos(radians(lat2)) * 
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return earthRadius * c;
}