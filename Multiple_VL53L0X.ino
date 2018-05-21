/* This example shows how to get single-shot range
  measurements from the VL53L0X. The sensor can optionally be
  configured with different ranging profiles, as described in
  the VL53L0X API user manual, to get better performance for
  a certain application. This code is based on the four
  "SingleRanging" examples in the VL53L0X API.

  The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
VL53L0X sensor2;

const int xshutPin = 2;
const int xshutPin2 = 3;

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
#define HIGH_ACCURACY


void setup()
{
  pinMode(xshutPin, OUTPUT);
  digitalWrite(xshutPin, LOW);

  delay(10);

  pinMode(xshutPin2, OUTPUT);
  digitalWrite(xshutPin2, LOW);

  Serial.begin(115200);
  Wire.begin();

  digitalWrite(xshutPin, HIGH);
  delay(10);
  sensor.init();
  sensor.setTimeout(200);
  sensor.setAddress((uint8_t)48);
  delay(10);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif

  digitalWrite(xshutPin2, HIGH);
  delay(10);
  sensor2.init();
  sensor2.setTimeout(200);
  sensor2.setAddress((uint8_t)49);
  delay(10);

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor2.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor2.setMeasurementTimingBudget(200000);
#endif
}

void loop()
{
  Serial.println("------------------------------------------------------------");
  Serial.print("Distance from Sensor 1 : ");
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }
  Serial.println();

  Serial.print("Distance from Sensor 2 : ");
  Serial.print(sensor2.readRangeSingleMillimeters());
  if (sensor2.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }
  Serial.println();
  Serial.println("------------------------------------------------------------");
  Serial.println();
}
