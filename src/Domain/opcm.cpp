#define thirdByte(w) ((uint8_t)((w) >> 16))
#define fourthByte(w) ((uint8_t)((w) >> 24))

#include "opcm.h"
#include <AppData.h>
#include <Arduino.h>
#include "Data/J1939Bus.h"
#include "Display/Nextion.h"

#include "Data/fram.h"

AppData currentData;
String serialBuffer;

float roundToTwo(const float var)
{
  const int value = static_cast<int>(var * 100.0f + .5f);
  return static_cast<float>(value) / 100.0f;
}

void opcm::newSweepValue()
{
  if (up == 1 && sweep < maxSweep)
  {
    sweep++;
  }
  else if (up == 1 && sweep >= maxSweep)
  {
    up = 0;
  }
  else if (sweep > 0)
  {
    sweep--;
  }
  else
  {
    up = 1;
  }
}

unsigned long opcm::count = 0;
unsigned long opcm::lastMillis = 0;
unsigned long opcm::thisMillis = 0;
unsigned long opcm::thisDuration = 0;
unsigned long opcm::lastOdometerUpdate = 0;
float opcm::thisMileage = 0;

long opcm::sweep = 0;
long opcm::maxSweep = 50;
int opcm::up = 1;

unsigned long loopCountLastMillis = 0;

void opcm::setup()
{
  Serial.begin(115200);
  count = 0;
  lastMillis = millis();
  thisMillis = millis();
  lastOdometerUpdate = 0;
  thisDuration = 0;
  thisMileage = 0;

  Serial.println("Starting up...");

  OPCM_Fram::initialize();

  J1939Bus::initialize(&currentData);

  currentData = OPCM_Fram::loadData();

  currentData.rpm = 0;
  currentData.coolantTemp = 0;
  currentData.speedInMph = 0;
  currentData.transmissionPressure = 0;
  currentData.oilPressureInPsi = 0;
  currentData.fuelTempF = 0;
  currentData.boost = 0;
  currentData.manifoldTempC = 0;

  currentData.timing = 0;
  currentData.fuelPercentage = 0;
  currentData.amt = 0;
  currentData.throttlePercentage = 0;
  currentData.load = 0;
  currentData.fuelPressure = 0.0;
  Nextion::initialize();
}

int newData = 0;

void opcm::loop()
{
  thisMillis = millis();
  thisDuration = thisMillis - lastMillis;
  count++;
  newSweepValue();

  J1939Bus::loop();
  currentData.rpm = J1939Bus::getCurrentRpms();
  currentData.coolantTemp = J1939Bus::getCurrentWaterTemp();
  currentData.oilPressureInPsi = J1939Bus::getCurrentOilPressure();
  currentData.fuelTempF = J1939Bus::getCurrentFuelTemp();
  currentData.boost = J1939Bus::getCurrentBoostInPsi();
  currentData.manifoldTempC = J1939Bus::getCurrentBoostTemp();

  currentData.timing = J1939Bus::getCurrentTiming();
  currentData.fuelPercentage = J1939Bus::getCurrentFuelPercentage();
  currentData.amt = J1939Bus::getCurrentAMT();
  currentData.throttlePercentage = J1939Bus::getCurrentThrottlePercentage();
  currentData.load = J1939Bus::getCurrentLoad();
  currentData.transmissionTempC = J1939Bus::getTransmissionTempC();
  currentData.speedInMph = J1939Bus::getVehicleSpeed();
  currentData.requestedRange = J1939Bus::getRequestedRange();
  currentData.currentGear = J1939Bus::getCurrentGear();
  currentData.selectedGear = J1939Bus::getSelectedGear();
  currentData.egt = J1939Bus::getCurrentEgtTemp();
  currentData.fuelPressure = J1939Bus::getCurrentFuelPressurePsi();

  thisMileage += (static_cast<float>(currentData.speedInMph) / 3600000.0f *
    static_cast<float>(thisDuration));

  if (thisMileage >= 0.1 || (thisMillis - lastOdometerUpdate) > 10)
  {
    lastOdometerUpdate = thisMillis;
    currentData.odometer += thisMileage;
    currentData.tripA += thisMileage;
    currentData.tripB += thisMileage;
    currentData.oilChange += thisMileage;
    currentData.transmissionFluidChange += thisMileage;
    currentData.transferCaseFluidChange += thisMileage;
    currentData.frontDifferentialFluidChange += thisMileage;
    currentData.rearDifferentialFluidChange += thisMileage;
    currentData.fuelFilterChange += thisMileage;
    currentData.tireRotation += thisMileage;
    currentData.milesOnEngine += thisMileage;
    currentData.milesOnTransmission += thisMileage;
    currentData.milesOnTransferCase += thisMileage;
    currentData.odometerSaveCount++;

    thisMileage = 0;

    OPCM_Fram::saveData(&currentData);
  }

  Nextion::updateDisplayData(&currentData);

  lastMillis = thisMillis;

  if (thisMillis - loopCountLastMillis > 1000)
  {
    loopCountLastMillis = thisMillis;
    Serial.println(
      "Loop Count/Sec: " + static_cast<String>(count) + " Miles On Engine: " + static_cast<String>(currentData.
        milesOnEngine) + " RPM: " + static_cast<String>(currentData.rpm));
    count = 0;
  }

  while (Serial.available())
  {
    newData = Serial.read();
    if (newData == ';')
    {
      Serial.println("Execute!" + serialBuffer);
      if (serialBuffer == "resetTripA")
      {
        Serial.println("reset trip A!");
        currentData.tripA = 0;
      }
      else if (serialBuffer.indexOf("setOdometer") > 0)
      {
        const double newOdometerReading =
          atof(serialBuffer
               .substring(serialBuffer.indexOf("=") + 1,
                          serialBuffer.indexOf(";"))
               .c_str());
        currentData.odometer = newOdometerReading;
        Serial.print("Set Odometer = ");
        Serial.println(currentData.odometer);
      }
      serialBuffer = "";
    }
    else
    {
      serialBuffer += static_cast<char>(newData);
    }
  }
}
