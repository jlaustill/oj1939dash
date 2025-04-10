#define thirdByte(w) ((uint8_t)((w) >> 16))
#define fourthByte(w) ((uint8_t)((w) >> 24))

#include "OJ1939Dash.h"
#include <AppData.h>
#include <Arduino.h>
#include <cmath>
#include <cerrno>   // For errno and ERANGE
#include <cstdlib>  // For strtod function
#include "Data/J1939Bus.h"
#include "Display/Nextion.h"

#include "Data/fram.h"

AppData currentData;
String serialBuffer;

float roundToTwo(const float var)
{
  const long value = std::lround(var * 100.0f);
  return static_cast<float>(value) / 100.0f;
}

void OJ1939Dash::newSweepValue()
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

unsigned long OJ1939Dash::count = 0;
unsigned long OJ1939Dash::lastMillis = 0;
unsigned long OJ1939Dash::thisMillis = 0;
unsigned long OJ1939Dash::thisDuration = 0;
unsigned long OJ1939Dash::lastOdometerUpdate = 0;
float OJ1939Dash::thisMileage = 0;

long OJ1939Dash::sweep = 0;
long OJ1939Dash::maxSweep = 50;
int OJ1939Dash::up = 1;

unsigned long loopCountLastMillis = 0;

void OJ1939Dash::setup()
{
  Serial.begin(115200);
  count = 0;
  lastMillis = millis();
  thisMillis = millis();
  lastOdometerUpdate = 0;
  thisDuration = 0;
  thisMileage = 0;

  Serial.println("Starting up...");

  if (!O_J1939_Dash_Fram::initialize())
  {
    Serial.println("FRAM initialization failed!");
  }

  // Check if J1939Bus initialization was successful
  if (!J1939Bus::initialize(&currentData))
  {
    Serial.println("CAN bus initialization failed!");
    // Consider adding some form of user feedback
    // For example, you could set a global error flag or blink an LED
  }

  if (!O_J1939_Dash_Fram::loadData(&currentData))
  {
    Serial.println("Failed to load odometer data!");
  }

  if (!Nextion::initialize())
  {
    Serial.println("Warning: Nextion display initialization failed");
    // Consider fallback behavior
  }
}

int newData = 0;

void OJ1939Dash::loop()
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

    if (!O_J1939_Dash_Fram::saveData(&currentData))
    {
      Serial.println("Failed to save odometer data!");
      // Consider retry logic or error handling
    }
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
      else if (serialBuffer.indexOf("setOdometer") == 0)
      {
        // Changed from > 0 to == 0
        // Find the position of the equals sign

        // Check if we have a valid format: "setOdometer=X"
        if (const uint32_t equalsPos = serialBuffer.indexOf("="); equalsPos > 0 && equalsPos < serialBuffer.length() -
          1)
        {
          // Extract the number substring
          String valueStr = serialBuffer.substring(equalsPos + 1);

          // Convert to C-style string for strtod
          const char* str = valueStr.c_str();
          char* end;

          // Convert string to double with error checking
          errno = 0;
          const double newOdometerReading = strtod(str, &end);

          // Check for conversion success
          if (end != str && errno != ERANGE && *end == '\0')
          {
            // Valid number, update odometer
            currentData.odometer = newOdometerReading;
            Serial.print("Set Odometer = ");
            Serial.println(currentData.odometer);
          }
          else
          {
            // Invalid number format
            Serial.println("Invalid odometer value");
          }
        }
        else
        {
          Serial.println("Invalid setOdometer format");
        }
      }

      // Clear buffer after processing command
      serialBuffer = "";
    }
    else
    {
      serialBuffer += static_cast<char>(newData);
    }
  }
}
