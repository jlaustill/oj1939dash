#ifndef O_J1939_DASH_DATA_FRAM_CPP
#define O_J1939_DASH_DATA_FRAM_CPP

#include <SPI.h>
#include "./fram.h"

bool O_J1939_Dash_Fram::isInitialized = false; // Start uninitialized
Adafruit_FRAM_SPI O_J1939_Dash_Fram::fram = Adafruit_FRAM_SPI(10);
unsigned long O_J1939_Dash_Fram::lastWriteTime = 0;

bool O_J1939_Dash_Fram::initialize()
{
    isInitialized = false; // Reset initialization flag

    if (fram.begin())
    {
        isInitialized = true;
        Serial.println("Found SPI FRAM");

        // Read the first byte
        uint32_t restarts = 0;
        if (!fram.read(0x0, reinterpret_cast<uint8_t*>(&restarts), sizeof(uint32_t)))
        {
            Serial.println("Error reading restart count");
            return false;
        }

        Serial.print("Restarted ");
        Serial.print(restarts++);
        Serial.println(" times");

        // restarts write ++
        fram.writeEnable(true);
        bool writeSuccess = fram.write(0x0, reinterpret_cast<uint8_t*>(&restarts), sizeof(uint32_t));
        fram.writeEnable(false);

        if (!writeSuccess)
        {
            Serial.println("Error writing restart count");
            return false;
        }

        return true;
    }
    else
    {
        Serial.println("No SPI FRAM found ... check your connections\r\n");
        return false;
    }
}

bool O_J1939_Dash_Fram::saveData(AppData* currentData)
{
    // Check if initialized
    if (!isInitialized)
    {
        Serial.println("FRAM not initialized");
        return false;
    }

    // Throttle writes - don't write more often than every 5 seconds
    unsigned long currentTime = millis();
    if (currentTime - lastWriteTime < 5000)
    {
        return true; // Skip this write but don't report error
    }

    // Start a transaction for better reliability
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    fram.writeEnable(true);
    bool success = true;

    // Use a macro or inline function to simplify and make the pattern consistent
#define WRITE_FIELD(addr, field) \
        if (!fram.write(addr, reinterpret_cast<uint8_t*>(&currentData->field), sizeof(currentData->field))) { \
            success = false; \
            Serial.print("FRAM write failed at address "); \
            Serial.println(addr); \
            break; \
        }

    do
    {
        WRITE_FIELD(4, odometer);
        WRITE_FIELD(12, tripA);
        WRITE_FIELD(20, tripB);
        WRITE_FIELD(28, oilChange);
        WRITE_FIELD(36, transmissionFluidChange);
        WRITE_FIELD(44, transferCaseFluidChange);
        WRITE_FIELD(52, frontDifferentialFluidChange);
        WRITE_FIELD(60, rearDifferentialFluidChange);
        WRITE_FIELD(68, fuelFilterChange);
        WRITE_FIELD(76, tireRotation);
        WRITE_FIELD(84, odometerSaveCount);
        WRITE_FIELD(92, milesOnEngine);
        WRITE_FIELD(100, milesOnTransmission);
        WRITE_FIELD(108, milesOnTransferCase);
    }
    while (false); // Using do-while(0) to allow for breaking out on error

    fram.writeEnable(false);
    SPI.endTransaction();

    if (success)
    {
        lastWriteTime = currentTime;
    }

    return success;
}

bool O_J1939_Dash_Fram::loadData(AppData* currentData)
{
    // Check if initialized
    if (!isInitialized)
    {
        Serial.println("FRAM not initialized");
        return false;
    }

    // Start a transaction
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    bool success = true;

#define READ_FIELD(addr, field) \
        if (!fram.read(addr, reinterpret_cast<uint8_t*>(&currentData->field), sizeof(currentData->field))) { \
            success = false; \
            Serial.print("FRAM read failed at address "); \
            Serial.println(addr); \
            break; \
        }

    do
    {
        READ_FIELD(4, odometer);
        READ_FIELD(12, tripA);
        READ_FIELD(20, tripB);
        READ_FIELD(28, oilChange);
        READ_FIELD(36, transmissionFluidChange);
        READ_FIELD(44, transferCaseFluidChange);
        READ_FIELD(52, frontDifferentialFluidChange);
        READ_FIELD(60, rearDifferentialFluidChange);
        READ_FIELD(68, fuelFilterChange);
        READ_FIELD(76, tireRotation);
        READ_FIELD(84, odometerSaveCount);
        READ_FIELD(92, milesOnEngine);
        READ_FIELD(100, milesOnTransmission);
        READ_FIELD(108, milesOnTransferCase);
    }
    while (false);

    SPI.endTransaction();

    return success;
}

#endif  // O_J1939_DASH_DATA_FRAM_CPP
