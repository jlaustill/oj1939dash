#ifndef O_J1939_DASH_DATA_FRAM_CPP
#define O_J1939_DASH_DATA_FRAM_CPP

#include "./fram.h"

bool OPCM_Fram::isInitialized = true;
Adafruit_FRAM_SPI OPCM_Fram::fram = Adafruit_FRAM_SPI(10); // Initialization


void OPCM_Fram::initialize()
{
    if (fram.begin())
    {
        isInitialized = true;
        Serial.println("Found SPI FRAM");

        // Read the first byte
        uint32_t restarts = 0;
        fram.read(0x0, reinterpret_cast<uint8_t*>(&restarts), sizeof(uint32_t));
        Serial.print("Restarted ");
        Serial.print(restarts++);
        Serial.println(" times");

        // restarts write ++
        fram.writeEnable(true);
        fram.write(0x0, reinterpret_cast<uint8_t*>(&restarts), sizeof(uint32_t));
        fram.writeEnable(false);

        // uint8_t value;
        // for (uint32_t a = 0; a < 256; a++) {
        //   value = fram.read8(a);
        //   if ((a % 32) == 0) {
        //     Serial.print("\n 0x");
        //     Serial.print(a, HEX);
        //     Serial.print(": ");
        //   }
        //   Serial.print("0x");
        //   if (value < 0x1) Serial.print('0');
        //   Serial.print(value, HEX);
        //   Serial.print(" ");
        // }
    }
    else
    {
        Serial.println("No SPI FRAM found ... check your connections\r\n");
    }
}

void OPCM_Fram::saveData(AppData* currentData)
{
    fram.writeEnable(true);

    fram.write(4, reinterpret_cast<uint8_t*>(&currentData->odometer), sizeof(currentData->odometer));
    fram.write(12, reinterpret_cast<uint8_t*>(&currentData->tripA), sizeof(currentData->tripA));
    fram.write(20, reinterpret_cast<uint8_t*>(&currentData->tripB), sizeof(currentData->tripB));
    fram.write(28, reinterpret_cast<uint8_t*>(&currentData->oilChange), sizeof(currentData->oilChange));
    fram.write(36, reinterpret_cast<uint8_t*>(&currentData->transmissionFluidChange),
               sizeof(currentData->transmissionFluidChange));
    fram.write(44, reinterpret_cast<uint8_t*>(&currentData->transferCaseFluidChange),
               sizeof(currentData->transferCaseFluidChange));
    fram.write(52, reinterpret_cast<uint8_t*>(&currentData->frontDifferentialFluidChange),
               sizeof(currentData->frontDifferentialFluidChange));
    fram.write(60, reinterpret_cast<uint8_t*>(&currentData->rearDifferentialFluidChange),
               sizeof(currentData->rearDifferentialFluidChange));
    fram.write(68, reinterpret_cast<uint8_t*>(&currentData->fuelFilterChange), sizeof(currentData->fuelFilterChange));
    fram.write(76, reinterpret_cast<uint8_t*>(&currentData->tireRotation), sizeof(currentData->tireRotation));
    fram.write(84, reinterpret_cast<uint8_t*>(&currentData->odometerSaveCount), sizeof(currentData->odometerSaveCount));
    fram.write(92, reinterpret_cast<uint8_t*>(&currentData->milesOnEngine), sizeof(currentData->milesOnEngine));
    fram.write(100, reinterpret_cast<uint8_t*>(&currentData->milesOnTransmission),
               sizeof(currentData->milesOnTransmission));
    fram.write(108, reinterpret_cast<uint8_t*>(&currentData->milesOnTransferCase),
               sizeof(currentData->milesOnTransferCase));

    fram.writeEnable(false);
}

AppData OPCM_Fram::loadData()
{
    AppData currentData{};

    fram.read(4, reinterpret_cast<uint8_t*>(&currentData.odometer), sizeof(currentData.odometer));
    fram.read(12, reinterpret_cast<uint8_t*>(&currentData.tripA), sizeof(currentData.tripA));
    fram.read(20, reinterpret_cast<uint8_t*>(&currentData.tripB), sizeof(currentData.tripB));
    fram.read(28, reinterpret_cast<uint8_t*>(&currentData.oilChange), sizeof(currentData.oilChange));
    fram.read(36, reinterpret_cast<uint8_t*>(&currentData.transmissionFluidChange),
              sizeof(currentData.transmissionFluidChange));
    fram.read(44, reinterpret_cast<uint8_t*>(&currentData.transferCaseFluidChange),
              sizeof(currentData.transferCaseFluidChange));
    fram.read(52, reinterpret_cast<uint8_t*>(&currentData.frontDifferentialFluidChange),
              sizeof(currentData.frontDifferentialFluidChange));
    fram.read(60, reinterpret_cast<uint8_t*>(&currentData.rearDifferentialFluidChange),
              sizeof(currentData.rearDifferentialFluidChange));
    fram.read(68, reinterpret_cast<uint8_t*>(&currentData.fuelFilterChange),
              sizeof(currentData.fuelFilterChange));
    fram.read(76, reinterpret_cast<uint8_t*>(&currentData.tireRotation), sizeof(currentData.tireRotation));
    fram.read(84, reinterpret_cast<uint8_t*>(&currentData.odometerSaveCount),
              sizeof(currentData.odometerSaveCount));
    fram.read(92, reinterpret_cast<uint8_t*>(&currentData.milesOnEngine), sizeof(currentData.milesOnEngine));
    fram.read(100, reinterpret_cast<uint8_t*>(&currentData.milesOnTransmission),
              sizeof(currentData.milesOnTransmission));
    fram.read(108, reinterpret_cast<uint8_t*>(&currentData.milesOnTransferCase),
              sizeof(currentData.milesOnTransferCase));

    return currentData;
}

#endif  // O_J1939_DASH_DATA_FRAM_CPP
