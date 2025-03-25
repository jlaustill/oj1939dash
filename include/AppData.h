//
// Created by jlaustill on 7/18/21.
//

#ifndef O_J1939_DASH_APPDATA_H
#define O_J1939_DASH_APPDATA_H

#include <Arduino.h>

struct AppData
{
  byte coolantTemp = 0;
  byte coolantTemp2 = 0;
  int rpm = 0;
  byte speedInMph = 0;
  float oilPressureInPsi = 0;
  float oilTempC = 0;
  byte transmissionTempC = 0;
  int fuelTempF = 0;
  float boost = 0;
  float manifoldTempC = 0;
  double odometer = 0;
  unsigned long odometerSaveCount = 0;
  double tripA = 0;
  double tripB = 0;
  double egt = 0;
  float fuelPressure = 0;
  int throttlePercentage = 0;
  int load = 0;
  double oilChange = 0;
  double transmissionFluidChange = 0;
  double transferCaseFluidChange = 0;
  double frontDifferentialFluidChange = 0;
  double rearDifferentialFluidChange = 0;
  double fuelFilterChange = 0;
  double tireRotation = 0;
  double milesOnEngine = 0;
  double milesOnTransferCase = 0;
  double milesOnTransmission = 0;
  char requestedRange = 'P';
  int8_t currentGear = 0;
  int8_t selectedGear = 0;
};

#endif  // O_J1939_DASH_APPDATA_H
