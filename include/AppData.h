//
// Created by jlaustill on 7/18/21.
//

#ifndef O_J1939_DASH_APPDATA_H
#define O_J1939_DASH_APPDATA_H

#include <Arduino.h>

struct AppData
{
  byte coolantTemp;
  byte coolantTemp2;
  int rpm;
  byte speedInMph;
  float oilPressureInPsi;
  float oilTempC;
  byte transmissionTempC;
  int fuelTempF;
  float boost;
  float manifoldTempC;
  double odometer;
  unsigned long odometerSaveCount;
  double tripA;
  double tripB;
  double egt;
  float fuelPressure;
  int throttlePercentage;
  int load;
  double oilChange;
  double transmissionFluidChange;
  double transferCaseFluidChange;
  double frontDifferentialFluidChange;
  double rearDifferentialFluidChange;
  double fuelFilterChange;
  double tireRotation;
  double milesOnEngine;
  double milesOnTransferCase;
  double milesOnTransmission;
  char requestedRange;
  int8_t currentGear;
  int8_t selectedGear;
};

#endif  // O_J1939_DASH_APPDATA_H
