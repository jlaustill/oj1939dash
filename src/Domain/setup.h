#ifndef OPCM_SETUP_H
#define OPCM_SETUP_H

#include <AppData.h>
#include <Arduino.h>

#ifdef CUMMINS_BUS_INPUT
#include "Data/CumminsBus.h"
#endif

#ifdef NEXTION
#include "Display/Nextion.h"
#endif

AppData currentData;
String serialBuffer;

float roundToTwo(float var) {
  float value = (int)(var * 100 + .5);
  return (float)value / 100.0;
}

#endif  // OPCM_SETUP_H