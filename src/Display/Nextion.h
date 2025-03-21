//
// Created by jlaustill on 8/27/21.
//
#ifndef O_J1939_DASH_NEXTION_H
#define O_J1939_DASH_NEXTION_H

#include <Arduino.h>
#include "../../include/AppData.h"

class Nextion {
 public:
  static void initialize();
  static void updateDisplayData(AppData *currentData);

 private:
  static void sendCmd(const String& cmd);
  static void sendBatch();
  static void processCommands(AppData *currentData);
};

#endif  // O_J1939_DASH_NEXTION_H