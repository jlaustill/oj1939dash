//
// Created by jlaustill on 8/27/21.
//
#ifndef OPCM_NEXTION_H
#define OPCM_NEXTION_H

#include <Arduino.h>
#include "../../include/AppData.h"

class Nextion {
 public:
  static void initialize();
  static void updateDisplayData(AppData *currentData);

 private:
  static void sendCmd(String cmd);
  static void sendBatch();
  static void processCommands(AppData *currentData);
};

#endif  // OPCM_NEXTION_H