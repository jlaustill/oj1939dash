#ifndef O_J1939_DASH_FRAM_H
#define O_J1939_DASH_FRAM_H

#include "AppData.h"
#include "Adafruit_FRAM_SPI.h"

class O_J1939_Dash_Fram
{
private:
 static bool isInitialized;
 static Adafruit_FRAM_SPI fram;
 static unsigned long lastWriteTime; // Track last write time

public:
 static bool initialize();
 static bool saveData(AppData* currentData);
 static bool loadData(AppData* currentData); // Changed to pass by reference
 static bool isReady() { return isInitialized; }
};

#endif  // O_J1939_DASH_FRAM_H
