#ifndef O_J1939_DASH_NEXTION_H
#define O_J1939_DASH_NEXTION_H

#include <Arduino.h>
#include "../../include/AppData.h"

class Nextion
{
public:
 static bool initialize(); // Return success/failure
 static void updateDisplayData(const AppData* currentData);

private:
 static void sendCmd(const String& cmd);
 static void sendBatch();
 static void processCommands(AppData* currentData, unsigned long startTime, unsigned long maxTime);

 // Configuration constants
 static const unsigned int MAX_BUFFER_SIZE;
 static const unsigned int MAX_SERIAL_BUFFER_SIZE;
 static const unsigned long INIT_TIMEOUT;

 // Timing variables
 static unsigned long last100msUpdate;
 static unsigned long last1sUpdate;

 // Command buffer
 static String batchCmdBuffer;
};

#endif // O_J1939_DASH_NEXTION_H
