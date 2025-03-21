//
// Created by jlaustill on 8/27/21.
//

#include <Arduino.h>
#include <cctype>
#include "Nextion.h"

// Track update timings
unsigned long Nextion::last100msUpdate = 0;
unsigned long Nextion::last1sUpdate = 0;

// Buffer for batching commands
String Nextion::batchCmdBuffer = "";
const unsigned int Nextion::MAX_BUFFER_SIZE = 1024; // Limit buffer size
const unsigned int Nextion::MAX_SERIAL_BUFFER_SIZE = 256; // Limit incoming command size
const unsigned long Nextion::INIT_TIMEOUT = 5000; // 5 second timeout for initialization

void Nextion::sendCmd(const String& cmd)
{
  // Check if adding this command would exceed buffer size
  if (batchCmdBuffer.length() + cmd.length() + 3 > MAX_BUFFER_SIZE)
  {
    // Buffer would overflow, send what we have first
    sendBatch();
  }
  batchCmdBuffer += cmd + "\xFF\xFF\xFF";
}

void Nextion::sendBatch()
{
  if (batchCmdBuffer.length() > 0)
  {
    Serial3.print(batchCmdBuffer);
    batchCmdBuffer = "";
  }
}

bool Nextion::initialize()
{
  Serial3.begin(115200);

  // Wait for Serial3 with timeout
  unsigned long startTime = millis();
  while (!Serial3)
  {
    if (millis() - startTime > INIT_TIMEOUT)
    {
      Serial.println("Nextion initialization timeout!");
      return false;
    }
    delay(10);
  }

  sendCmd(""); // clear the buffer
  sendBatch(); // Send immediately

  delay(100); // Give display time to process

  // Flush any existing data in the incoming buffer
  while (Serial3.available())
  {
    Serial3.read();
  }

  return true;
}

String formatNumber(const double number)
{
  String finalText = "";
  char buffer[20]; // Adjust the buffer size according to your needs

  // Convert the input number to a formatted string
  snprintf(buffer, sizeof(buffer), "%.2f", number);

  int numDigits = 0;
  size_t periodIndex = strcspn(buffer, "."); // Find the position of the decimal point

  for (size_t i = 0; i < periodIndex; i++)
  {
    if (isdigit(buffer[i]))
    {
      numDigits++;
    }
  }

  // Loop through the input string and copy each number to the output string,
  // inserting commas along the way
  for (size_t i = 0; i < periodIndex; i++)
  {
    finalText += buffer[i];

    // Decrease the distance from the decimal point
    size_t distance = numDigits - i - 1;

    // Insert a comma every three decimal positions away from the decimal point
    if ((distance > 0) && (distance % 3 == 0))
    {
      finalText += ',';
    }
  }

  finalText += '.';
  finalText += buffer[periodIndex + 1];

  return finalText;
}

void Nextion::updateDisplayData(AppData* currentData)
{
  unsigned long currentMillis = millis();

  // Update the display every 100ms
  if (currentMillis - last100msUpdate >= 100)
  {
    sendCmd("mph.val=" + static_cast<String>(currentData->speedInMph));
    sendCmd("rpm.val=" + static_cast<String>(currentData->rpm));
    const String bpt = "bp.val=" + static_cast<String>(static_cast<int>(currentData->boost));
    sendCmd(bpt);
    const float boostTempF = (currentData->manifoldTempC * 9.0f / 5.0f) + 32.0f;
    const String btt = "bt.val=" + static_cast<String>(static_cast<int>(boostTempF));
    sendCmd(btt);
    sendCmd("throttle.val=" + static_cast<String>(currentData->throttlePercentage));
    sendCmd("selGear.val=" + static_cast<String>(currentData->selectedGear));
    sendCmd("curGear.val=" + static_cast<String>(currentData->currentGear));
    sendCmd("reqRange.txt=\"" + static_cast<String>(currentData->requestedRange) + "\"");
    sendCmd("load.val=" + static_cast<String>(currentData->load));
    sendCmd("fuelPres.val=" + static_cast<String>(static_cast<int>(currentData->fuelPressure)));

    sendBatch();
    last100msUpdate = currentMillis;
  }

  // Update slow data every 1 second
  if (currentMillis - last1sUpdate >= 1000)
  {
    sendCmd("odometer.txt=\"" + formatNumber(currentData->odometer) + "\"");
    sendCmd("tripA.txt=\"" + formatNumber(currentData->tripA) + "\"");
    // ... other 1s updates ...

    // Send batch commands
    sendBatch();
    last1sUpdate = currentMillis;
  }

  // Process incoming commands with a limit on how long we spend doing this
  // to avoid getting stuck in command processing
  const unsigned long processStartTime = millis();
  constexpr unsigned long MAX_PROCESS_TIME = 50; // max 50ms processing commands

  processCommands(currentData, processStartTime, MAX_PROCESS_TIME);
}

void Nextion::processCommands(AppData* currentData, const unsigned long startTime, unsigned long maxTime)
{
  static String serialBuffer = "";

  // Process commands with time limit
  while (Serial3.available() && (millis() - startTime < maxTime))
  {
    const int newData = Serial3.read();

    // Check for buffer overflow
    if (serialBuffer.length() >= MAX_SERIAL_BUFFER_SIZE)
    {
      Serial.println("Serial buffer overflow, clearing");
      serialBuffer = "";
    }

    if (newData == ';')
    {
      Serial.println("Execute!" + serialBuffer);

      // Process commands with proper prefix matching
      if (serialBuffer.startsWith("resetTripA"))
      {
        Serial.println("reset trip A!");
        currentData->tripA = 0;
      }
      else if (serialBuffer.startsWith("resetTripB"))
      {
        Serial.println("reset trip B!");
        currentData->tripB = 0;
      }
      // ... other reset commands ...

      serialBuffer = ""; // Clear buffer after processing
    }
    else
    {
      serialBuffer += static_cast<char>(newData);
    }
  }
}
