//
// Created by jlaustill on 8/21/21.
//
#ifndef O_J1939_DASH_CUMMINS_BUS_H
#define O_J1939_DASH_CUMMINS_BUS_H
#include <FlexCAN_T4.h>
#include <J1939Message.h>
#include <AppData.h>

struct CanMessage
{
    byte id;
    byte length;
    byte data[8];
    unsigned count;
};

class CumminsBus
{
public:
    static AppData* appData;
    static void initialize(AppData* appData);
    static void loop();
    static void updateRpms();
    static int getCurrentRpms();
    static int getCurrentWaterTemp();
    static byte getCurrentOilPressure();
    static int getCurrentFuelTemp();
    static float getCurrentBoostInPsi();
    static float getCurrentBoostTemp();
    static void updateLoad();
    static int getCurrentLoad();
    static void updateThrottlePercentage();
    static int getCurrentThrottlePercentage();
    static float getCurrentAMT();
    static float getCurrentFuelPercentage();
    static float getCurrentEgtTemp();
    static float getCurrentFuelPressurePsi();
    static void updateTiming();
    static float getCurrentTiming();
    static void updateTiming(CAN_message_t& msg);
    static byte getTransmissionTempC();
    static byte getVehicleSpeed();
    static char getRequestedRange();
    static int8_t getCurrentGear();
    static int8_t getSelectedGear();
    static J1939Message ElectronicTransmissionController1Pgn;
};

#endif  // O_J1939_DASH_CUMMINS_BUS_H
