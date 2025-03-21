//
// Created by jlaustill on 8/21/21.
//
#ifndef O_J1939_DASH_J1939_BUS_H
#define O_J1939_DASH_J1939_BUS_H
#include <FlexCAN_T4.h>
#include <J1939Message.h>
#include <AppData.h>
#include <SeaDash.hpp>
#include <unordered_set>

struct CanMessage
{
    byte id;
    byte length;
    byte data[8];
    unsigned count;
};

class J1939Bus
{
public:
    static void initialize(AppData* appData);
    static void loop();
    static int getCurrentRpms();
    static int getCurrentWaterTemp();
    static byte getCurrentOilPressure();
    static int getCurrentFuelTemp();
    static float getCurrentBoostInPsi();
    static float getCurrentBoostTemp();
    static int getCurrentLoad();
    static int getCurrentThrottlePercentage();
    static float getCurrentAMT();
    static float getCurrentFuelPercentage();
    static float getCurrentEgtTemp();
    static float getCurrentFuelPressurePsi();
    static float getCurrentTiming();
    static byte getTransmissionTempC();
    static byte getVehicleSpeed();
    static char getRequestedRange();
    static int8_t getCurrentGear();
    static int8_t getSelectedGear();

private:
    // CAN hardware interface
    static FlexCAN_T4<CAN1, RX_SIZE_256> Can1;

    // Message storage
    static J1939Message currentMessage;
    static std::unordered_set<uint8_t> sourceAddresses;
    static J1939Message ElectronicTransmissionController1Pgn;

    // CAN messages for different PGNs
    static volatile CanMessage pgn65262;
    static volatile CanMessage pgn65262_149;
    static volatile CanMessage pgn65263;
    static volatile CanMessage pgn65263_149;
    static volatile CanMessage pgn65272;
    static volatile CanMessage pgn65129;
    static volatile CanMessage pgn61445;
    static volatile CanMessage message256;
    static volatile CanMessage message274;
    static volatile CanMessage pgn61443;
    static volatile CanMessage pgn65270;
    static CAN_message_t msg;

    // Sensor data
    static volatile unsigned long RPM;
    static volatile double fuelTemp;
    static volatile double oilPressure;
    static volatile double waterTemp;
    static volatile byte load;
    static volatile byte throttlePercentage;
    static volatile float Timing;
    static volatile float FuelPercentage;

    // Fuel pressure tracking
    static volatile float minFuelPressure;
    static volatile float maxFuelPressure;

    // Timing data
    static volatile float maxTiming;
    static volatile int maxOfThrottleAndLoad;
    static volatile float newTiming;
    static volatile unsigned short shortTimingValue;

    // ID tracking
    static volatile int ids[8];
    static volatile int idsP;

    // Timing
    static uint32_t lastJ1939Request;
    static uint32_t currentMillis;

    // Application data
    static AppData* appData;

    // Speed calculation
    static SeaDash::Uint32::IncrementalCircularAverage speedAverage;

    // Private methods
    static void updateMessage(volatile CanMessage* _messageToUpdate, const CAN_message_t& _msg);
    static void updateJ1939Message(J1939Message* _messageToUpdate, const CAN_message_t& _msg);
    static void requestPgn(const uint32_t pgn);
    static void updateRpms();
    static void updateLoad();
    static void updateThrottlePercentage();
    static void updateTiming();

    // Friend function for CAN interrupt handler
    friend void CumminsBusSniff(const CAN_message_t& _msg);
};

#endif  // O_J1939_DASH_J1939_BUS_H
