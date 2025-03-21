//
// Created by jlaustill on 8/21/21.
//

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <J1939Message.h>
#include <SeaDash.hpp>
#include <cstdint>
#include <unordered_set>

#include "J1939Bus.h"
#include "J1939/pgn_constants.h"

// Initialize static members
FlexCAN_T4<CAN1, RX_SIZE_256> J1939Bus::Can1;
J1939Message J1939Bus::currentMessage;
std::unordered_set<uint8_t> J1939Bus::sourceAddresses;
uint32_t J1939Bus::lastJ1939Request = 0;
uint32_t J1939Bus::currentMillis = 0;
AppData* J1939Bus::appData = nullptr;
J1939Message J1939Bus::ElectronicTransmissionController1Pgn{};

// Initialize CAN message containers
volatile CanMessage J1939Bus::pgn65262 = {0x67, 0x8, {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, 0};
volatile CanMessage J1939Bus::pgn65262_149{};
volatile CanMessage J1939Bus::pgn65263{};
volatile CanMessage J1939Bus::pgn65263_149{};
volatile CanMessage J1939Bus::pgn65272{};
volatile CanMessage J1939Bus::pgn65129{};
volatile CanMessage J1939Bus::pgn61445{};
volatile CanMessage J1939Bus::message256{};
volatile CanMessage J1939Bus::message274{};
volatile CanMessage J1939Bus::pgn61443 = {0x67, 0x8, {0xF1, 0x0, 0x0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 0};
volatile CanMessage J1939Bus::pgn65270{};
CAN_message_t J1939Bus::msg;

// Initialize sensor data
volatile unsigned long J1939Bus::RPM = 0;
volatile double J1939Bus::fuelTemp = 0;
volatile double J1939Bus::oilPressure = 0;
volatile double J1939Bus::waterTemp = 0;
volatile byte J1939Bus::load = 0;
volatile byte J1939Bus::throttlePercentage = 0;
volatile float J1939Bus::Timing = 0;
volatile float J1939Bus::FuelPercentage = 0;

// Initialize fuel pressure tracking
volatile float J1939Bus::minFuelPressure = 99999999.0;
volatile float J1939Bus::maxFuelPressure = 0.0;

// Initialize timing data
volatile float J1939Bus::maxTiming = 15.0f;
volatile int J1939Bus::maxOfThrottleAndLoad = 0;
volatile float J1939Bus::newTiming = 15.0f;
volatile unsigned short J1939Bus::shortTimingValue = 0;

// Initialize ID tracking
volatile int J1939Bus::ids[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile int J1939Bus::idsP = 0;

// Initialize speed calculation
SeaDash::Uint32::IncrementalCircularAverage J1939Bus::speedAverage(10);

void J1939Bus::updateJ1939Message(J1939Message* _messageToUpdate, const CAN_message_t& _msg)
{
    _messageToUpdate->setCanId(_msg.id);
    _messageToUpdate->setData(_msg.buf);
}

void J1939Bus::updateMessage(volatile CanMessage* _messageToUpdate, const CAN_message_t& _msg)
{
    _messageToUpdate->id = _msg.id;
    _messageToUpdate->length = _msg.len;
    _messageToUpdate->data[0] = _msg.buf[0];
    _messageToUpdate->data[1] = _msg.buf[1];
    _messageToUpdate->data[2] = _msg.buf[2];
    _messageToUpdate->data[3] = _msg.buf[3];
    _messageToUpdate->data[4] = _msg.buf[4];
    _messageToUpdate->data[5] = _msg.buf[5];
    _messageToUpdate->data[6] = _msg.buf[6];
    _messageToUpdate->data[7] = _msg.buf[7];
    _messageToUpdate->count++;
}

void J1939Bus::requestPgn(const uint32_t pgn)
{
    auto tempMessage = J1939Message();
    tempMessage.setPgn(59904);
    tempMessage.setSourceAddress(51); // 248 works
    tempMessage.setPriority(3);

    msg.flags.extended = true;

    msg.id = tempMessage.canId;
    msg.len = 3;
    msg.buf[0] = SeaDash::Bytes::getNthByte(pgn, 1);
    msg.buf[1] = SeaDash::Bytes::getNthByte(pgn, 2);
    msg.buf[2] = SeaDash::Bytes::getNthByte(pgn, 3);
    Can1.write(msg);
}

// Global function that needs access to J1939Bus internals
void CumminsBusSniff(const CAN_message_t& _msg)
{
    J1939Bus::currentMessage = J1939Message();
    J1939Bus::currentMessage.setCanId(_msg.id);
    J1939Bus::currentMessage.setData(_msg.buf);

    J1939Bus::sourceAddresses.insert(J1939Bus::currentMessage.sourceAddress);

    // First switch statement for message.canId
    switch (J1939Bus::currentMessage.canId)
    {
    case 256:
        J1939Bus::updateMessage(&J1939Bus::message256, _msg);
        return;

    case 274:
        J1939Bus::updateMessage(&J1939Bus::message274, _msg);
        return;
    default:
        // Handle other canId cases if necessary
        break;
    }

    // Second switch statement for message.pgn
    switch (J1939Bus::currentMessage.pgn)
    {
    case 61443:
        J1939Bus::updateMessage(&J1939Bus::pgn61443, _msg);
        break;

    case 65129:
        J1939Bus::updateMessage(&J1939Bus::pgn65129, _msg);
        break;

    case ENGINE_TEMP_1_PGN:
        if (J1939Bus::currentMessage.sourceAddress == 149)
        {
            J1939Bus::updateMessage(&J1939Bus::pgn65262_149, _msg);
        }
        else
        {
            J1939Bus::updateMessage(&J1939Bus::pgn65262, _msg);
        }
        break;

    case 65263:
        if (J1939Bus::currentMessage.sourceAddress == 149)
        {
            J1939Bus::updateMessage(&J1939Bus::pgn65263_149, _msg);
        }
        else
        {
            J1939Bus::updateMessage(&J1939Bus::pgn65263, _msg);
        }
        break;

    case 65270:
        if (J1939Bus::currentMessage.sourceAddress == 149)
        {
            J1939Bus::updateMessage(&J1939Bus::pgn65270, _msg);
        }
        break;

    case 65272:
        J1939Bus::updateMessage(&J1939Bus::pgn65272, _msg);
        break;

    case 61442:
        J1939Bus::updateJ1939Message(&J1939Bus::ElectronicTransmissionController1Pgn, _msg);
        break;

    case 61445:
        J1939Bus::updateMessage(&J1939Bus::pgn61445, _msg);
        break;

    case DM1_DTCS_PGN:
        {
            // This is a DTC message - keep the diagnostic printing as it's useful
            const uint8_t mil = SeaDash::Bits::getNBits(J1939Bus::currentMessage.data[0], 6, 2);
            const uint8_t rsl = SeaDash::Bits::getNBits(J1939Bus::currentMessage.data[0], 4, 2);
            const uint8_t awl = SeaDash::Bits::getNBits(J1939Bus::currentMessage.data[0], 2, 2);
            const uint8_t pls = SeaDash::Bits::getNBits(J1939Bus::currentMessage.data[0], 0, 2);
            const uint8_t milBlink = SeaDash::Bits::getNBits(J1939Bus::currentMessage.data[1], 6, 2);
            const uint8_t rslBlink = SeaDash::Bits::getNBits(J1939Bus::currentMessage.data[1], 4, 2);
            const uint8_t awlBlink = SeaDash::Bits::getNBits(J1939Bus::currentMessage.data[1], 2, 2);
            const uint8_t plsBlink = SeaDash::Bits::getNBits(J1939Bus::currentMessage.data[1], 0, 2);
            uint32_t spn = J1939Bus::currentMessage.data[2];
            spn = SeaDash::Bits::setNBitsAt<uint32_t>(spn, J1939Bus::currentMessage.data[3], 8, 8);
            const auto spnLeastSignificantBits =
                SeaDash::Bits::getNBits<uint8_t>(J1939Bus::currentMessage.data[4], 5, 3);
            spn = SeaDash::Bits::setNBitsAt<uint32_t>(spn, spnLeastSignificantBits, 16, 3);
            const uint8_t fmi = SeaDash::Bits::getNBits(J1939Bus::currentMessage.data[4], 0, 5);
            const uint8_t oc = SeaDash::Bits::getNBits(J1939Bus::currentMessage.data[5], 0, 7);
            Serial.println(
                "DM1 DTC: ID: " + static_cast<String>(J1939Bus::currentMessage.canId) +
                " SA: " + static_cast<String>(J1939Bus::currentMessage.sourceAddress) + " SPN: " + static_cast<String>(spn) +
                " Failure Mode Indicator: " + static_cast<String>(fmi) +
                " Occurrence Count: " + static_cast<String>(oc) + " " + " MIL " + static_cast<String>(mil) +
                " " + " RSL " + static_cast<String>(rsl) + " " + " AWL " + static_cast<String>(awl) + " " +
                " PLS " + static_cast<String>(pls) + " MIL Blink " + static_cast<String>(milBlink) + " " +
                " RSL Blink" + static_cast<String>(rslBlink) + " " + " AWL Blink" +
                static_cast<String>(awlBlink) + " " + " PLS Blink" + static_cast<String>(plsBlink) +
                " Data: " + static_cast<String>(J1939Bus::currentMessage.data[0]) + " " + static_cast<String>(J1939Bus::currentMessage.data[1]) +
                " " + static_cast<String>(J1939Bus::currentMessage.data[2]) + " " + static_cast<String>(J1939Bus::currentMessage.data[3]) + " " +
                static_cast<String>(J1939Bus::currentMessage.data[4]) + " " + static_cast<String>(J1939Bus::currentMessage.data[5]) + " " +
                static_cast<String>(J1939Bus::currentMessage.data[6]) + " " + static_cast<String>(J1939Bus::currentMessage.data[7]));
        }
        break;

    default:
        // Fix potential array bounds issue
        if (J1939Bus::idsP < 8) {
            J1939Bus::ids[J1939Bus::idsP] = static_cast<int>(_msg.id);
        }
        J1939Bus::idsP = (J1939Bus::idsP + 1) % 8;
        break;
    }
}

void J1939Bus::updateTiming()
{
    // compute timing advance
    Timing = static_cast<float>(message256.data[5] << 8 | message256.data[4]); // convert from little endian
    Timing = Timing / 128.0f;
}

float J1939Bus::getCurrentTiming()
{
    updateTiming();
    return Timing;
}

float J1939Bus::getCurrentFuelPressurePsi()
{
    auto fuelPressure = static_cast<float>(pgn65263_149.data[0] * 4);
    fuelPressure /= 6.895;
    if (fuelPressure > maxFuelPressure) maxFuelPressure = fuelPressure;
    if (fuelPressure < minFuelPressure) minFuelPressure = fuelPressure;
    return fuelPressure;
}

byte J1939Bus::getTransmissionTempC()
{
    const std::uint16_t tempRaw = pgn65272.data[5] << 8 | pgn65272.data[4];
    const float transmissionTemp = static_cast<float>(tempRaw) * 0.03125f - 273.15f;
    return static_cast<byte>(transmissionTemp);
}

float J1939Bus::getCurrentFuelPercentage()
{
    // Fuel compute
    FuelPercentage = static_cast<float>((message256.data[1] << 8) | message256.data[0]);
    FuelPercentage = FuelPercentage * 100.0f / 4096.0f; // 4095 is max fuel allowed by pump
    return FuelPercentage;
}

void J1939Bus::updateThrottlePercentage()
{
    throttlePercentage = static_cast<byte>(static_cast<float>(pgn61443.data[1]) * .4f);
}

int J1939Bus::getCurrentThrottlePercentage()
{
    updateThrottlePercentage();
    return throttlePercentage;
}

void J1939Bus::updateLoad()
{
    load = static_cast<byte>(static_cast<float>(pgn61443.data[2]) * 0.8f);
}

int J1939Bus::getCurrentLoad()
{
    updateLoad();
    return load;
}

void J1939Bus::updateRpms()
{
    RPM = message256.data[7] << 8 | message256.data[6]; // convert from little endian
    RPM /= 4;
}

int J1939Bus::getCurrentRpms()
{
    updateRpms();
    return static_cast<int>(RPM);
}

float J1939Bus::getCurrentAMT()
{
    return static_cast<float>(pgn65270.data[2]) - 40.0f;
}

float J1939Bus::getCurrentBoostInPsi()
{
    const float kpa = static_cast<float>(pgn65270.data[1]) * 2.0f;
    const float psi = static_cast<float>(kpa) / 6.895f;
    return psi;
}

float J1939Bus::getCurrentBoostTemp()
{
    // Compute Boost Temperature
    auto boostTemp = static_cast<float>(pgn65129.data[0] << 8 | pgn65129.data[1]); // Raw
    boostTemp = boostTemp * 0.03125f; // Offset
    boostTemp = boostTemp - 273.0f; // Celsius
    boostTemp = ((boostTemp * 9) / 5) + 32; // Fahrenheit

    return boostTemp;
}

float J1939Bus::getCurrentEgtTemp()
{
    auto egt = static_cast<float>(pgn65270.data[5] * 255);
    egt += static_cast<float>(pgn65270.data[6]);
    egt *= 0.03125f;
    egt -= 273.0f;

    return egt;
}

int J1939Bus::getCurrentWaterTemp()
{
    waterTemp = pgn65262.data[0] - 40;
    return static_cast<int>(waterTemp);
}

byte J1939Bus::getCurrentOilPressure()
{
    // Compute Oil Pressure
    oilPressure = pgn65263.data[3] * 4 / 6.895; // Confirmed!!!
    return static_cast<byte>(oilPressure);
}

int J1939Bus::getCurrentFuelTemp()
{
    // Compute Fuel Temperature
    fuelTemp = (message274.data[7] << 8) | message274.data[6]; // Raw
    fuelTemp = fuelTemp / 16; // Kelvin
    fuelTemp = fuelTemp - 273.15; // Celsius
    fuelTemp = ((fuelTemp * 9) / 5) + 32; // Fahrenheit

    return static_cast<int>(fuelTemp);
}

char J1939Bus::getRequestedRange()
{
    return static_cast<char>(pgn61445.data[4]);
}

int8_t J1939Bus::getCurrentGear()
{
    return static_cast<int8_t>(pgn61445.data[3] - 125);
}

int8_t J1939Bus::getSelectedGear()
{
    return static_cast<int8_t>(pgn61445.data[0] - 125);
}

void J1939Bus::initialize(AppData* _appData)
{
    Serial.println("Cummins Bus initializing");

    appData = _appData;

    Can1.begin();
    Can1.setBaudRate(250 * 1000);
    Can1.setMaxMB(16);
    Can1.enableFIFO();
    Can1.enableFIFOInterrupt();
    Can1.onReceive(CumminsBusSniff);
    Can1.mailboxStatus();

    // pgn to request water temp pgn :)
    requestPgn(ENGINE_TEMP_1_PGN);
    requestPgn(DM1_DTCS_PGN);
}

void J1939Bus::loop()
{
    currentMillis = millis();

    if (currentMillis - lastJ1939Request >= 1000)
    {
        requestPgn(ENGINE_TEMP_1_PGN);
        requestPgn(65129);
        requestPgn(64870);
        requestPgn(DM1_DTCS_PGN);
        requestPgn(DM2_DTCS_PGN);

        lastJ1939Request = currentMillis;
    }
}

byte J1939Bus::getVehicleSpeed()
{
    // Compute Vehicle Speed
    const uint32_t speedRaw =
        ElectronicTransmissionController1Pgn.data[2] << 8 |
        ElectronicTransmissionController1Pgn.data[1];
    const auto outputShaftRpm = static_cast<float>(speedRaw) * .125f;
    const float wheelRpm = outputShaftRpm / 3.73f;
    const float inchesPerMinute = wheelRpm * 100.11f;
    const float milesPerHour = inchesPerMinute * 60 / 63360;

    speedAverage.addValue(static_cast<byte>(milesPerHour));

    return speedAverage.getAverage();
}