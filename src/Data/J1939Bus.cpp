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

// Initialize CAN message containers
volatile CanMessage J1939Bus::pgn65262 = {0x67, 0x8, {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, 0};
volatile CanMessage J1939Bus::pgn65262_149{0x00, 0x8, {}, 0};
volatile CanMessage J1939Bus::pgn65263{0x00, 0x8, {}, 0};
volatile CanMessage J1939Bus::pgn65263_149{0x00, 0x8, {}, 0};
volatile CanMessage J1939Bus::pgn65272{0x00, 0x8, {}, 0};
volatile CanMessage J1939Bus::pgn65129{0x00, 0x8, {}, 0};
volatile CanMessage J1939Bus::pgn61442{0x00, 0x8, {}, 0};
volatile CanMessage J1939Bus::pgn61445{0x00, 0x8, {}, 0};
volatile CanMessage J1939Bus::message256{0x00, 0x8, {}, 0};
volatile CanMessage J1939Bus::message274{0x00, 0x8, {}, 0};
volatile CanMessage J1939Bus::pgn61443 = {0x67, 0x8, {0xF1, 0x0, 0x0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 0};
volatile CanMessage J1939Bus::pgn65270{0x00, 0x8, {}, 0};
CAN_message_t J1939Bus::msg;

// Initialize sensor data
volatile unsigned long J1939Bus::RPM = 0;
volatile double J1939Bus::fuelTemp = 0;
volatile double J1939Bus::oilPressure = 0;
volatile double J1939Bus::waterTemp = 0;
volatile byte J1939Bus::load = 0;
volatile byte J1939Bus::throttlePercentage = 0;
volatile float J1939Bus::FuelPercentage = 0;

// Initialize fuel pressure tracking
volatile float J1939Bus::minFuelPressure = 99999999.0;
volatile float J1939Bus::maxFuelPressure = 0.0;

// Initialize ID tracking
volatile int J1939Bus::ids[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile int J1939Bus::idsP = 0;

// Initialize speed calculation
SeaDash::Uint32::IncrementalCircularAverage J1939Bus::speedAverage(10);

// Initialize the DTC buffer variables
DTCMessage J1939Bus::dtcBuffer[DTC_BUFFER_SIZE] = {};
volatile size_t J1939Bus::dtcBufferHead = 0;
volatile size_t J1939Bus::dtcBufferTail = 0;
volatile bool J1939Bus::dtcBufferFull = false;

// Initialize the CAN buffer variables
CAN_message_t J1939Bus::canBuffer[CAN_BUFFER_SIZE] = {};
volatile uint8_t J1939Bus::canBufferHead = 0;
volatile uint8_t J1939Bus::canBufferTail = 0;
volatile bool J1939Bus::canBufferFull = false;

// Add a method to safely add a message to the buffer
bool J1939Bus::addToDTCBuffer(const J1939Message& message)
{
    if (dtcBufferFull)
    {
        return false; // Buffer is full, can't add
    }

    // Copy message data to the buffer
    dtcBuffer[dtcBufferHead].canId = message.canId;
    dtcBuffer[dtcBufferHead].sourceAddress = message.sourceAddress;
    for (int i = 0; i < 8; i++)
    {
        dtcBuffer[dtcBufferHead].data[i] = message.data[i];
    }
    dtcBuffer[dtcBufferHead].valid = true;

    // Move head pointer
    size_t nextHead = (dtcBufferHead + 1) % DTC_BUFFER_SIZE;
    dtcBufferHead = nextHead;

    // Check if buffer is full
    dtcBufferFull = (dtcBufferHead == dtcBufferTail);

    return true;
}

// Add a method to safely get a message from the buffer
bool J1939Bus::getFromDTCBuffer(DTCMessage& message)
{
    if (dtcBufferHead == dtcBufferTail && !dtcBufferFull)
    {
        return false; // Buffer is empty
    }

    // Copy data from buffer to output message
    message = dtcBuffer[dtcBufferTail];
    dtcBuffer[dtcBufferTail].valid = false; // Mark as read

    // Move tail pointer
    dtcBufferTail = (dtcBufferTail + 1) % DTC_BUFFER_SIZE;

    // Buffer is no longer full
    dtcBufferFull = false;

    return true;
}

// Add a method to process DTC messages in the main loop
void J1939Bus::processDTCMessages()
{
    DTCMessage message;

    // Process up to 5 messages per loop iteration to avoid blocking too long
    int processCount = 0;
    while (getFromDTCBuffer(message) && processCount < 5)
    {
        if (message.valid)
        {
            // Process the DTC message - this is the code moved from the interrupt handler
            const uint8_t mil = SeaDash::Bits::getNBits(message.data[0], 6, 2);
            const uint8_t rsl = SeaDash::Bits::getNBits(message.data[0], 4, 2);
            const uint8_t awl = SeaDash::Bits::getNBits(message.data[0], 2, 2);
            const uint8_t pls = SeaDash::Bits::getNBits(message.data[0], 0, 2);
            const uint8_t milBlink = SeaDash::Bits::getNBits(message.data[1], 6, 2);
            const uint8_t rslBlink = SeaDash::Bits::getNBits(message.data[1], 4, 2);
            const uint8_t awlBlink = SeaDash::Bits::getNBits(message.data[1], 2, 2);
            const uint8_t plsBlink = SeaDash::Bits::getNBits(message.data[1], 0, 2);
            uint32_t spn = message.data[2];
            spn = SeaDash::Bits::setNBitsAt<uint32_t>(spn, message.data[3], 8, 8);
            const auto spnLeastSignificantBits =
                SeaDash::Bits::getNBits<uint8_t>(message.data[4], 5, 3);
            spn = SeaDash::Bits::setNBitsAt<uint32_t>(spn, spnLeastSignificantBits, 16, 3);
            const uint8_t fmi = SeaDash::Bits::getNBits(message.data[4], 0, 5);
            const uint8_t oc = SeaDash::Bits::getNBits(message.data[5], 0, 7);

            // Print DTC information
            Serial.println(
                "DM1 DTC: ID: " + static_cast<String>(message.canId) +
                " SA: " + static_cast<String>(message.sourceAddress) + " SPN: " + static_cast<String>(spn) +
                " Failure Mode Indicator: " + static_cast<String>(fmi) +
                " Occurrence Count: " + static_cast<String>(oc) + " " + " MIL " + static_cast<String>(mil) +
                " " + " RSL " + static_cast<String>(rsl) + " " + " AWL " + static_cast<String>(awl) + " " +
                " PLS " + static_cast<String>(pls) + " MIL Blink " + static_cast<String>(milBlink) + " " +
                " RSL Blink" + static_cast<String>(rslBlink) + " " + " AWL Blink" +
                static_cast<String>(awlBlink) + " " + " PLS Blink" + static_cast<String>(plsBlink) +
                " Data: " + static_cast<String>(message.data[0]) + " " + static_cast<String>(message.data[1]) +
                " " + static_cast<String>(message.data[2]) + " " + static_cast<String>(message.data[3]) + " " +
                static_cast<String>(message.data[4]) + " " + static_cast<String>(message.data[5]) + " " +
                static_cast<String>(message.data[6]) + " " + static_cast<String>(message.data[7]));

            processCount++;
        }
    }
}

// Add messages to the CAN buffer
bool J1939Bus::addToCANBuffer(const CAN_message_t& msg)
{
    if (canBufferFull)
    {
        Serial.println("Buffer full");
        return false; // Buffer is full, can't add
    }

    // Copy message data to the buffer
    canBuffer[canBufferHead] = msg;

    // Move head pointer
    const uint8_t nextHead = (canBufferHead + 1) % CAN_BUFFER_SIZE;
    canBufferHead = nextHead;

    // Check if buffer is full
    canBufferFull = (canBufferHead == canBufferTail);

    return true;
}

// Get messages from the CAN buffer
bool J1939Bus::getFromCANBuffer(CAN_message_t& message)
{
    if (canBufferHead == canBufferTail && !canBufferFull)
    {
        return false; // Buffer is empty
    }

    // Copy data from buffer to output message
    message = canBuffer[canBufferTail];

    // Move tail pointer
    canBufferTail = (canBufferTail + 1) % CAN_BUFFER_SIZE;

    // Buffer is no longer full
    canBufferFull = false;

    return true;
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
    _messageToUpdate->lastMessageReceived = millis();
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
    // Just add the message to the buffer and return
    // This keeps the interrupt handler extremely lightweight
    J1939Bus::addToCANBuffer(_msg);
}

float J1939Bus::getCurrentFuelPressurePsi()
{
    if (pgn65263_149.count == 0)
    {
        return 0;
    }
    if (millis() - pgn65263_149.lastMessageReceived > 1000)
    {
        return 0;
    }
    auto fuelPressure = static_cast<float>(pgn65263_149.data[0] * 4);
    fuelPressure /= 6.895f;
    if (fuelPressure > maxFuelPressure) maxFuelPressure = fuelPressure;
    if (fuelPressure < minFuelPressure) minFuelPressure = fuelPressure;
    return fuelPressure;
}

byte J1939Bus::getTransmissionTempC()
{
    if (pgn65272.count == 0)
    {
        return 0;
    }
    if (millis() - pgn65272.lastMessageReceived > 1000)
    {
        return 0;
    }
    const uint16_t tempRaw = SeaDash::Bytes::combine2Bytes(pgn65272.data[5], pgn65272.data[4]);
    const float transmissionTemp = static_cast<float>(tempRaw) * 0.03125f - 273.15f;
    return static_cast<byte>(transmissionTemp);
}

float J1939Bus::getCurrentFuelPercentage()
{
    if (message256.count == 0)
    {
        return 0;
    }
    if (millis() - message256.lastMessageReceived > 1000)
    {
        return 0;
    }
    // Fuel compute
    FuelPercentage = static_cast<float>(SeaDash::Bytes::combine2Bytes(message256.data[1], message256.data[0]));
    FuelPercentage = FuelPercentage * 100.0f / 4096.0f; // 4095 is max fuel allowed by pump
    return FuelPercentage;
}

void J1939Bus::updateThrottlePercentage()
{
    if (pgn61443.count == 0)
    {
        throttlePercentage = 0;
        return;
    }
    if (millis() - pgn61443.lastMessageReceived > 1000)
    {
        throttlePercentage = 0;
        return;
    }
    throttlePercentage = static_cast<byte>(static_cast<float>(pgn61443.data[1]) * .4f);
}

int J1939Bus::getCurrentThrottlePercentage()
{
    updateThrottlePercentage();
    return throttlePercentage;
}

void J1939Bus::updateLoad()
{
    if (pgn61443.count == 0)
    {
        load = 0;
        return;
    }
    if (millis() - pgn61443.lastMessageReceived > 1000)
    {
        load = 0;
        return;
    }
    load = static_cast<byte>(static_cast<float>(pgn61443.data[2]) * 0.8f);
}

int J1939Bus::getCurrentLoad()
{
    updateLoad();
    return load;
}

void J1939Bus::updateRpms()
{
    if (message256.count == 0)
    {
        RPM = 0;
        return;
    }
    if (millis() - message256.lastMessageReceived > 1000)
    {
        RPM = 0;
        return;
    }
    RPM = SeaDash::Bytes::combine2Bytes(message256.data[7], message256.data[6]); // convert from little endian
    RPM /= 4;
}

int J1939Bus::getCurrentRpms()
{
    updateRpms();
    return static_cast<int>(RPM);
}

float J1939Bus::getCurrentBoostInPsi()
{
    if (pgn65270.count == 0)
    {
        return 0;
    }
    if (millis() - pgn65270.lastMessageReceived > 1000)
    {
        return 0;
    }
    const float kpa = static_cast<float>(pgn65270.data[1]) * 2.0f;
    const float psi = static_cast<float>(kpa) / 6.895f;
    return psi;
}

float J1939Bus::getCurrentBoostTemp()
{
    if (pgn65129.count == 0)
    {
        return 0;
    }
    if (millis() - pgn65129.lastMessageReceived > 1000)
    {
        return 0;
    }
    // Compute Boost Temperature
    auto boostTemp = static_cast<float>(SeaDash::Bytes::combine2Bytes(pgn65129.data[0], pgn65129.data[1])); // Raw
    boostTemp = boostTemp * 0.03125f; // Offset
    boostTemp = boostTemp - 273.0f; // Celsius
    boostTemp = boostTemp * 9.0f / 5.0f + 32.0f; // Fahrenheit

    return boostTemp;
}

float J1939Bus::getCurrentEgtTemp()
{
    if (pgn65270.count == 0)
    {
        return 0;
    }
    if (millis() - pgn65270.lastMessageReceived > 1000)
    {
        return 0;
    }
    auto egt = static_cast<float>(pgn65270.data[5] * 255);
    egt += static_cast<float>(pgn65270.data[6]);
    egt *= 0.03125f;
    egt -= 273.0f;

    return egt;
}

int J1939Bus::getCurrentWaterTemp()
{
    if (pgn65262.count == 0)
    {
        return 0;
    }
    if (millis() - pgn65262.lastMessageReceived > 1000)
    {
        return 0;
    }
    waterTemp = pgn65262.data[0] - 40;
    return static_cast<int>(waterTemp);
}

byte J1939Bus::getCurrentOilPressure()
{
    if (pgn65263.count == 0)
    {
        return 0;
    }
    if (millis() - pgn65263.lastMessageReceived > 1000)
    {
        return 0;
    }
    // Compute Oil Pressure
    oilPressure = pgn65263.data[3] * 4 / 6.895; // Confirmed!!!
    return static_cast<byte>(oilPressure);
}

int J1939Bus::getCurrentFuelTemp()
{
    if (message274.count == 0)
    {
        return 0;
    }
    if (millis() - message274.lastMessageReceived > 1000)
    {
        return 0;
    }
    // Compute Fuel Temperature
    fuelTemp = SeaDash::Bytes::combine2Bytes(message274.data[7], message274.data[6]); // Raw
    fuelTemp = fuelTemp / 16; // Kelvin
    fuelTemp = fuelTemp - 273.15; // Celsius
    fuelTemp = ((fuelTemp * 9) / 5) + 32; // Fahrenheit

    return static_cast<int>(fuelTemp);
}

char J1939Bus::getRequestedRange()
{
    if (pgn61445.count > 0 && millis() - pgn61445.lastMessageReceived < 1000)
    {
        return static_cast<char>(pgn61445.data[4]);
    }

    return 'P';
}

int8_t J1939Bus::getCurrentGear()
{
    if (pgn61445.count > 0 && millis() - pgn61445.lastMessageReceived < 1000)
    {
        return static_cast<int8_t>(pgn61445.data[3] - 125);
    }

    return 0;
}

int8_t J1939Bus::getSelectedGear()
{
    if (pgn61445.count > 0 && millis() - pgn61445.lastMessageReceived < 1000)
    {
        return static_cast<int8_t>(pgn61445.data[0] - 125);
    }
    return 0;
}

void J1939Bus::processCANMessages()
{
    CAN_message_t message;

    // Process up to 10 messages per loop iteration to avoid blocking too long
    int processCount = 0;
    while (getFromCANBuffer(message) && processCount < 10)
    {
        // Convert to J1939Message for processing
        J1939Message j1939Msg;
        j1939Msg.setCanId(message.id);
        j1939Msg.setData(message.buf);

        // Process based on ID first
        if (message.id == 256)
        {
            updateMessage(&message256, message);
        }
        else if (message.id == 274)
        {
            updateMessage(&message274, message);
        }
        else if (message.id == 1280 || message.id == 1298)
        {
        }
        // Process based on PGN second
        else
        {
            switch (j1939Msg.pgn)
            {
            case 61443:
                updateMessage(&pgn61443, message);
                break;

            case 65129:
                updateMessage(&pgn65129, message);
                break;

            case ENGINE_TEMP_1_PGN:
                if (j1939Msg.sourceAddress == 149)
                {
                    updateMessage(&pgn65262_149, message);
                }
                else
                {
                    updateMessage(&pgn65262, message);
                }
                break;

            case 65263:
                // PGN: 65263
                if (j1939Msg.sourceAddress == 149)
                {
                    updateMessage(&pgn65263_149, message);
                }
                else
                {
                    updateMessage(&pgn65263, message);
                }
                break;
            case 65270:
                if (j1939Msg.sourceAddress == 149)
                {
                    updateMessage(&pgn65270, message);
                }
                break;

            case 65272:
                // Transmission Fluids
                updateMessage(&pgn65272, message);
                break;

            case 61440:
            case 61452:
            case 65264:
            case 65098:
            case 65504:
            case 65265:
            case 65247:
            case 60415:
            case 65252:
            case 65269:
            case 65271:
            case 65099:
            case 59443:
            case 65266:
            case 60671:
            case 61444:
                break;

            case 61445:
                // Serial.println("Got 61445");
                updateMessage(&pgn61445, message);
                break;
            case 61442:
                updateMessage(&pgn61442, message);
                break;

            case DM1_DTCS_PGN:
                // Add to DTC buffer for detailed processing
                addToDTCBuffer(j1939Msg);
                break;

            default:
                Serial.print("Unknown PGN: ");
                Serial.print(j1939Msg.pgn);
                Serial.print(" ID: ");
                Serial.println(j1939Msg.canId);
            // Add to ID tracking
                if (idsP < 8)
                {
                    ids[idsP] = static_cast<int>(message.id);
                }
                idsP = (idsP + 1) % 8;
                break;
            }


            processCount++;
        }
    }
}

bool J1939Bus::initialize(AppData* _appData)
{
    Serial.println("Cummins Bus initializing");

    appData = _appData;

    Can1.begin();
    Can1.setBaudRate(250 * 1000);
    Can1.setMaxMB(16);
    Can1.enableFIFO();
    Can1.enableFIFOInterrupt();
    Can1.onReceive(CumminsBusSniff);

    // Give the CAN controller a moment to initialize
    const unsigned long startTime = millis();
    bool canInitialized = false;

    // Try sending a dummy message to see if CAN is ready
    while (!canInitialized && (millis() - startTime < 2000))
    {
        // 2 second timeout
        // Set up a test message
        CAN_message_t testMsg;
        testMsg.flags.extended = true;
        testMsg.id = 0x18EAFFF9; // A standard J1939 address that should be safe
        testMsg.len = 8;
        for (unsigned char& i : testMsg.buf) i = 0;

        // Try to send it
        if (Can1.write(testMsg))
        {
            canInitialized = true;
        }
        else
        {
            delay(100); // Wait a bit before trying again
        }
    }

    if (!canInitialized)
    {
        Serial.println("CAN initialization timeout");
        return false;
    }

    Can1.mailboxStatus();

    // pgn to request water temp pgn :)
    if (!requestPgnWithTimeout(ENGINE_TEMP_1_PGN, 500))
    {
        Serial.println("Warning: Initial ENGINE_TEMP_1_PGN request failed");
    }

    if (!requestPgnWithTimeout(DM1_DTCS_PGN, 500))
    {
        Serial.println("Warning: Initial DM1_DTCS_PGN request failed");
    }

    return true;
}

// Add a timeout version of requestPgn
bool J1939Bus::requestPgnWithTimeout(const uint32_t pgn, const unsigned long timeoutMs)
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

    const unsigned long startTime = millis();
    while (!Can1.write(msg))
    {
        if (millis() - startTime > timeoutMs)
        {
            return false; // Timeout occurred
        }
        delay(1);
    }

    return true;
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

    // Process general CAN messages first
    processCANMessages();

    // Process any DTC messages in the buffer
    processDTCMessages();
}

byte J1939Bus::getVehicleSpeed()
{
    if (pgn61442.count == 0)
    {
        return 0;
    }
    if (millis() - pgn61442.lastMessageReceived > 1000)
    {
        return 0;
    }
    // Compute Vehicle Speed
    const uint32_t speedRaw = SeaDash::Bytes::combine2Bytes(pgn61442.data[2], pgn61442.data[1]);
    const auto outputShaftRpm = static_cast<float>(speedRaw) * .125f;
    const float wheelRpm = outputShaftRpm / 3.73f;
    const float inchesPerMinute = wheelRpm * 100.11f;
    const float milesPerHour = inchesPerMinute * 60.0f / 63360.0f;

    speedAverage.addValue(static_cast<byte>(milesPerHour));

    return speedAverage.getAverage();
}
