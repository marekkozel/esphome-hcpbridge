// Credits to https://github.com/Gifford47/HCPBridgeMqtt for the initial code base

#ifndef HOERMANN_H_
#define HOERMANN_H_
#define MODBUSRTU_DEBUG 1

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "ModbusRTU.h"

#define SLAVE_ID 2
#define SIMULATEKEYPRESSDELAYMS 100
#define DEADREPORTTIMEOUT 60000

#define UART_PORT UART_NUM_2
#ifdef CONFIG_IDF_TARGET_ESP32S3
#define PIN_TXD 17
#define PIN_RXD 18
#else
#define PIN_TXD 17 // UART 2 TXD - GPIO17
#define PIN_RXD 16 // UART 2 RXD - GPIO16
#endif

// Workaround as my Supramatic did not Report the Status 0x0A when it's in vent Position
// When the door is at position 0x08 and not moving, Status gets changed to Venting.
#define VENT_POS 0x08

static const char *TAG_HCI = "HCI-BUS";

class HoermannCommand {
public:
    static const HoermannCommand STARTOPENDOOR;
    static const HoermannCommand STARTCLOSEDOOR;
    static const HoermannCommand STARTSTOPDOOR;
    static const HoermannCommand STARTOPENDOORHALF;
    static const HoermannCommand STARTVENTPOSITION;
    static const HoermannCommand STARTTOGGLELAMP;
    static const HoermannCommand WAITING;

    uint16_t commandRegPlus2Value;
    uint16_t commandEndPlus2Value;
    uint16_t commandRegPlus3Value;
    uint16_t commandEndPlus3Value;

private:
    HoermannCommand(
        uint16_t commandRegPlus2Value,
        uint16_t commandEndPlus2Value,
        uint16_t commandRegPlus3Value,
        uint16_t commandEndPlus3Value)
        : commandRegPlus2Value(commandRegPlus2Value),
          commandEndPlus2Value(commandEndPlus2Value),
          commandRegPlus3Value(commandRegPlus3Value),
          commandEndPlus3Value(commandEndPlus3Value) {}
};

class HoermannState {
public:
    enum State {
        OPEN,
        OPENING,
        CLOSED,
        CLOSING,
        HALFOPEN,
        MOVE_VENTING,
        VENT,
        MOVE_HALF,
        STOPPED
    };

    float targetPosition = 0;
    float currentPosition = 0;
    bool lightOn = false;
    State state = CLOSED;
    std::string debugMessage = "initial";
    unsigned long lastModbusResponse = 0;
    bool changed = false;
    bool debMessage = false;
    float gotoPosition = 0.0f;
    bool valid = false;

    void setTargetPosition(float targetPosition);
    void setGotoPosition(float setPosition);
    void setCurrentPosition(float currentPosition);
    void setLightOn(bool lightOn);
    void recordModbusResponse();
    void clearChanged();
    void clearDebug();
    long responseAge();
    void setState(State state);
    void setValid(bool isValid);
};

class HoermannGarageEngine {
public:
    HoermannState *state = new HoermannState();

    static HoermannGarageEngine &getInstance();

    void setup(int8_t rx, int8_t tx);
    void handleModbus();
    Modbus::ResultCode onRequest(Modbus::FunctionCode fc, const Modbus::RequestData data);
    void setCommandValuesToRead();
    uint16_t onDoorPositionChanged(TRegister *reg, uint16_t val);
    uint16_t onCurrentStateChanged(TRegister *reg, uint16_t val);
    uint16_t onLampState(TRegister *reg, uint16_t val);

    uint16_t onCounterWrite(TRegister *reg, uint16_t val);
    void setCommand(bool cond, const HoermannCommand *command);

    void stopDoor();
    void closeDoor();
    void openDoor();
    void toggleDoor();
    void halfPositionDoor();
    void ventilationPositionDoor();
    void turnLight(bool on);
    void toggleLight();
    void setPosition(int setPosition);

private:
    HoermannGarageEngine(){};
    ModbusRTU mb;                                 
    const HoermannCommand *nextCommand = nullptr; 
    unsigned long commandWrittenOn = 0;           
};

#endif
