#include <Arduino.h>
#include "can_manager.h"
#include "esp32_can.h"
#include "config.h"
#include "SerialConsole.h"
#include "gvret_comm.h"
#include "lawicel.h"

CANManager::CANManager()
{

}

void CANManager::setup()
{
    busLoad[0].bitsSoFar = 0;
    busLoad[0].busloadPercentage = 0;
    busLoad[0].bitsPerQuarter = settings.CAN0Speed / 4;

    busLoadTimer = millis();
}

void CANManager::addBits(int offset, CAN_FRAME &frame)
{
    if (offset < 0) return;
    if (offset > 0) return;
    busLoad[offset].bitsSoFar += 41 + (frame.length * 9);
    if (frame.extended) busLoad[offset].bitsSoFar += 18;
}

void CANManager::sendFrame(CAN_COMMON *bus, CAN_FRAME &frame)
{
    int whichBus = 0;
    bus->sendFrame(frame);
    addBits(whichBus, frame);
}

void CANManager::outputFrame(CAN_FRAME &frame, int whichBus)
{
    uint8_t buff[40];
    uint8_t writtenBytes;
    uint8_t temp;
    uint32_t now = micros();

    if (SysSettings.lawicelMode) {
        if (SysSettings.lawicellExtendedMode) {
            Serial.print(micros());
            Serial.print(" - ");
            Serial.print(frame.id, HEX);            
            if (frame.extended) Serial.print(" X ");
            else Serial.print(" S ");
            console.printBusName(whichBus);
            for (int d = 0; d < frame.length; d++) {
                Serial.print(" ");
                Serial.print(frame.data.uint8[d], HEX);
            }
        }else {
            if (frame.extended) {
                Serial.print("T");
                sprintf((char *)buff, "%08x", frame.id);
                Serial.print((char *)buff);
            } else {
                Serial.print("t");
                sprintf((char *)buff, "%03x", frame.id);
                Serial.print((char *)buff);
            }
            Serial.print(frame.length);
            for (int i = 0; i < frame.length; i++) {
                sprintf((char *)buff, "%02x", frame.data.uint8[i]);
                Serial.print((char *)buff);
            }
            if (SysSettings.lawicelTimestamping) {
                uint16_t timestamp = (uint16_t)millis();
                sprintf((char *)buff, "%04x", timestamp);
                Serial.print((char *)buff);
            }
        }
        Serial.write(13);
    } else {
        if (SysSettings.isWifiActive) wifiGVRET.sendFrameToBuffer(frame, whichBus);
        else serialGVRET.sendFrameToBuffer(frame, whichBus);
    }
}

void CANManager::loop()
{
    CAN_FRAME incoming;
    size_t wifiLength = wifiGVRET.numAvailableBytes();
    size_t serialLength = serialGVRET.numAvailableBytes();
    size_t maxLength = (wifiLength>serialLength)?wifiLength:serialLength;

    if (millis() > (busLoadTimer + 250)) {
        busLoadTimer = millis();
        busLoad[0].busloadPercentage = ((busLoad[0].busloadPercentage * 3) + (((busLoad[0].bitsSoFar * 1000) / busLoad[0].bitsPerQuarter) / 10)) / 4;
        //Force busload percentage to be at least 1% if any traffic exists at all. This forces the LED to light up for any traffic.
        if (busLoad[0].busloadPercentage == 0 && busLoad[0].bitsSoFar > 0) busLoad[0].busloadPercentage = 1;
        busLoad[0].bitsPerQuarter = settings.CAN0Speed / 4;
        busLoad[0].bitsSoFar = 0;
        if(busLoad[0].busloadPercentage > busLoad[1].busloadPercentage){
            //updateBusloadLED(busLoad[0].busloadPercentage);
        } else{
            //updateBusloadLED(busLoad[1].busloadPercentage);
        }
    }

    while (CAN0.available() > 0 && (maxLength < (WIFI_BUFF_SIZE - 80))) {
        CAN0.read(incoming);
        addBits(0, incoming);
        toggleRXLED();
        outputFrame(incoming, 0);
    }
}
