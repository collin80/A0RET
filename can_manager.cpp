#include <Arduino.h>
#include "can_manager.h"
#include "esp32_can.h"
#include "config.h"
#include "SerialConsole.h"
#include "gvret_comm.h"
#include "lawicel.h"
#include "ELM327_Emulator.h"

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

void CANManager::displayFrame(CAN_FRAME &frame, int whichBus)
{
    if (settings.enableLawicel && SysSettings.lawicelMode) 
    {
        lawicel.sendFrameToBuffer(frame, whichBus);
    } 
    else 
    {
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
        displayFrame(incoming, 0);
        if (incoming.id > 0x7DF && incoming.id < 0x7F0) elmEmulator.processCANReply(incoming);
        wifiLength = wifiGVRET.numAvailableBytes();
        serialLength = serialGVRET.numAvailableBytes();
        maxLength = (wifiLength > serialLength) ? wifiLength:serialLength;
    }
}
