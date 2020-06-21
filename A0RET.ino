/*
 A0RET.ino

 Created: June 1, 2020
 Author: Collin Kidder

Copyright (c) 2014-2020 Collin Kidder, Michael Neuweiler

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
This code is now all specialized for the A0 and its ESP32 WRover-B which has 4MB flash and
8MB of PSRAM. It also only has a single CAN port.
*/
#include "config.h"
#include <esp32_can.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Preferences.h>

#include "ELM327_Emulator.h"
#include "SerialConsole.h"

byte i = 0;

typedef struct {
    uint32_t bitsPerQuarter;
    uint32_t bitsSoFar;
    uint8_t busloadPercentage;
} BUSLOAD;

byte serialBuffer[WIFI_BUFF_SIZE];
int serialBufferLength = 0; //not creating a ring buffer. The buffer should be large enough to never overflow
uint32_t lastFlushMicros = 0;
uint32_t lastBroadcast = 0;
BUSLOAD busLoad[2];
uint32_t busLoadTimer;
bool markToggle[6];
uint32_t lastMarkTrigger = 0;

EEPROMSettings settings;
SystemSettings SysSettings;
Preferences nvPrefs;

ELM327Emu elmEmulator;

SerialConsole console;

WiFiMulti wifiMulti;
WiFiServer wifiServer(23); //Register as a telnet server
WiFiUDP wifiUDPServer;
IPAddress broadcastAddr(255,255,255,255);

//initializes all the system EEPROM values. Chances are this should be broken out a bit but
//there is only one checksum check for all of them so it's simple to do it all here.
void loadSettings()
{
    Logger::console("Loading settings....");

    boolean CAN0_Enabled;
    boolean useBinarySerialComm; //use a binary protocol on the serial link or human readable format?
    uint8_t logLevel; //Level of logging to output on serial line
    boolean CAN0ListenOnly; //if true we don't allow any messing with the bus but rather just passively monitor.
    uint8_t wifiMode; //0 = don't use wifi, 1 = connect to an AP, 2 = Create an AP
    uint8_t SSID[32];     //null terminated string for the SSID
    uint8_t WPA2Key[64]; //Null terminated string for the key. Can be a passphase or the actual key

    nvPrefs.begin(PREF_NAME, false);

    settings.CAN0Speed = nvPrefs.getUInt("can0speed", 500000);
    settings.CAN0_Enabled = nvPrefs.getBool("can0_en", true);
    settings.CAN0ListenOnly = nvPrefs.getBool("can0-listenonly", false);
    settings.useBinarySerialComm = nvPrefs.getBool("binarycomm", false);
    settings.logLevel = nvPrefs.getUChar("loglevel", 1); //info
    settings.wifiMode = nvPrefs.getUChar("wifiMode", 0); //Wifi defaults to being off
    if (nvPrefs.getString("SSID", settings.SSID, 32) == 0)
    {
        strcpy(settings.SSID, "ESP32DUE");
    }

    if (nvPrefs.getString("wpa2Key", settings.WPA2Key, 64) == 0)
    {
        strcpy(settings.WPA2Key, "aBigSecret");
    }

    nvPrefs.end();

    Logger::setLoglevel((Logger::LogLevel)settings.logLevel);

    Logger::console("Running on Macchina A0");
    SysSettings.LED_CANTX = 255;
    SysSettings.LED_CANRX = 13;
    SysSettings.LED_LOGGING = 255;
    SysSettings.logToggle = false;
    SysSettings.txToggle = true;
    SysSettings.rxToggle = true;
    SysSettings.lawicelAutoPoll = false;
    SysSettings.lawicelMode = false;
    SysSettings.lawicellExtendedMode = false;
    SysSettings.lawicelTimestamping = false;
    SysSettings.numBuses = 1; //Currently we support CAN0
    SysSettings.isWifiActive = false;
    SysSettings.isWifiConnected = false;
    for (int rx = 0; rx < NUM_BUSES; rx++) SysSettings.lawicelBusReception[rx] = true; //default to showing messages on RX 
    //set pin mode for all LEDS

    busLoad[0].bitsSoFar = 0;
    busLoad[0].busloadPercentage = 0;
    busLoad[0].bitsPerQuarter = settings.CAN0Speed / 4;

    busLoadTimer = millis();
}

void setup()
{
    //delay(5000); //just for testing. Don't use in production

    Serial.begin(1000000);

    SysSettings.isWifiConnected = false;

    loadSettings();

    if (settings.wifiMode == 1)
    {
        WiFi.mode(WIFI_STA);
        WiFi.begin((const char *)settings.SSID, (const char *)settings.WPA2Key);
    }
    if (settings.wifiMode == 2)
    {
        WiFi.mode(WIFI_AP);
        WiFi.softAP((const char *)settings.SSID, (const char *)settings.WPA2Key);
    }

    Serial.print("Build number: ");
    Serial.println(CFG_BUILD_NUM);

    if (settings.CAN0_Enabled) {
        CAN0.enable();
        CAN0.begin(settings.CAN0Speed, 255);
        Serial.print("Enabled CAN0 with speed ");
        Serial.println(settings.CAN0Speed);
        if (settings.CAN0ListenOnly) {
            CAN0.setListenOnlyMode(true);
        } else {
            CAN0.setListenOnlyMode(false);
        }
    } 
    else
    {
        CAN0.disable();
    }

    setPromiscuousMode();

    SysSettings.lawicelMode = false;
    SysSettings.lawicelAutoPoll = false;
    SysSettings.lawicelTimestamping = false;
    SysSettings.lawicelPollCounter = 0;
    
    //elmEmulator.setup();

    Serial.print("Done with init\n");
}

void setPromiscuousMode()
{
    CAN0.watchFor();
}

void toggleRXLED()
{
    static int counter = 0;
    counter++;
    if (counter >= BLINK_SLOWNESS) {
        counter = 0;
        SysSettings.rxToggle = !SysSettings.rxToggle;
        setLED(SysSettings.LED_CANRX, SysSettings.rxToggle);
    }
}

void toggleTXLED()
{
    static int counter = 0;
    counter++;
    if (counter >= BLINK_SLOWNESS) {
        counter = 0;
        SysSettings.txToggle = !SysSettings.txToggle;
        setLED(SysSettings.LED_CANTX, SysSettings.txToggle);
    }
}

//Get the value of XOR'ing all the bytes together. This creates a reasonable checksum that can be used
//to make sure nothing too stupid has happened on the comm.
uint8_t checksumCalc(uint8_t *buffer, int length)
{
    uint8_t valu = 0;
    for (int c = 0; c < length; c++) {
        valu ^= buffer[c];
    }
    return valu;
}

void addBits(int offset, CAN_FRAME &frame)
{
    if (offset < 0) return;
    if (offset > 1) return;
    busLoad[offset].bitsSoFar += 41 + (frame.length * 9);
    if (frame.extended) busLoad[offset].bitsSoFar += 18;
}

void sendFrame(CAN_COMMON *bus, CAN_FRAME &frame)
{
    int whichBus = 0;
    bus->sendFrame(frame);
    addBits(whichBus, frame);
}

void sendFrameToUSB(CAN_FRAME &frame, int whichBus)
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
        if (settings.useBinarySerialComm) {
            if (frame.extended) frame.id |= 1 << 31;
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 0; //0 = canbus frame sending
            serialBuffer[serialBufferLength++] = (uint8_t)(now & 0xFF);
            serialBuffer[serialBufferLength++] = (uint8_t)(now >> 8);
            serialBuffer[serialBufferLength++] = (uint8_t)(now >> 16);
            serialBuffer[serialBufferLength++] = (uint8_t)(now >> 24);
            serialBuffer[serialBufferLength++] = (uint8_t)(frame.id & 0xFF);
            serialBuffer[serialBufferLength++] = (uint8_t)(frame.id >> 8);
            serialBuffer[serialBufferLength++] = (uint8_t)(frame.id >> 16);
            serialBuffer[serialBufferLength++] = (uint8_t)(frame.id >> 24);
            serialBuffer[serialBufferLength++] = frame.length + (uint8_t)(whichBus << 4);
            for (int c = 0; c < frame.length; c++) {
                serialBuffer[serialBufferLength++] = frame.data.uint8[c];
            }
            //temp = checksumCalc(buff, 11 + frame.length);
            temp = 0;
            serialBuffer[serialBufferLength++] = temp;
            //Serial.write(buff, 12 + frame.length);
        } else {
            writtenBytes = sprintf((char *)&serialBuffer[serialBufferLength], "%d - %x", micros(), frame.id);
            serialBufferLength += writtenBytes;
            if (frame.extended) sprintf((char *)&serialBuffer[serialBufferLength], " X ");
            else sprintf((char *)&serialBuffer[serialBufferLength], " S ");
            serialBufferLength += 3;
            writtenBytes = sprintf((char *)&serialBuffer[serialBufferLength], "%i %i", whichBus, frame.length);
            serialBufferLength += writtenBytes;
            for (int c = 0; c < frame.length; c++) {
                writtenBytes = sprintf((char *)&serialBuffer[serialBufferLength], " %x", frame.data.uint8[c]);
                serialBufferLength += writtenBytes;
            }
            sprintf((char *)&serialBuffer[serialBufferLength], "\r\n");
            serialBufferLength += 2;
        }
    }
}


/*
Send a fake frame out USB and maybe to file to show where the mark was triggered at. The fake frame has bits 31 through 3
set which can never happen in reality since frames are either 11 or 29 bit IDs. So, this is a sign that it is a mark frame
and not a real frame. The bottom three bits specify which mark triggered.
*/
void sendMarkTriggered(int which)
{
    CAN_FRAME frame;
    frame.id = 0xFFFFFFF8ull + which;
    frame.extended = true;
    frame.length = 0;
    frame.rtr = 0;
    sendFrameToUSB(frame, 0);
}

void processIncomingByte(uint8_t in_byte)
{
    static CAN_FRAME build_out_frame;
    static int out_bus;
    static byte buff[20];
    static int step = 0;
    static STATE state = IDLE;
    static uint32_t build_int;
    uint32_t busSpeed = 0;
    uint32_t now = micros();

    uint8_t temp8;
    uint16_t temp16;

    switch (state) {
    case IDLE:
        if(in_byte == 0xF1){
            state = GET_COMMAND;
        }else if(in_byte == 0xE7){
            settings.useBinarySerialComm = true;
            SysSettings.lawicelMode = false;
            setPromiscuousMode(); //going into binary comm will set promisc. mode too.
        } else{
            console.rcvCharacter((uint8_t) in_byte);
        }
        break;
    case GET_COMMAND:
        switch(in_byte)
        {
        case PROTO_BUILD_CAN_FRAME:
            state = BUILD_CAN_FRAME;
            buff[0] = 0xF1;
            step = 0;
            break;
        case PROTO_TIME_SYNC:
            state = TIME_SYNC;
            step = 0;
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 1; //time sync
            serialBuffer[serialBufferLength++] = (uint8_t) (now & 0xFF);
            serialBuffer[serialBufferLength++] = (uint8_t) (now >> 8);
            serialBuffer[serialBufferLength++] = (uint8_t) (now >> 16);
            serialBuffer[serialBufferLength++] = (uint8_t) (now >> 24);
            break;
        case PROTO_DIG_INPUTS:
            //immediately return the data for digital inputs
            temp8 = 0; //getDigital(0) + (getDigital(1) << 1) + (getDigital(2) << 2) + (getDigital(3) << 3) + (getDigital(4) << 4) + (getDigital(5) << 5);
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 2; //digital inputs
            serialBuffer[serialBufferLength++] = temp8;
            temp8 = checksumCalc(buff, 2);
            serialBuffer[serialBufferLength++]  = temp8;
            state = IDLE;
            break;
        case PROTO_ANA_INPUTS:
            //immediately return data on analog inputs
            temp16 = 0;// getAnalog(0);  // Analogue input 1
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 3;
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp16 = 0;//getAnalog(1);  // Analogue input 2
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp16 = 0;//getAnalog(2);  // Analogue input 3
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp16 = 0;//getAnalog(3);  // Analogue input 4
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp16 = 0;//getAnalog(4);  // Analogue input 5
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp16 = 0;//getAnalog(5);  // Analogue input 6
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp16 = 0;//getAnalog(6);  // Vehicle Volts
            serialBuffer[serialBufferLength++] = temp16 & 0xFF;
            serialBuffer[serialBufferLength++] = uint8_t(temp16 >> 8);
            temp8 = checksumCalc(buff, 9);
            serialBuffer[serialBufferLength++] = temp8;
            state = IDLE;
            break;
        case PROTO_SET_DIG_OUT:
            state = SET_DIG_OUTPUTS;
            buff[0] = 0xF1;
            break;
        case PROTO_SETUP_CANBUS:
            state = SETUP_CANBUS;
            step = 0;
            buff[0] = 0xF1;
            break;
        case PROTO_GET_CANBUS_PARAMS:
            //immediately return data on canbus params
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 6;
            serialBuffer[serialBufferLength++] = settings.CAN0_Enabled + ((unsigned char) settings.CAN0ListenOnly << 4);
            serialBuffer[serialBufferLength++] = settings.CAN0Speed;
            serialBuffer[serialBufferLength++] = settings.CAN0Speed >> 8;
            serialBuffer[serialBufferLength++] = settings.CAN0Speed >> 16;
            serialBuffer[serialBufferLength++] = settings.CAN0Speed >> 24;
            serialBuffer[serialBufferLength++] = 0;
            serialBuffer[serialBufferLength++] = 0;
            serialBuffer[serialBufferLength++] = 0;
            serialBuffer[serialBufferLength++] = 0;
            serialBuffer[serialBufferLength++] = 0;
            state = IDLE;
            break;
        case PROTO_GET_DEV_INFO:
            //immediately return device information
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 7;
            serialBuffer[serialBufferLength++] = CFG_BUILD_NUM & 0xFF;
            serialBuffer[serialBufferLength++] = (CFG_BUILD_NUM >> 8);
            serialBuffer[serialBufferLength++] = EEPROM_VER;
            serialBuffer[serialBufferLength++] = 0;
            serialBuffer[serialBufferLength++] = 0;
            serialBuffer[serialBufferLength++] = 0; //was single wire mode. Should be rethought for this board.
            state = IDLE;
            break;
        case PROTO_SET_SW_MODE:
            buff[0] = 0xF1;
            state = SET_SINGLEWIRE_MODE;
            step = 0;
            break;
        case PROTO_KEEPALIVE:
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 0x09;
            serialBuffer[serialBufferLength++] = 0xDE;
            serialBuffer[serialBufferLength++] = 0xAD;
            state = IDLE;
            break;
        case PROTO_SET_SYSTYPE:
            buff[0] = 0xF1;
            state = SET_SYSTYPE;
            step = 0;
            break;
        case PROTO_ECHO_CAN_FRAME:
            state = ECHO_CAN_FRAME;
            buff[0] = 0xF1;
            step = 0;
            break;
        case PROTO_GET_NUMBUSES:
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 12;
            serialBuffer[serialBufferLength++] = 1; //just CAN0 on this hardware
            state = IDLE;
            break;
        case PROTO_GET_EXT_BUSES:
            serialBuffer[serialBufferLength++]  = 0xF1;
            serialBuffer[serialBufferLength++]  = 13;
            for (int u = 2; u < 17; u++) serialBuffer[serialBufferLength++] = 0;
            step = 0;
            state = IDLE;
            break;
        case PROTO_SET_EXT_BUSES:
            state = SETUP_EXT_BUSES;
            step = 0;
            buff[0] = 0xF1;
            break;
        }
        break;
    case BUILD_CAN_FRAME:
        buff[1 + step] = in_byte;
        switch(step)
        {
        case 0:
            build_out_frame.id = in_byte;
            break;
        case 1:
            build_out_frame.id |= in_byte << 8;
            break;
        case 2:
            build_out_frame.id |= in_byte << 16;
            break;
        case 3:
            build_out_frame.id |= in_byte << 24;
            if(build_out_frame.id & 1 << 31)
            {
                build_out_frame.id &= 0x7FFFFFFF;
                build_out_frame.extended = true;
            } else build_out_frame.extended = false;
            break;
        case 4:
            out_bus = in_byte & 3;
            break;
        case 5:
            build_out_frame.length = in_byte & 0xF;
            if(build_out_frame.length > 8) 
            {
                build_out_frame.length = 8;
            }
            break;
        default:
            if(step < build_out_frame.length + 6)
            {
                build_out_frame.data.uint8[step - 6] = in_byte;
            } 
            else
            {
                state = IDLE;
                //this would be the checksum byte. Compute and compare.
                temp8 = checksumCalc(buff, step);
                build_out_frame.rtr = 0;
                sendFrame(&CAN0, build_out_frame);
            }
            break;
        }
        step++;
        break;
        case TIME_SYNC:
            state = IDLE;
            break;
        case GET_DIG_INPUTS:
            // nothing to do
            break;
        case GET_ANALOG_INPUTS:
            // nothing to do
            break;
        case SET_DIG_OUTPUTS: //todo: validate the XOR byte
            buff[1] = in_byte;
            //temp8 = checksumCalc(buff, 2);
            for(int c = 0; c < 8; c++){
                if(in_byte & (1 << c)) setOutput(c, true);
                else setOutput(c, false);
            }
            state = IDLE;
            break;
        case SETUP_CANBUS: //todo: validate checksum
            switch(step)
            {
            case 0:
                build_int = in_byte;
                break;
            case 1:
                build_int |= in_byte << 8;
                break;
            case 2:
                build_int |= in_byte << 16;
                break;
            case 3:
                build_int |= in_byte << 24;
                busSpeed = build_int & 0xFFFFF;
                if(busSpeed > 1000000) busSpeed = 1000000;

                if(build_int > 0)
                {
                    if(build_int & 0x80000000ul) //signals that enabled and listen only status are also being passed
                    {
                        if(build_int & 0x40000000ul)
                        {
                            settings.CAN0_Enabled = true;
                        } else 
                        {
                            settings.CAN0_Enabled = false;
                        }
                        if(build_int & 0x20000000ul)
                        {
                            settings.CAN0ListenOnly = true;
                        } else 
                        {
                            settings.CAN0ListenOnly = false;
                        }
                    } else 
                    {
                        //if not using extended status mode then just default to enabling - this was old behavior
                        settings.CAN0_Enabled = true;
                    }
                    //CAN0.set_baudrate(build_int);
                    settings.CAN0Speed = busSpeed;
                } else { //disable first canbus
                    settings.CAN0_Enabled = false;
                }

                if (settings.CAN0_Enabled)
                {
                    CAN0.begin(settings.CAN0Speed, 255);
                    if (settings.CAN0ListenOnly) CAN0.setListenOnlyMode(true);
                    else CAN0.setListenOnlyMode(false);
                    CAN0.watchFor();
                }
                else CAN0.disable();
                break;
            case 4:
                build_int = in_byte;
                break;
            case 5:
                build_int |= in_byte << 8;
                break;
            case 6:
                build_int |= in_byte << 16;
                break;
            case 7:
                build_int |= in_byte << 24;
                busSpeed = build_int & 0xFFFFF;
                if(busSpeed > 1000000) busSpeed = 1000000;

                state = IDLE;
                //now, write out the new canbus settings to EEPROM
                //EEPROM.writeBytes(0, &settings, sizeof(settings));
                //EEPROM.commit();
                //setPromiscuousMode();
                break;
            }
            step++;
            break;
        case GET_CANBUS_PARAMS:
            // nothing to do
            break;
        case GET_DEVICE_INFO:
            // nothing to do
            break;
        case SET_SINGLEWIRE_MODE:
            if(in_byte == 0x10){
            } else {
            }
            //EEPROM.writeBytes(0, &settings, sizeof(settings));
            //EEPROM.commit();
            state = IDLE;
            break;
        case SET_SYSTYPE:
            //settings.sysType = in_byte;
            //EEPROM.writeBytes(0, &settings, sizeof(settings));
            //EEPROM.commit();
            //loadSettings();
            state = IDLE;
            break;
        case ECHO_CAN_FRAME:
            buff[1 + step] = in_byte;
            switch(step)
            {
            case 0:
                build_out_frame.id = in_byte;
                break;
            case 1:
                build_out_frame.id |= in_byte << 8;
                break;
            case 2:
                build_out_frame.id |= in_byte << 16;
                break;
            case 3:
                build_out_frame.id |= in_byte << 24;
                if(build_out_frame.id & 1 << 31) {
                    build_out_frame.id &= 0x7FFFFFFF;
                    build_out_frame.extended = true;
                } else build_out_frame.extended = false;
                break;
            case 4:
                out_bus = in_byte & 1;
                break;
            case 5:
                build_out_frame.length = in_byte & 0xF;
                if(build_out_frame.length > 8) build_out_frame.length = 8;
                break;
            default:
                if(step < build_out_frame.length + 6) 
                {
                    build_out_frame.data.bytes[step - 6] = in_byte;
                } 
                else 
                {
                    state = IDLE;
                    //this would be the checksum byte. Compute and compare.
                    temp8 = checksumCalc(buff, step);
                    //if (temp8 == in_byte)
                    //{
                    toggleRXLED();
                    //if(isConnected) {
                    sendFrameToUSB(build_out_frame, 0);
                    //}
                    //}
                }
                break;
            }
            step++;
            break;
        case SETUP_EXT_BUSES: //setup enable/listenonly/speed for SWCAN, Enable/Speed for LIN1, LIN2
            switch(step)
            {
            case 0:
                build_int = in_byte;
                break;
            case 1:
                build_int |= in_byte << 8;
                break;
            case 2:
                build_int |= in_byte << 16;
                break;
            case 3:
                build_int |= in_byte << 24;
                break;
            case 4:
                build_int = in_byte;
                break;
            case 5:
                build_int |= in_byte << 8;
                break;
            case 6:
                build_int |= in_byte << 16;
                break;
            case 7:
                build_int |= in_byte << 24;
                break;
            case 8:
                build_int = in_byte;
                break;
            case 9:
                build_int |= in_byte << 8;
                break;
            case 10:
                build_int |= in_byte << 16;
                break;
            case 11:
                build_int |= in_byte << 24;
                state = IDLE;
                //now, write out the new canbus settings to EEPROM
                //EEPROM.writeBytes(0, &settings, sizeof(settings));
                //EEPROM.commit();
                //setPromiscuousMode();
                break;
            }
        step++;
        break;
    }
}

/*
Loop executes as often as possible all the while interrupts fire in the background.
The serial comm protocol is as follows:
All commands start with 0xF1 this helps to synchronize if there were comm issues
Then the next byte specifies which command this is.
Then the command data bytes which are specific to the command
Lastly, there is a checksum byte just to be sure there are no missed or duped bytes
Any bytes between checksum and 0xF1 are thrown away

Yes, this should probably have been done more neatly but this way is likely to be the
fastest and safest with limited function calls
*/
void loop()
{
    CAN_FRAME incoming;
    //uint32_t temp32;    
    bool isConnected = false;
    int serialCnt;
    uint8_t in_byte;
    boolean needServerInit = false;    

    if (settings.wifiMode > 0)
    {
        if (!SysSettings.isWifiConnected)
        {
            if (WiFi.isConnected())
            {
                Serial.print("Wifi now connected to SSID ");
                Serial.println((const char *)settings.SSID);
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
                Serial.print("RSSI: ");
                Serial.println(WiFi.RSSI());
                needServerInit = true;
            }
            if (settings.wifiMode == 2)
            {
                Serial.print("Wifi setup as SSID ");
                Serial.println((const char *)settings.SSID);
                Serial.print("IP address: ");
                Serial.println(WiFi.softAPIP());
                needServerInit = true;
            }
            if (needServerInit)
            {
                SysSettings.isWifiConnected = true;
                wifiServer.begin();
                wifiServer.setNoDelay(true);                    
                ArduinoOTA.setPort(3232);
                ArduinoOTA.setHostname("A0RET");
                // No authentication by default
                //ArduinoOTA.setPassword("admin");
                
                ArduinoOTA
                   .onStart([]() {
                      String type;
                      if (ArduinoOTA.getCommand() == U_FLASH)
                         type = "sketch";
                      else // U_SPIFFS
                         type = "filesystem";

                      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                      Serial.println("Start updating " + type);
                   })
                   .onEnd([]() {
                      Serial.println("\nEnd");
                   })
                   .onProgress([](unsigned int progress, unsigned int total) {
                       Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
                   })
                   .onError([](ota_error_t error) {
                      Serial.printf("Error[%u]: ", error);
                      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                      else if (error == OTA_END_ERROR) Serial.println("End Failed");
                   });
                   
                ArduinoOTA.begin();
            }
        }
        else
        {
            if (WiFi.isConnected() || settings.wifiMode == 2)
            {
                if (wifiServer.hasClient())
                {
                    for(i = 0; i < MAX_CLIENTS; i++)
                    {
                        if (!SysSettings.clientNodes[i] || !SysSettings.clientNodes[i].connected())
                        {
                            if (SysSettings.clientNodes[i]) SysSettings.clientNodes[i].stop();
                            SysSettings.clientNodes[i] = wifiServer.available();
                            if (!SysSettings.clientNodes[i]) Serial.println("Couldn't accept client connection!");
                            else 
                            {
                                Serial.print("New client: ");
                                Serial.print(i); Serial.print(' ');
                                Serial.println(SysSettings.clientNodes[i].remoteIP());
                            }
                        }
                    }
                    if (i >= MAX_CLIENTS) {
                        //no free/disconnected spot so reject
                        wifiServer.available().stop();
                    }
                }

                //check clients for data
                for(i = 0; i < MAX_CLIENTS; i++){
                    if (SysSettings.clientNodes[i] && SysSettings.clientNodes[i].connected())
                    {
                        if(SysSettings.clientNodes[i].available())
                        {
                            //get data from the telnet client and push it to input processing
                            while(SysSettings.clientNodes[i].available()) 
                            {
                                uint8_t inByt;
                                inByt = SysSettings.clientNodes[i].read();
                                SysSettings.isWifiActive = true;
                                //Serial.write(inByt); //echo to serial - just for debugging. Don't leave this on!
                                processIncomingByte(inByt);
                            }
                        }
                    }
                    else {
                        if (SysSettings.clientNodes[i]) {
                            SysSettings.clientNodes[i].stop();
                        }
                    }
                }                    
            }
            else 
            {
                if (settings.wifiMode == 1)
                {
                    Serial.println("WiFi disconnected. Bummer!");
                    SysSettings.isWifiConnected = false;
                    SysSettings.isWifiActive = false;
                }
            }
        }
    }
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

    /*if (Serial)*/ isConnected = true;

    //if (!SysSettings.lawicelMode || SysSettings.lawicelAutoPoll || SysSettings.lawicelPollCounter > 0)
    //{
    while (CAN0.available() > 0 && (serialBufferLength < (WIFI_BUFF_SIZE - 80))) {
        CAN0.read(incoming);
        addBits(0, incoming);
        toggleRXLED();
        if (isConnected) sendFrameToUSB(incoming, 0);
    }
  
    if (SysSettings.lawicelPollCounter > 0) SysSettings.lawicelPollCounter--;
    //}

    if (SysSettings.isWifiConnected && micros() - lastBroadcast > 1000000ul) //every second send out a broadcast ping
    {
        uint8_t buff[4] = {0x1C,0xEF,0xAC,0xED};
        lastBroadcast = micros();
        wifiUDPServer.beginPacket(broadcastAddr, 17222);
        wifiUDPServer.write(buff, 4);
        wifiUDPServer.endPacket();
    }

    //If the max time has passed or the buffer is almost filled then send buffered data out
    if ((micros() - lastFlushMicros > SER_BUFF_FLUSH_INTERVAL) || (serialBufferLength > (WIFI_BUFF_SIZE - 40)) ) {
        if (serialBufferLength > 0) {
            if (settings.wifiMode == 0 || !SysSettings.isWifiActive) Serial.write(serialBuffer, serialBufferLength);
            else
            {
                for(i = 0; i < MAX_CLIENTS; i++)
                {
                    if (SysSettings.clientNodes[i] && SysSettings.clientNodes[i].connected())
                    {
                        SysSettings.clientNodes[i].write(serialBuffer, serialBufferLength);
                    }
                }
            }
            serialBufferLength = 0;
            lastFlushMicros = micros();
        }
    }

    serialCnt = 0;
    while ( (Serial.available() > 0) && serialCnt < 128 ) {
        serialCnt++;
        in_byte = Serial.read();
        SysSettings.isWifiActive = false;
        processIncomingByte(in_byte);
    }

    //elmEmulator.loop();
    ArduinoOTA.handle();
}
