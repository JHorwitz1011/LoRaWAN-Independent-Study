/*
Module:  Environment-Sensor.ino
Function:
    Connect a humidity sensor to the Helium Network.
Copyright notice and License:
    See LICENSE file accompanying this project.
Author:
    Jim Horwitz, December 2021
Taken from an example by:
    Terry Moore, MCCI Corporation  May 2021
Notes:
*/

//LoRaWAN Imports
#include <Arduino_LoRaWAN_network.h>
#include <Arduino_LoRaWAN_EventLog.h>
#include <arduino_lmic.h>

//Sensor Imports
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

//Package Formatting Imports
#include <CayenneLPP.h>

//compiler flags
#define ARDUINO_LMIC_CFG_NETWORK_HELIUM 1
#define ARDUINO_LMIC_CFG_SUBBAND -1

// Debug level doesnt seem to do anything. Worth playing around with in a future edition, though, seems useful.
//#define LMIC_DEBUG_LEVEL 2
//#define LMIC_PRINTF_TO Print 

//BME sensor constants
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

/****************************************************************************\
|
| The LoRaWAN object
|
\****************************************************************************/

class cMyLoRaWAN : public Arduino_LoRaWAN_network {
public:
    cMyLoRaWAN() {};
    using Super = Arduino_LoRaWAN_network;
    void setup();

protected:
    // you'll need to provide implementation for this.
    virtual bool GetOtaaProvisioningInfo(Arduino_LoRaWAN::OtaaProvisioningInfo*) override;
    
    // if you have persistent storage, you can provide implementations for these:
    virtual void NetSaveSessionInfo(const SessionInfo &Info, const uint8_t *pExtraInfo, size_t nExtraInfo) override;
    virtual void NetSaveSessionState(const SessionState &State) override;
    virtual bool NetGetSessionState(SessionState &State) override;
};


/****************************************************************************\
|
| The sensor object
|
\****************************************************************************/

class cSensor {
public:
    /// \brief the constructor. Deliberately does very little.
    cSensor() {};

    ///
    /// \brief set up the sensor object
    ///
    /// \param uplinkPeriodMs optional uplink interval. If not specified,
    ///         transmit every six minutes.
    ///
    void setup(std::uint32_t uplinkPeriodMs = 6 * 60 * 1000);

    ///
    /// \brief update sensor loop.
    ///
    /// \details
    ///     This should be called from the global loop(); it periodically
    ///     gathers and transmits sensor data.
    ///
    void loop();

private:
    void doUplink();

    bool m_fUplinkRequest;              // set true when uplink is requested
    bool m_fBusy;                       // set true while sending an uplink
    std::uint32_t m_uplinkPeriodMs;     // uplink period in milliseconds
    std::uint32_t m_tReference;         // time of last uplink

    Adafruit_BME680 bme;                // sensor object
};

/****************************************************************************\
|
| Globals
|
\****************************************************************************/

// the global LoRaWAN instance.
cMyLoRaWAN myLoRaWAN {};  

// the global sensor instance
cSensor mySensor {};

// the global event log instance
Arduino_LoRaWAN::cEventLog myEventLog;

/****************************************************************************\
|
| Provisioning info for LoRaWAN OTAA
|
\****************************************************************************/

//
// For normal use, we require that you edit the sketch to replace FILLMEIN_x
// with values assigned by the your network. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN_x to non-
// working but innocuous values.
//
// #define COMPILE_REGRESSION_TEST 1

//#ifdef COMPILE_REGRESSION_TEST
//# define FILLMEIN_8     1, 0, 0, 0, 0, 0, 0, 0
//# define FILLMEIN_16    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2
//#else
//# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
//# define FILLMEIN_8 (#dont edit this, edit the lines that use FILLMEIN_8)
//# define FILLMEIN_16 (#dont edit this, edit the lines that use FILLMEIN_16)
//#endif

// deveui, little-endian
static const std::uint8_t deveui[] = { 0x66, 0x18, 0x4E, 0x66, 0x45, 0xF9, 0x81, 0x60 };

// appeui, little-endian
static const std::uint8_t appeui[] = { 0x1C, 0x42, 0xF6, 0x22, 0x2A, 0xF9, 0x81, 0x60 };

// appkey: just a string of bytes, sometimes referred to as "big endian".
static const std::uint8_t appkey[] = { 0x29, 0x74, 0xB4, 0x5F, 0x02, 0xDA, 0xEC, 0xD4, 0xDD, 0xB1, 0xFE, 0x5D, 0xF7, 0x2C, 0xDD, 0x8C  };

/****************************************************************************\
|
| setup()
|
\****************************************************************************/
bool led_on = false;
void setup() {

    //builtin-LED
    pinMode(LED_BUILTIN, OUTPUT);
    
    // set baud rate, and wait for serial to be ready.
    Serial.begin(115200);
    while (! Serial)
        yield();

    Serial.println(F("simple_sensor_bmi280.ino: setup()"));

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);                       // wait for a second
    rapidBlink();

    // set up the log; do this fisrt.
    //myEventLog.setup();
    //Serial.println("log setup complete");
    
    // set up lorawan.
    myLoRaWAN.setup();
    Serial.println("lorawan setup complete");

    // similarly, set up the sensor.
    mySensor.setup(5*1000);
      Serial.println("sensor setup complete");

    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
//    
//    bool provisioned = myLoRaWAN.IsProvisioned();
//    Serial.println(provisioned);    
//    Serial.println(myLoRaWAN.GetNetworkName());
    rapidBlink();



}

void rapidBlink() {
    for(int i = 0; i < 10; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
/****************************************************************************\
|
| loop()
|
\****************************************************************************/

void loop() {
    // the order of these is arbitrary, but you must poll them all.
    digitalWrite(LED_BUILTIN, led_on);
    led_on = !led_on;
    myLoRaWAN.loop();
    mySensor.loop();
    myEventLog.loop();
}

/****************************************************************************\
|
| LoRaWAN methods
|
\****************************************************************************/

// our setup routine does the class setup and then registers an event handler so
// we can see some interesting things
void
cMyLoRaWAN::setup() {
    // simply call begin() w/o parameters, and the LMIC's built-in
    // configuration for this board will be used.
    this->Super::begin();
    this->SetLinkCheckMode(false);
//    LMIC_selectSubBand(0);  ** don't need because helium
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    this->RegisterListener(
        // use a lambda so we're "inside" the cMyLoRaWAN from public/private perspective
        [](void *pClientInfo, uint32_t event) -> void {
            auto const pThis = (cMyLoRaWAN *)pClientInfo;

            // for tx start, we quickly capture the channel and the RPS
            if (event == EV_TXSTART) {
                // use another lambda to make log prints easy
                myEventLog.logEvent(
                    (void *) pThis,
                    LMIC.txChnl,
                    LMIC.rps,
                    0,
                    // the print-out function
                    [](cEventLog::EventNode_t const *pEvent) -> void {
                        Serial.print(F(" TX:"));
                        myEventLog.printCh(std::uint8_t(pEvent->getData(0)));
                        myEventLog.printRps(rps_t(pEvent->getData(1)));
                    }
                );
            }
            // else if (event == some other), record with print-out function
            else {
                // do nothing.
            }
        },
        (void *) this   // in case we need it.
        );
}

// this method is called when the LMIC needs OTAA info.
// return false to indicate "no provisioning", otherwise
// fill in the data and return true.
bool
cMyLoRaWAN::GetOtaaProvisioningInfo(
    OtaaProvisioningInfo *pInfo
    ) {

    // initialize info
    memcpy(pInfo->DevEUI, deveui, sizeof(pInfo->DevEUI));
    memcpy(pInfo->AppEUI, appeui, sizeof(pInfo->AppEUI));
    memcpy(pInfo->AppKey, appkey, sizeof(pInfo->AppKey));

    return true;
}

// save Info somewhere (if possible)
// if not possible, just do nothing and make sure you return false
// from NetGetSessionState().
void
cMyLoRaWAN::NetSaveSessionInfo(
    const SessionInfo &Info,
    const uint8_t *pExtraInfo,
    size_t nExtraInfo
    ) {
    // in this example, we can't save, so we just return.
}

// save State somewhere. Note that it's often the same;
// often only the frame counters change.
// if not possible, just do nothing and make sure you return false
// from NetGetSessionState().
void
cMyLoRaWAN::NetSaveSessionState(const SessionState &State) {
    // in this example, we can't save, so we just return.
}

// either fetch SessionState from somewhere and return true or...
// return false, which forces a re-join.
bool
cMyLoRaWAN::NetGetSessionState(SessionState &State) {
    // we didn't save earlier, so just tell the core we don't have data.
    return false;
}


/****************************************************************************\
|
| Sensor methods
|
\****************************************************************************/

void
cSensor::setup(std::uint32_t uplinkPeriodMs) {
    if (! this->bme.begin()) {
        while (true) {
            // just loop forever, printing an error occasionally.
            Serial.println("BME280.begin failed");
            delay(2000);
        }
    }
      
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
    
    // set the initial time.
    this->m_uplinkPeriodMs = uplinkPeriodMs;
    this->m_tReference = millis();

    // uplink right away
    this->m_fUplinkRequest = true;
}

void
cSensor::loop(void) {
    auto const tNow = millis();
    auto const deltaT = tNow - this->m_tReference;

    if (deltaT >= this->m_uplinkPeriodMs) {
        Serial.println("requesting uplink");
        // request an uplink
        this->m_fUplinkRequest = true;

        // keep trigger time locked to uplinkPeriod
        auto const advance = deltaT / this->m_uplinkPeriodMs;
        this->m_tReference += advance * this->m_uplinkPeriodMs; 
    }

    // if an uplink was requested, do it.
    if (this->m_fUplinkRequest) {
        this->m_fUplinkRequest = false;
        this->doUplink();
    }
}

/*
 * I am sorry your eyes need to see this code.
 * CayenneLPP requires very strict rounding of floating point numbers to work. If they are not rounded correctly, the decoder function will break and return a failure.
 */
float roundNumber(int input) {
  return (float)((int)(input*10))/10;
}

/*
 * This is the method that formats and sends the uplink package. 
 */
void
cSensor::doUplink(void) {
    Serial.println("attempting the uplink");
    
    // if busy uplinking, just skip
    if (this->m_fBusy){
        Serial.println("already uplinking, skipping!");
        return;
    }  
        
    // if LMIC is busy, just skip
    if (LMIC.opmode & (OP_POLL | OP_TXDATA | OP_TXRXPEND)){
        if (LMIC.opmode & OP_POLL) {
          Serial.println("OP_POLL! flagged");  
        }

        if (LMIC.opmode & OP_TXDATA) {
          Serial.println("OP_TXDATA! flagged");  
        }
        
        if (LMIC.opmode & OP_TXRXPEND) {
          Serial.println("OP_TXRXPEND! flagged");  
        }
        Serial.println("lmic busy, skipping uplink request!..");
        return;
      }

    //attempt to perform reading
    if (! bme.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }

   
    // format the uplink
   
    //cayenne lpp encoder/decoder
    CayenneLPP lpp(51);

    //reset required to erase any previous data stored
    lpp.reset();

    // add sensor data to LPP
    lpp.addTemperature(0, roundNumber(bme.temperature));    
    lpp.addBarometricPressure(1,roundNumber(bme.pressure/100));
    lpp.addRelativeHumidity(2, roundNumber(bme.humidity));
    lpp.addAltitude(3, roundNumber(bme.readAltitude(SEALEVELPRESSURE_HPA)));

    // **notes on LPP: if you don't format it to the correct decimal place, lpp breaks and can't be decoded.

    // turn the busy flag true so we don't try uplinking again before the first package finishes
    this->m_fBusy = true;
    
    Serial.println("sending the buffer now!");
    
    //SendBuffer takes the buffer pointer, its size, and a pointer to a completion function called once the package is sent.
    if (! myLoRaWAN.SendBuffer(
        lpp.getBuffer(), lpp.getSize(),
        // this is the completion function:
        [](void *pClientData, bool fSucccess) -> void {
            Serial.println("callback!");
            auto const pThis = (cSensor *)pClientData;
            pThis->m_fBusy = false;
        },
        (void *)this,
        /* confirmed */ false,
        /* port */ 1
        )) {
        // sending failed; callback has not been called and will not
        // be called. Reset busy flag.
        this->m_fBusy = false;
    }


}
