/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
//TODO
// - Umbau fuer Datenlogging wie bei Prototyp 1. (todo suchen)
// - Benachrichtigung wenn Stressdetektiert implementiere?
// X Debugging Ueber debugging defina abschaltbar gestalten 

#define CDEBUG

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/gap/Gap.h"
#include "ble/services/HeartRateService.h"
#include "pretty_printer.h"
#include "SEGGER_RTT.h"
#include "SEGGER_RTT.c"
#include "SEGGER_RTT_printf.c"

// Additions to add I2C Heartrate Sensor SEN-15219
// Based on the arm mbed example: https://os.mbed.com/docs/mbed-os/v6.1/apis/i2c.html
#include "SparkFun_Bio_Sensor_Hub_Library.h"

//Adding GSR to Bluetooth
//#include "GSRService.h"

I2C i2c(I2C_SDA0,I2C_SCL0);
  DigitalOut   resetPin(p26,0);
  DigitalInOut mfioPin(p25,PIN_OUTPUT,PullUp,0);
//SparkFun_Bio_Sensor_Hub bioHub(p26, p25);//resPin, mfioPin
SparkFun_Bio_Sensor_Hub bioHub(resetPin, mfioPin);//resPin, mfioPin

bioData body;

//I2C end

// TODO Sensorsoeckchen?
const static char DEVICE_NAME[] = "Prototyp 2";

static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

class HeartrateDemo : ble::Gap::EventHandler {
public:
    HeartrateDemo(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),
        //_led1(LED1, 1),
        _connected(false),
        _hr_uuid(GattService::UUID_HEART_RATE_SERVICE),
        _hr_counter(100),
        _hr_service(ble, _hr_counter, HeartRateService::LOCATION_FINGER),
        _adv_data_builder(_adv_buffer) { }

    void start() {
        _ble.gap().setEventHandler(this);

        _ble.init(this, &HeartrateDemo::on_init_complete);
        //TODO Testdatenlogging mit Intervall von 100ms (wie Prototyp 1)
        _event_queue.call_every(500ms, this, &HeartrateDemo::blink);
        _event_queue.call_every(1s, this, &HeartrateDemo::update_sensor_value);

        _event_queue.dispatch_forever();
    }

private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        if (params->error != BLE_ERROR_NONE) {
            SEGGER_RTT_WriteString(0,"Ble initialization failed.\n");
            return;
        }

        print_mac_address();

        start_advertising();
    }

    void start_advertising() {
        /* Create advertising parameters and payload */

        ble::AdvertisingParameters adv_parameters(
            ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(1000))
        );
        //TODO an GSR bzw Sensorsoeckhen Anpassen?!
        _adv_data_builder.setFlags();
        _adv_data_builder.setAppearance(ble::adv_data_appearance_t::GENERIC_HEART_RATE_SENSOR);
        _adv_data_builder.setLocalServiceList(mbed::make_Span(&_hr_uuid, 1));
        _adv_data_builder.setName(DEVICE_NAME);

        /* Setup advertising */

        ble_error_t error = _ble.gap().setAdvertisingParameters(
            ble::LEGACY_ADVERTISING_HANDLE,
            adv_parameters
        );

        if (error) {
            SEGGER_RTT_WriteString(0,"_ble.gap().setAdvertisingParameters() failed\r\n");
            return;
        }

        error = _ble.gap().setAdvertisingPayload(
            ble::LEGACY_ADVERTISING_HANDLE,
            _adv_data_builder.getAdvertisingData()
        );

        if (error) {
            SEGGER_RTT_WriteString(0,"_ble.gap().setAdvertisingPayload() failed\r\n");
            return;
        }

        /* Start advertising */

        error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

        if (error) {
            SEGGER_RTT_WriteString(0,"_ble.gap().startAdvertising() failed\r\n");
            return;
        }
    }

    void update_sensor_value() {
        if (_connected) {
            //Todo um andere Sensorwerte erweitern
            body = bioHub.readBpm();
            _hr_service.updateHeartRate(body.heartRate);
        }
    }

    void blink(void) {
        //_led1 = !_led1;
        body = bioHub.readBpm();
        #ifdef CDEBUG
        SEGGER_RTT_printf(0,"Heartrate: %d\n",body.heartRate);
        SEGGER_RTT_printf(0,"Confidence: %d\n",body.confidence);
        SEGGER_RTT_printf(0,"Oxygen: %d\n",body.oxygen);
        SEGGER_RTT_printf(0,"Status: %d\n",body.status);
        #endif
    }

private:
    /* Event handler */

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
        _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        _connected = false;
    }

    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event) {
        if (event.getStatus() == BLE_ERROR_NONE) {
            _connected = true;
        }
    }

private:
    BLE &_ble;
    events::EventQueue &_event_queue;
    //DigitalOut _led1;

    bool _connected;

    UUID _hr_uuid;

    uint8_t _hr_counter;
    HeartRateService _hr_service;

    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

int main()
{
    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);

    //mfioPin.write(0);
    int result = bioHub.begin(i2c);
    #ifdef CDEBUG
    SEGGER_RTT_printf(0,"bioHub returned %d\n",result);
    #endif //CDEBUG
    ThisThread::sleep_for(1s);
    result = bioHub.configBpm(MODE_ONE);
    #ifdef CDEBUG
    SEGGER_RTT_printf(0,"bioHub.configBpm returned %d\n",result);
    #endif //CDEBUG
    ThisThread::sleep_for(4s);
    body = bioHub.readBpm();
    #ifdef CDEBUG
    SEGGER_RTT_printf(0,"Heartrate: %d\n",body.heartRate);
    SEGGER_RTT_printf(0,"Confidence: %d\n",body.confidence);
    SEGGER_RTT_printf(0,"Oxygen: %d\n",body.oxygen);
    SEGGER_RTT_printf(0,"Status: %d\n",body.status);
    #endif //CDEBUG

    ThisThread::sleep_for(1s);

    HeartrateDemo demo(ble, event_queue);
    demo.start();

    return 0;
}

