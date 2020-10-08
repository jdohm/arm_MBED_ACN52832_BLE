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

//#define CDEBUG

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/gap/Gap.h"
#include "ble/services/HeartRateService.h"
#include "pretty_printer.h"
#include "SEGGER_RTT.h"
#include "SEGGER_RTT.c"
#include "SEGGER_RTT_printf.c"

// Initialise the digital pin LD1 as an output
DigitalOut led(p24);

// Additions to add I2C Heartrate Sensor SEN-15219
// Based on the arm mbed example: https://os.mbed.com/docs/mbed-os/v6.1/apis/i2c.html
#include "SparkFun_Bio_Sensor_Hub_Library.h"

//Adding GSR to Bluetooth
#include "GSRService.h"
AnalogIn adc_gsr1(p28);
//Adding Stress to Bluetooth
#include "StressService.h"

//Adding ACC to this program
#include "BMA253.h"

//HR
I2C i2c(I2C_SDA0,I2C_SCL0);
  DigitalOut   resetPin(p26,0);
  DigitalInOut mfioPin(p25,PIN_OUTPUT,PullUp,0);
//SparkFun_Bio_Sensor_Hub bioHub(p26, p25);//resPin, mfioPin
SparkFun_Bio_Sensor_Hub bioHub(resetPin, mfioPin);//resPin, mfioPin

bioData body;

    // --- ACC ---
    DigitalIn   ACC_Int1(p27);
    BMA253 bma(ADDRESS_ONE);
    bmaData accData;
//I2C end

// TODO Sensorsoeckchen?
const static char DEVICE_NAME[] = "Prototyp 2.1";

static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

class GSRLoggerC : ble::Gap::EventHandler {
public:
    GSRLoggerC(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),
        //_led1(LED1, 1),
        _connected(false),
        _gsr_uuid(GSRService::GSR_SERVICE_UUID),
        _stress_uuid(StressService::STRESS_SERVICE_UUID),
        _gsr_counter(100),
        _gsr_service(ble, _gsr_counter, GSRService::LOCATION_FOOT),
        _stress_service(ble),
        _adv_data_builder(_adv_buffer) { }

    void start() {
        _ble.gap().setEventHandler(this);

        _ble.init(this, &GSRLoggerC::on_init_complete);
        //TODO Testdatenlogging mit Intervall von 100ms (wie Prototyp 1)
        //_event_queue.call_every(500ms, this, &GSRLoggerC::blink);//LED ohne Funktion bei 1.8V
        _event_queue.call_every(100ms, this, &GSRLoggerC::update_sensor_value);

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
            ble::adv_interval_t(ble::millisecond_t(500))
        );
        //TODO an GSR bzw Sensorsoeckhen Anpassen?!
        _adv_data_builder.setFlags();
        _adv_data_builder.setAppearance(ble::adv_data_appearance_t::UNKNOWN);
        _uuids[0] = _gsr_uuid;
        _uuids[1] = _stress_uuid;
        _adv_data_builder.setLocalServiceList(mbed::make_Span(_uuids, 2));
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
            uint16_t newgsr = adc_gsr1.read_u16();
            _gsr_service.updateGSR(newgsr);
            if (newgsr < 0x0FFF) _stress_service.updatestressState(true);
            else _stress_service.updatestressState(false);
            //x komponente der Beschleunigung anstelle des GSR Werts (fuer Tests)
            //_gsr_service.updateGSR(bma.read().x);
        }
    }

    void blink(void) {
        //_led1 = !_led1;
        led = !led;
        #ifdef CDEBUG
        body = bioHub.readBpm();
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

    UUID _gsr_uuid;
    UUID _stress_uuid;
    UUID _uuids[2];

    uint8_t _gsr_counter;
    GSRService _gsr_service;
    
    //Stress
    StressService _stress_service;

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
    //BMA253 Acceleration - start
    result = bma.begin(i2c);
    result = bma.setElIntBehaviour(INTERRUPT_PIN_INT1,INTERRUPT_EL_BEHAVIOUR_PUSHPULL,INTERRUPT_EL_BEHAVIOUR_LVL_NORMAL);
    #ifdef CDEBUG
		SEGGER_RTT_printf(0,"#setElIntBehaviour returned: %X\n",result);
	#endif
    result = bma.moveIntSetThreashold(0x10,0b10);
    #ifdef CDEBUG
		SEGGER_RTT_printf(0,"#moveIntSetThreashold returned: %X\n",result);
	#endif
    result = bma.moveInt(true);
    #ifdef CDEBUG
		SEGGER_RTT_printf(0,"#moveInt returned: %X\n",result);
	#endif
    result = bma.knockOnInt(true);
    #ifdef CDEBUG
   	    SEGGER_RTT_printf(0,"#knockOnInt returned: %X\n",result);
    #endif
   ThisThread::sleep_for(1s);
    //BMA253 Acceleration - end


    GSRLoggerC demo(ble, event_queue);
    demo.start();

    return 0;
}

