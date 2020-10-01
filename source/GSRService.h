/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
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

 /* modified by jdohm to fit GSR Values rather than HR */

#ifndef MBED_BLE_GSR_SERVICE_H__
#define MBED_BLE_GSR_SERVICE_H__

#include "ble/BLE.h"

#if BLE_FEATURE_GATT_SERVER

/**
 * BLE GSR Service.
 *
 * @par purpose
 *
 *
 * Clients can read the intended location of the sensor and the last galvanic skin resistance
 * value measured. Additionally, clients can subscribe to server initiated
 * updates of the GSR value measured by the sensor. The service delivers
 * these updates to the subscribed client in a notification packet.
 *
 * The subscription mechanism is useful to save power; it avoids unecessary data
 * traffic between the client and the server, which may be induced by polling the
 * value of the GSR measurement characteristic.
 *
 * @par usage
 *
 * When this class is instantiated, it adds a GSR service in the GattServer.
 * The service contains the location of the sensor and the initial value measured
 * by the sensor.
 *
 * Application code can invoke updateGSR() when a new GSR measurement
 * is acquired; this function updates the value of the GSR measurement
 * characteristic and notifies the new value to subscribed clients.
 *
 */
class GSRService {
public:
    const static uint16_t GSR_SERVICE_UUID              = 0xA000;
    const static uint16_t GSR_STATE_CHARACTERISTIC_UUID = 0xA001;
    /**
     * Intended location of the sensor.
     */
    enum BodySensorLocation {
        /**
         * Other location.
         */
        LOCATION_OTHER = 0,

        /**
         * Chest.
         */
        LOCATION_CHEST = 1,

        /**
         * Wrist.
         */
        LOCATION_WRIST = 2,

        /**
         * Finger.
         */
        LOCATION_FINGER,

        /**
         * Hand.
         */
        LOCATION_HAND,

        /**
         * Earlobe.
         */
        LOCATION_EAR_LOBE,

        /**
         * Foot.
         */
        LOCATION_FOOT,
    };

public:
    /**
     * Construct and initialize a GSR service.
     *
     * The construction process adds a GATT GSR service in @p _ble
     * GattServer, sets the value of the GSR measurement characteristic
     * to @p gsrCounter and the value of the body sensor location characteristic
     * to @p location.
     *
     * @param[in] _ble BLE device that hosts the gsr service.
     * @param[in] gsrCounter adc value (0-0xFFFF) which corelates to skin resistance
     * @param[in] location Intended location of the electrodes.
     */
    GSRService(BLE &_ble, uint16_t gsrCounter, BodySensorLocation location) :
        ble(_ble),
        valueBytes(gsrCounter),
        gsrRate(
            GSRService::GSR_STATE_CHARACTERISTIC_UUID,
            valueBytes.getPointer(),
            2,
            GSRValueBytes::MAX_VALUE_BYTES,
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
        ),
        gsrLocation(
            GattCharacteristic::UUID_BODY_SENSOR_LOCATION_CHAR,
            reinterpret_cast<uint8_t*>(&location)
        )
    {
        setupService();
    }

    /**
     * Update the GSR that the service exposes.
     *
     * The server sends a notification of the new value to clients that have
     * subscribed to updates of the GSR measurement characteristic; clients
     * reading the GSR measurement characteristic after the update obtain
     * the updated value.
     *
     * @param[in] gsrCounter GSR Valie as measured by the adc.
     *
     * @attention This function must be called in the execution context of the
     * BLE stack.
     */
    void updateGSR(uint16_t gsrCounter) {
        valueBytes.updateGSR(gsrCounter);
        ble.gattServer().write(
            gsrRate.getValueHandle(),
            valueBytes.getPointer(),
            2
        );
    }

protected:
    /**
     * Construct and add to the GattServer the GSR service.
     */
    void setupService(void) {
        GattCharacteristic *charTable[] = {
            &gsrRate,
            &gsrLocation
        };
        GattService gsrService(
            GSRService::GSR_SERVICE_UUID,
            charTable,
            sizeof(charTable) / sizeof(GattCharacteristic*)
        );

        ble.gattServer().addService(gsrService);
    }

protected:
    /*
     * GSR value.
     */
    struct GSRValueBytes {
        /* 2 byte for the ADC GSR Value (16bit). */
        static const unsigned MAX_VALUE_BYTES = 2;
        static const unsigned FLAGS_BYTE_INDEX = 0;

        static const unsigned VALUE_FORMAT_BITNUM = 0;
        static const uint8_t  VALUE_FORMAT_FLAG = (1 << VALUE_FORMAT_BITNUM);

        GSRValueBytes(uint16_t gsrCounter) : valueBytes()
        {
            updateGSR(gsrCounter);
        }

        void updateGSR(uint16_t gsrCounter)
        {
            valueBytes[1] = (gsrCounter>>8);
            valueBytes[0] = gsrCounter;
        }

        uint8_t *getPointer(void)
        {
            return valueBytes;
        }

        const uint8_t *getPointer(void) const
        {
            return valueBytes;
        }

    private:
        uint8_t valueBytes[MAX_VALUE_BYTES];
    };

protected:
    BLE &ble;
    GSRValueBytes valueBytes;
    GattCharacteristic gsrRate;
    ReadOnlyGattCharacteristic<uint8_t> gsrLocation;
};

#endif // BLE_FEATURE_GATT_SERVER

#endif /* #ifndef MBED_BLE_GSR_SERVICE_H__*/
