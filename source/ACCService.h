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

#ifndef __BLE_ACC_SERVICE_H__
#define __BLE_ACC_SERVICE_H__

#include <mbed.h>
#include "ble/BLE.h"
#include "ble/gap/Gap.h"

class ACCService {
public:
    const static uint16_t ACC_SERVICE_UUID              = 0xA020;
    const static uint16_t ACC_STATE_CHARACTERISTIC_UUID = 0xA021;

    ACCService(BLE &_ble, uint16_t xACCInitial) :
        ble(_ble), buttonState(ACC_STATE_CHARACTERISTIC_UUID, &xACCInitial, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY)
            {
                GattCharacteristic *charTable[] = {&buttonState};
                GattService         ACCService(ACCService::ACC_SERVICE_UUID, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));
                ble.gattServer().addService(ACCService);
            }

    void updateButtonState(uint16_t newState) {
        ble.gattServer().write(buttonState.getValueHandle(), (uint8_t *)&newState, sizeof(uint16_t));
    }

private:
    BLE                              &ble;
    ReadOnlyGattCharacteristic<uint16_t>  buttonState;
};

#endif /* #ifndef __BLE_ACC_SERVICE_H__ */