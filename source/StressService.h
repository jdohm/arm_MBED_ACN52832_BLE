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

#ifndef __BLE_STRESS_SERVICE_H__
#define __BLE_STRESS_SERVICE_H__

#include "ble/BLE.h"

class StressService {
public:
    const static uint16_t STRESS_SERVICE_UUID              = 0xA010;
    const static uint16_t STRESS_DETECTED_CHARACTERISTIC_UUID = 0xA011;

    StressService(BLE &_ble) :
        ble(_ble), stressState(STRESS_DETECTED_CHARACTERISTIC_UUID, &falseP, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY)
    {
        GattCharacteristic *charTable[] = {&stressState};
        GattService         StressService(StressService::STRESS_SERVICE_UUID, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));
        ble.gattServer().addService(StressService);
    }

    void updatestressState(bool newState) {
        ble.gattServer().write(stressState.getValueHandle(), (uint8_t *)&newState, sizeof(bool));
    }

private:
    BLE                              &ble;
    ReadOnlyGattCharacteristic<bool>  stressState;
    bool    falseP = false;
};

#endif /* #ifndef __BLE_STRESS_SERVICE_H__ */