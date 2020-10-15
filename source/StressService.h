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
    const static uint16_t STRESS_LVL_CHARACTERISTIC_UUID = 0xA012;
    const static uint16_t STRESS_THRESHOLD_CHARACTERISTIC_UUID = 0xA013;

    StressService(BLE &_ble) :
        ble(_ble), 
        stressState(STRESS_DETECTED_CHARACTERISTIC_UUID, &falseP, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY), 
        stressLvl(STRESS_LVL_CHARACTERISTIC_UUID, &nullLvl, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),
        stressThreshold(STRESS_THRESHOLD_CHARACTERISTIC_UUID, &Threshold)
    {
        GattCharacteristic *charTable[] = {&stressState, &stressLvl, &stressThreshold};
        GattService         StressService(StressService::STRESS_SERVICE_UUID, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));
        ble.gattServer().addService(StressService);
    }

    void updatestressState(bool newState) {
        ble.gattServer().write(stressState.getValueHandle(), (uint8_t *)&newState, sizeof(bool));
    }

    void updatestressLvl(uint16_t newLvl) {
        ble.gattServer().write(stressLvl.getValueHandle(), (uint8_t *)&newLvl, sizeof(uint16_t));
    }

    void updatestressThreshold(uint16_t newThreshhold) {
        ble.gattServer().write(stressThreshold.getValueHandle(), (uint8_t *)&newThreshhold, sizeof(uint16_t));
    }

    uint16_t readThreshold(void) {
        uint8_t ThresholdNow[2];
        uint16_t length=2*sizeof(uint8_t);
        uint16_t ThresholdOut;
        ble.gattServer().read(stressThreshold.getValueHandle(), ThresholdNow, &length);
        ThresholdOut = uint16_t(ThresholdNow[0]&0x00FF | uint16_t(ThresholdNow[1]&0x00FF)<<8);

        //ble.gattServer().read(ble.getValueHandle(),stressThreshold.getValueHandle(), (uint8_t *)&ThresholdOut, &length);
        return ThresholdOut;
    }

private:
    BLE                              &ble;
    ReadOnlyGattCharacteristic<bool>  stressState;
    ReadOnlyGattCharacteristic<uint16_t>  stressLvl;
    ReadWriteGattCharacteristic<uint16_t>  stressThreshold;
    bool    falseP = false;
    uint16_t    nullLvl = 0x0000;
    uint16_t    Threshold = 0x0050;
};

#endif /* #ifndef __BLE_STRESS_SERVICE_H__ */