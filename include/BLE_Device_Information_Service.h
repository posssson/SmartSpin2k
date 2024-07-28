/*
 * Copyright (C) 2020  Anthony Doud & Joel Baranick
 * All rights reserved
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <NimBLEDevice.h>
#include "BLE_Common.h"

class BLE_Device_Information_Service {
 public:
  BLE_Device_Information_Service();
  void setupService(NimBLEServer* pServer);

 private:
  NimBLEService* pDeviceInformationService;
  NimBLECharacteristic* pManufacturerNameCharacteristic;
  NimBLECharacteristic* pModelNumberCharacteristic;
  NimBLECharacteristic* pSerialNumberCharacteristic;
  NimBLECharacteristic* pHardwareRevisionCharacteristic;
  NimBLECharacteristic* pFirmwareRevisionCharacteristic;
  NimBLECharacteristic* pSoftwareRevisionCharacteristic;
  NimBLECharacteristic* pSystemIDCharacteristic;
  NimBLECharacteristic* pPnPIDCharacteristic;
};
