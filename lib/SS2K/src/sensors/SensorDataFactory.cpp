/*
 * Copyright (C) 2020  Anthony Doud & Joel Baranick
 * All rights reserved
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <cmath>
#include "Constants.h"
#include "sensors/SensorDataFactory.h"
#include "sensors/CyclePowerData.h"
#include "sensors/FlywheelData.h"
#include "sensors/FitnessMachineIndoorBikeData.h"
#include "sensors/HeartRateData.h"
#include "sensors/EchelonData.h"

std::shared_ptr<SensorData> SensorDataFactory::getSensorData(const NimBLEUUID characteristicUUID, uint8_t *data, size_t length) {
  for (auto &it : SensorDataFactory::knownDevices) {
    if (it->getUUID() == characteristicUUID) {
      return it->decode(data, length);
    }
  }

  std::shared_ptr<SensorData> sensorData = NULL_SENSOR_DATA;
  if (characteristicUUID == CYCLINGPOWERMEASUREMENT_UUID) {
    sensorData = std::shared_ptr<SensorData>(new CyclePowerData());
  } else if (characteristicUUID == HEARTCHARACTERISTIC_UUID) {
    sensorData = std::shared_ptr<SensorData>(new HeartRateData());
  } else if (characteristicUUID == FITNESSMACHINEINDOORBIKEDATA_UUID) {
    sensorData = std::shared_ptr<SensorData>(new FitnessMachineIndoorBikeData());
  } else if (characteristicUUID == FLYWHEEL_UART_SERVICE_UUID) {
    sensorData = std::shared_ptr<SensorData>(new FlywheelData());
  } else if (characteristicUUID == ECHELON_DATA_UUID) {
    sensorData = std::shared_ptr<SensorData>(new EchelonData());
  } else {
    return NULL_SENSOR_DATA;
  }

  KnownDevice *knownDevice = new KnownDevice(characteristicUUID, sensorData);
  SensorDataFactory::knownDevices.push_back(knownDevice);
  return knownDevice->decode(data, length);
}

NimBLEUUID SensorDataFactory::KnownDevice::getUUID() { return this->uuid; }

std::shared_ptr<SensorData> SensorDataFactory::KnownDevice::decode(uint8_t *data, size_t length) {
  this->sensorData->decode(data, length);
  return this->sensorData;
}

bool SensorDataFactory::NullData::hasHeartRate() { return false; }

bool SensorDataFactory::NullData::hasCadence() { return false; }

bool SensorDataFactory::NullData::hasPower() { return false; }

bool SensorDataFactory::NullData::hasSpeed() { return false; }

int SensorDataFactory::NullData::getHeartRate() { return INT_MIN; }

float SensorDataFactory::NullData::getCadence() { return nanf(""); }

int SensorDataFactory::NullData::getPower() { return INT_MIN; }

float SensorDataFactory::NullData::getSpeed() { return nanf(""); }

void SensorDataFactory::NullData::decode(uint8_t *data, size_t length) {}

std::shared_ptr<SensorData> SensorDataFactory::NULL_SENSOR_DATA = std::shared_ptr<SensorData>(new NullData());
