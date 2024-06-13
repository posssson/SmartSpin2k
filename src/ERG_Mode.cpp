/*
 * Copyright (C) 2020  Anthony Doud & Joel Baranick
 * All rights reserved
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "ERG_Mode.h"
#include "SS2KLog.h"
#include "Main.h"
#include <LittleFS.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

TaskHandle_t ErgTask;
PowerTable powerTable;

// Create a torque table representing 0w-1000w in 50w increments.
// i.e. powerTable[1] corresponds to the incline required for 50w. powerTable[2] is the incline required for 100w and so on.

void setupERG() {
  SS2K_LOG(ERG_MODE_LOG_TAG, "Starting ERG Mode task...");
  xTaskCreatePinnedToCore(ergTaskLoop,    /* Task function. */
                          "FTMSModeTask", /* name of task. */
                          ERG_STACK,      /* Stack size of task*/
                          NULL,           /* parameter of the task */
                          1,              /* priority of the task*/
                          &ErgTask,       /* Task handle to keep track of created task */
                          0);             /* pin task to core 0 */

  SS2K_LOG(ERG_MODE_LOG_TAG, "ERG Mode task started");
}

void ergTaskLoop(void* pvParameters) {
  ErgMode ergMode = ErgMode(&powerTable);
  PowerBuffer powerBuffer;

  ergMode._writeLogHeader();
  bool isInErgMode            = false;
  bool hasConnectedPowerMeter = false;
  bool simulationRunning      = false;
  int loopCounter             = 0;

  while (true) {
    // be quiet while updating via BLE
    while (ss2k->isUpdating) {
      vTaskDelay(100);
    }

    vTaskDelay(ERG_MODE_DELAY / portTICK_PERIOD_MS);

    hasConnectedPowerMeter = spinBLEClient.connectedPM;
    simulationRunning      = rtConfig->watts.getTarget();
    if (!simulationRunning) {
      simulationRunning = rtConfig->watts.getSimulate();
    }

    // add values to torque table
    powerTable.processPowerValue(powerBuffer, rtConfig->cad.getValue(), rtConfig->watts);

    // compute ERG
    if ((rtConfig->getFTMSMode() == FitnessMachineControlPointProcedure::SetTargetPower) && (hasConnectedPowerMeter || simulationRunning)) {
      ergMode.computeErg();
    }

    // resistance mode
    if ((rtConfig->getFTMSMode() == FitnessMachineControlPointProcedure::SetTargetResistanceLevel) && (rtConfig->getMaxResistance() != DEFAULT_RESISTANCE_RANGE)) {
      ergMode.computeResistance();
    }

    // Set Min and Max Stepper positions
    if (loopCounter > 50) {
      loopCounter = 0;
      powerTable.setStepperMinMax();
    }

    loopCounter++;

#ifdef DEBUG_STACK
    Serial.printf("ERG Task: %d \n", uxTaskGetStackHighWaterMark(ErgTask));
#endif  // DEBUG_STACK
  }
}

void PowerBuffer::set(int i) {
  this->powerEntry[i].readings++;
  this->powerEntry[i].watts          = rtConfig->watts.getValue();
  this->powerEntry[i].cad            = rtConfig->cad.getValue();
  this->powerEntry[i].targetPosition = rtConfig->getCurrentIncline() / 100;  // dividing by 100 to save memory.
}

void PowerBuffer::reset() {
  for (int i = 0; i < POWER_SAMPLES; i++) {
    this->powerEntry[i].readings       = 0;
    this->powerEntry[i].cad            = 0;
    this->powerEntry[i].watts          = 0;
    this->powerEntry[i].targetPosition = 0;
  }
}

// return the number of entries with readings.
int PowerBuffer::getReadings() {
  int ret = 0;
  for (int i = 0; i < POWER_SAMPLES; i++) {
    if (this->powerEntry[i].readings != 0) {
      ret++;
    }
  }
  return ret;
}

void PowerTable::processPowerValue(PowerBuffer& powerBuffer, int cadence, Measurement watts) {
  if ((cadence >= (MINIMUM_TABLE_CAD - (POWERTABLE_CAD_INCREMENT / 2))) &&
      (cadence <= (MINIMUM_TABLE_CAD + (POWERTABLE_CAD_INCREMENT * POWERTABLE_CAD_SIZE) - (POWERTABLE_CAD_SIZE / 2))) && (watts.getValue() > 10) &&
      (watts.getValue() < (POWERTABLE_WATT_SIZE * POWERTABLE_WATT_INCREMENT))) {
    if (powerBuffer.powerEntry[0].readings == 0) {
      // Take Initial reading
      powerBuffer.set(0);
      // Check that reading is within 1/2 of the initial reading
    } else if (abs(powerBuffer.powerEntry[0].watts - watts.getValue()) < (POWERTABLE_WATT_INCREMENT / 2)) {
      for (int i = 1; i < POWER_SAMPLES; i++) {
        if (powerBuffer.powerEntry[i].readings == 0) {
          powerBuffer.set(i);  // Add additional readings to the buffer.
          break;
        }
      }
      if (powerBuffer.powerEntry[POWER_SAMPLES - 1].readings == 1) {  // If buffer is full, create a new table entry and clear the buffer.
        this->newEntry(powerBuffer);
        this->toLog();
        this->_manageSaveState();
        powerBuffer.reset();
      }
    } else {  // Reading was outside the range - clear the buffer and start over.
      powerBuffer.reset();
    }
  }
}

// Set min / max stepper position
void PowerTable::setStepperMinMax() {
  int _return = RETURN_ERROR;

  // if the FTMS device reports resistance feedback, skip estimating min_max
  if (rtConfig->resistance.getValue() > 0) {
    rtConfig->setMinStep(-DEFAULT_STEPPER_TRAVEL);
    rtConfig->setMaxStep(DEFAULT_STEPPER_TRAVEL);
    SS2K_LOG(ERG_MODE_LOG_TAG, "Using Resistance Travel Limits");
    return;
  }

  int minBreakWatts = userConfig->getMinWatts();
  if (minBreakWatts > 1) {
    _return = this->lookup(minBreakWatts, NORMAL_CAD);
    if (_return != RETURN_ERROR) {
      // never set less than one shift below current incline.
      if ((_return >= rtConfig->getCurrentIncline()) && (rtConfig->watts.getValue() > userConfig->getMinWatts())) {
        _return = rtConfig->getCurrentIncline() - userConfig->getShiftStep();
      }
      // never set above max step.
      if (_return + userConfig->getShiftStep() >= rtConfig->getMaxStep()) {
        _return = rtConfig->getCurrentIncline() - userConfig->getShiftStep();
      }
      rtConfig->setMinStep(_return);
      SS2K_LOG(ERG_MODE_LOG_TAG, "Min Position Set: %d", _return);
    }
  }

  int maxBreakWatts = userConfig->getMaxWatts();
  if (maxBreakWatts > 1) {
    _return = this->lookup(maxBreakWatts, NORMAL_CAD);
    if (_return != RETURN_ERROR) {
      // never set less than one shift above current incline.
      if ((_return <= rtConfig->getCurrentIncline()) && (rtConfig->watts.getValue() < userConfig->getMaxWatts())) {
        _return = rtConfig->getCurrentIncline() + userConfig->getShiftStep();
      }
      // never set below min step.
      if (_return - userConfig->getShiftStep() <= rtConfig->getMinStep()) {
        _return = rtConfig->getCurrentIncline() + userConfig->getShiftStep();
      }
      rtConfig->setMaxStep(_return);
      SS2K_LOG(ERG_MODE_LOG_TAG, "Max Position Set: %d", _return);
    }
  }
}

void PowerTable::newEntry(PowerBuffer& powerBuffer) {
  // these are floats so that we make sure division works correctly.
  float watts        = 0;
  float cad          = 0;
  int targetPosition = 0;

  // First, take the power buffer and average all of the samples together.
  int validEntries = 0;
  for (int i = 0; i < POWER_SAMPLES; i++) {
    if (powerBuffer.powerEntry[i].readings == 0) {
      // Stop when buffer is empty
      break;
    }

    // Accumulate values
    watts += powerBuffer.powerEntry[i].watts;
    cad += powerBuffer.powerEntry[i].cad;
    targetPosition += powerBuffer.powerEntry[i].targetPosition;
    validEntries++;
  }

  // Calculate the average if there are valid entries
  if (validEntries > 0) {
    watts /= validEntries;
    cad /= validEntries;
    targetPosition /= validEntries;
  } else {
    SS2K_LOG(POWERTABLE_LOG_TAG, "No valid entries in the power buffer.");
    return;
  }

  // To start working on the PowerTable, we need to calculate position in the table for the new entry
  int i = round(watts / (float)POWERTABLE_WATT_INCREMENT);
  int k = round((cad - (float)MINIMUM_TABLE_CAD) / (float)POWERTABLE_CAD_INCREMENT);
  SS2K_LOG(POWERTABLE_LOG_TAG, "Averaged Entry: watts=%f, cad=%f, targetPosition=%d, (%d)(%d)", watts, cad, targetPosition, k, i);

  // Ensure k is within valid range
  if (k < 0 || k >= POWERTABLE_CAD_SIZE) {
    SS2K_LOG(POWERTABLE_LOG_TAG, "Cad index was out of range");
    return;
  }
  // Ensure i is within valid range
  if (i < 0 || i >= POWERTABLE_WATT_SIZE) {
    SS2K_LOG(POWERTABLE_LOG_TAG, "Watt index was out of range");
    return;
  }

  // Prohibit entries that are less than the number to the left
  if (i > 0) {
    for (int j = i - 1; j >= 0; j--) {
      if (this->tableRow[k].tableEntry[j].targetPosition != INT_MIN) {
        if (this->tableRow[k].tableEntry[j].targetPosition >= targetPosition) {
          SS2K_LOG(POWERTABLE_LOG_TAG, "Target Slot (%d)(%d)(%d)(%d) was less than previous (%d)(%d)(%d)", this->tableRow[k].tableEntry[j].targetPosition, k, i,
                   (int)targetPosition, k, j, this->tableRow[k].tableEntry[j].targetPosition);
          // downvote the blocking entry
          this->tableRow[k].tableEntry[j].readings--;
          // reset the blocking entry
          if (this->tableRow[k].tableEntry[j].readings < 1) {
            this->tableRow[k].tableEntry[j].targetPosition = INT_MIN;
          }
          return;
        }
        break;  // Found a valid left neighbor
      }
    }
  }

  // Prohibit entries that are greater than the number to the right
  if (i < POWERTABLE_WATT_SIZE - 1) {
    for (int j = i + 1; j < POWERTABLE_WATT_SIZE; j++) {
      if (this->tableRow[k].tableEntry[j].targetPosition != INT_MIN) {
        if (targetPosition >= this->tableRow[k].tableEntry[j].targetPosition) {
          SS2K_LOG(POWERTABLE_LOG_TAG, "Target Slot (%d)(%d)(%d)(%d) was greater than next (%d)(%d)(%d)", this->tableRow[k].tableEntry[j].targetPosition, k, i, (int)targetPosition,
                   k, j, this->tableRow[k].tableEntry[j].targetPosition);
          // downvote the blocking entry
          this->tableRow[k].tableEntry[j].readings--;
          // if it's downvoted to 0, reset the blocking entry
          if (this->tableRow[k].tableEntry[j].readings < 1) {
            this->tableRow[k].tableEntry[j].targetPosition = INT_MIN;
          }
          return;
        }
        break;  // Found a valid right neighbor
      }
    }
  }

  // Update or create a new entry
  if (this->tableRow[k].tableEntry[i].readings == 0) {  // if first reading in this entry
    this->tableRow[k].tableEntry[i].targetPosition = targetPosition;
    SS2K_LOG(POWERTABLE_LOG_TAG, "New entry recorded (%d)(%d)(%d)", k, i, this->tableRow[k].tableEntry[i].targetPosition);
  } else {  // Average and update the readings.
    this->tableRow[k].tableEntry[i].targetPosition =
        (targetPosition + (this->tableRow[k].tableEntry[i].targetPosition * this->tableRow[k].tableEntry[i].readings)) / (this->tableRow[k].tableEntry[i].readings + 1.0);
    SS2K_LOG(POWERTABLE_LOG_TAG, "Existing entry averaged (%d)(%d)(%d), readings(%d)", k, i, this->tableRow[k].tableEntry[i].targetPosition,
             this->tableRow[k].tableEntry[i].readings);
    if (this->tableRow[k].tableEntry[i].readings > POWER_SAMPLES * 2) {
      this->tableRow[k].tableEntry[i].readings = POWER_SAMPLES * 2;  // keep from diluting recent readings too far.
    }
  }
  this->tableRow[k].tableEntry[i].readings++;
}

// Helper function to solve linear equation
float linearFit(const std::vector<float>& x, const std::vector<float>& y, float wattage) {
  float x1 = x[0], y1 = y[0];
  float x2 = x[1], y2 = y[1];
  return y1 + (y2 - y1) * (wattage - x1) / (x2 - x1);
}

// Helper function to solve quadratic equation
float quadraticFit(const std::vector<float>& x, const std::vector<float>& y, float wattage) {
  float x1 = x[0], y1 = y[0];
  float x2 = x[1], y2 = y[1];
  float x3 = x[2], y3 = y[2];

  float a = ((y2 - y1) / (x2 - x1) - (y3 - y2) / (x3 - x2)) / (x3 - x1);
  float b = (y2 - y1) / (x2 - x1) - a * (x2 + x1);
  float c = y1 - a * x1 * x1 - b * x1;

  return a * wattage * wattage + b * wattage + c;
}

// Helper function to perform linear interpolation between cadence lines
float cadLinearFit(float wattage, float cad1, float pos1, float cad2, float pos2, float targetCadence) { return pos1 + (pos2 - pos1) * (targetCadence - cad1) / (cad2 - cad1); }

int32_t PowerTable::lookup(int watts, int cad) {
  int i        = round(watts / (float)POWERTABLE_WATT_INCREMENT);
  int cadIndex = round((cad - (float)MINIMUM_TABLE_CAD) / (float)POWERTABLE_CAD_INCREMENT);

  if (cadIndex < 0)
    cadIndex = 0;
  else if (cadIndex >= POWERTABLE_CAD_SIZE)
    cadIndex = POWERTABLE_CAD_SIZE - 1;

  if (i < 0)
    i = 0;
  else if (i >= POWERTABLE_WATT_SIZE)
    i = POWERTABLE_WATT_SIZE - 1;

  // Check if the exact data point is available and within the POWERTABLE_WATT_INCREMENT range
  if (this->tableRow[cadIndex].tableEntry[i].targetPosition != INT_MIN) {
    SS2K_LOG(POWERTABLE_LOG_TAG, "PTab Direct Result (%d)(%d)", cadIndex, i)
    return this->tableRow[cadIndex].tableEntry[i].targetPosition * 100;
  }

  std::vector<float> validWattages;
  std::vector<float> validPositions;

  for (int j = 0; j < POWERTABLE_WATT_SIZE; ++j) {
    if (this->tableRow[cadIndex].tableEntry[j].targetPosition != INT_MIN) {
      validWattages.push_back(j * POWERTABLE_WATT_INCREMENT);
      validPositions.push_back(this->tableRow[cadIndex].tableEntry[j].targetPosition);
    }
  }

  // If no valid data in this cadence line, try to collect data from other lines
  if (validWattages.empty()) {
    for (int k = 0; k < POWERTABLE_CAD_SIZE; ++k) {
      if (k == cadIndex) continue;
      for (int j = 0; j < POWERTABLE_WATT_SIZE; ++j) {
        if (this->tableRow[k].tableEntry[j].targetPosition != INT_MIN) {
          validWattages.push_back(j * POWERTABLE_WATT_INCREMENT);
          validPositions.push_back(this->tableRow[k].tableEntry[j].targetPosition);
        }
      }
    }
  }

  // Interpolate or extrapolate if necessary
  if (validWattages.size() >= 2) {
    // Check if we have enough data in the target cadence line
    std::vector<float> cadPositions;
    for (int j = 0; j < POWERTABLE_WATT_SIZE; ++j) {
      if (this->tableRow[cadIndex].tableEntry[j].targetPosition == INT_MIN) {
        float extrapolatedValue = INT_MIN;
        std::vector<std::pair<int, float>> surroundingPositions;

        for (int k = 0; k < POWERTABLE_CAD_SIZE; ++k) {
          if (k == cadIndex) continue;
          if (this->tableRow[k].tableEntry[j].targetPosition != INT_MIN) {
            surroundingPositions.push_back(std::make_pair(k, this->tableRow[k].tableEntry[j].targetPosition));
          }
        }

        if (surroundingPositions.size() >= 2) {
          // Sort surrounding positions by their cadence index distance to the target cadence
          std::sort(surroundingPositions.begin(), surroundingPositions.end(),
                    [cadIndex](const std::pair<int, float>& a, const std::pair<int, float>& b) { return abs(a.first - cadIndex) < abs(b.first - cadIndex); });

          // Use the closest two surrounding positions to perform linear interpolation
          float cad1 = surroundingPositions[0].first * POWERTABLE_CAD_INCREMENT + MINIMUM_TABLE_CAD;
          float pos1 = surroundingPositions[0].second;
          float cad2 = surroundingPositions[1].first * POWERTABLE_CAD_INCREMENT + MINIMUM_TABLE_CAD;
          float pos2 = surroundingPositions[1].second;

          extrapolatedValue = cadLinearFit(watts, cad1, pos1, cad2, pos2, cad);
        }

        cadPositions.push_back(extrapolatedValue);
      } else {
        cadPositions.push_back(this->tableRow[cadIndex].tableEntry[j].targetPosition);
      }
    }

    validWattages.clear();
    validPositions.clear();

    for (int j = 0; j < POWERTABLE_WATT_SIZE; ++j) {
      if (cadPositions[j] != INT_MIN) {
        validWattages.push_back(j * POWERTABLE_WATT_INCREMENT);
        validPositions.push_back(cadPositions[j]);
      }
    }

    // Ensure unique data points are used for interpolation
    std::vector<std::pair<float, float>> uniquePoints;
    for (size_t idx = 0; idx < validWattages.size(); ++idx) {
      uniquePoints.emplace_back(validWattages[idx], validPositions[idx]);
    }

    std::sort(uniquePoints.begin(), uniquePoints.end());
    uniquePoints.erase(std::unique(uniquePoints.begin(), uniquePoints.end()), uniquePoints.end());

    validWattages.clear();
    validPositions.clear();

    for (const auto& point : uniquePoints) {
      validWattages.push_back(point.first);
      validPositions.push_back(point.second);
    }

    if (watts < validWattages.front()) {
      SS2K_LOG(POWERTABLE_LOG_TAG, "Using linear interpolation for low wattage: %d", watts);
      SS2K_LOG(POWERTABLE_LOG_TAG, "Valid wattages: %f, %f", validWattages.front(), validWattages[1]);
      SS2K_LOG(POWERTABLE_LOG_TAG, "Valid positions: %f, %f", validPositions.front(), validPositions[1]);
      return linearFit({validWattages.front(), validWattages[1]}, {validPositions.front(), validPositions[1]}, watts) * 100;
    }
    if (watts > validWattages.back()) {
      SS2K_LOG(POWERTABLE_LOG_TAG, "Using linear interpolation for high wattage: %d", watts);
      SS2K_LOG(POWERTABLE_LOG_TAG, "Valid wattages: %f, %f", validWattages[validWattages.size() - 2], validWattages.back());
      SS2K_LOG(POWERTABLE_LOG_TAG, "Valid positions: %f, %f", validPositions[validPositions.size() - 2], validPositions.back());
      return linearFit({validWattages[validWattages.size() - 2], validWattages.back()}, {validPositions[validPositions.size() - 2], validPositions.back()}, watts) * 100;
    }

    if (validWattages.size() == 2) {
      SS2K_LOG(POWERTABLE_LOG_TAG, "Using linear interpolation within valid range for wattage: %d", watts);
      SS2K_LOG(POWERTABLE_LOG_TAG, "Valid wattages: ");
      for (const auto& vw : validWattages) SS2K_LOG(POWERTABLE_LOG_TAG, "%f ", vw);
      SS2K_LOG(POWERTABLE_LOG_TAG, "Valid positions: ");
      for (const auto& vp : validPositions) SS2K_LOG(POWERTABLE_LOG_TAG, "%f ", vp);
      return linearFit(validWattages, validPositions, watts) * 100;
    } else {
      SS2K_LOG(POWERTABLE_LOG_TAG, "Using quadratic interpolation within valid range for wattage: %d", watts);
      SS2K_LOG(POWERTABLE_LOG_TAG, "Valid wattages: ");
      for (const auto& vw : validWattages) SS2K_LOG(POWERTABLE_LOG_TAG, "%f ", vw);
      SS2K_LOG(POWERTABLE_LOG_TAG, "Valid positions: ");
      for (const auto& vp : validPositions) SS2K_LOG(POWERTABLE_LOG_TAG, "%f ", vp);
      return quadraticFit(validWattages, validPositions, watts) * 100;
    }
  }

  SS2K_LOG(POWERTABLE_LOG_TAG, "Insufficient data to calculate position.");
  return RETURN_ERROR;
}

bool PowerTable::_manageSaveState() {
  // Check if the table has been loaded in this session
  if (!this->_hasBeenLoadedThisSession) {
    SS2K_LOG(POWERTABLE_LOG_TAG, "Loading Power Table....");
    File file = LittleFS.open(POWER_TABLE_FILENAME, FILE_READ);
    if (!file) {
      SS2K_LOG(POWERTABLE_LOG_TAG, "Failed to Load Power Table.");
      file.close();
      this->_save();
      return false;
    }

    // Read version and size
    int version;
    file.read((uint8_t*)&version, sizeof(version));
    if (version != TABLE_VERSION) {
      SS2K_LOG(POWERTABLE_LOG_TAG, "Expected power table version %d, found version %d", TABLE_VERSION, version);
      file.close();
      this->_save();
      return false;
    }

    SS2K_LOG(POWERTABLE_LOG_TAG, "Loading power table version %d", version);

    int size;
    file.read((uint8_t*)&size, sizeof(size));

    // Initialize a counter for reliable positions
    int reliablePositions = 0;
    std::vector<int> offsetDifferences;
    int activeReliability = 0;
    int savedReliability  = 0;
    // Check if we have at least 3 reliable positions in the active table in order to determine a reliable offset to load the saved table
    for (int i = 0; i < POWERTABLE_CAD_SIZE; i++) {
      for (int j = 0; j < POWERTABLE_WATT_SIZE; j++) {
        int savedTargetPosition = INT_MIN;
        int savedReadings       = 0;
        file.read((uint8_t*)&savedTargetPosition, sizeof(savedTargetPosition));
        file.read((uint8_t*)&savedReadings, sizeof(savedReadings));
        if ((this->tableRow[i].tableEntry[j].targetPosition != INT_MIN) && (this->tableRow[i].tableEntry[j].readings > 3) && (savedReadings > 0)) {
          reliablePositions++;
        }
        if (this->tableRow[i].tableEntry[j].targetPosition != INT_MIN) {
          activeReliability += this->tableRow[i].tableEntry[j].readings;
        }
        if (savedTargetPosition != INT_MIN) {
          savedReliability += savedReadings;
        }
      }
    }
    if (activeReliability > savedReliability) {  // Is the data we are working with better than the saved file?
      SS2K_LOG(POWERTABLE_LOG_TAG, "Active table had a reliability of %d, vs %d for the saved file. Overwriting save.", activeReliability, savedReliability);
      file.close();
      this->_save();
    } else if (reliablePositions < MINIMUM_RELIABLE_POSITIONS) {  // Do we have enough active data in order to calculate an offset when we load the new table?
      SS2K_LOG(POWERTABLE_LOG_TAG, "Not enough reliable positions to load the Power Table.");
      file.close();
      return false;
    } else {  // continue loading
      // we will recalculate this again later as we find offsets
      reliablePositions = 0;
    }
    file.close();
    file = LittleFS.open(POWER_TABLE_FILENAME, FILE_READ);
    if (!file) {
      SS2K_LOG(POWERTABLE_LOG_TAG, "Failed to Load Power Table.");
      file.close();
      this->_save();
      return false;
    }

    // get these reads done, so that we're in the right position to read the data from the file.
    file.read((uint8_t*)&version, sizeof(version));
    file.read((uint8_t*)&size, sizeof(size));

    // Read table entries and calculate offsets
    for (int i = 0; i < POWERTABLE_CAD_SIZE; i++) {
      for (int j = 0; j < POWERTABLE_WATT_SIZE; j++) {
        int savedTargetPosition = INT_MIN;
        int savedReadings       = 0;
        file.read((uint8_t*)&savedTargetPosition, sizeof(savedTargetPosition));
        file.read((uint8_t*)&savedReadings, sizeof(savedReadings));
        if ((this->tableRow[i].tableEntry[j].targetPosition != INT_MIN) && (savedTargetPosition != INT_MIN) && (savedReadings > 0)) {
          int offset = this->tableRow[i].tableEntry[j].targetPosition - savedTargetPosition;
          offsetDifferences.push_back(offset);
          reliablePositions++;
        }
        this->tableRow[i].tableEntry[j].targetPosition = savedTargetPosition;
        this->tableRow[i].tableEntry[j].readings       = savedReadings;
      }
    }

    file.close();

    int totalOffset = 0;
    for (int offset : offsetDifferences) {
      totalOffset += offset;
    }
    int averageOffset = totalOffset / reliablePositions;

    // Apply the offset to all loaded positions except for INT_MIN values
    for (int i = 0; i < POWERTABLE_CAD_SIZE; i++) {
      for (int j = 0; j < POWERTABLE_WATT_SIZE; j++) {
        if (this->tableRow[i].tableEntry[j].targetPosition != INT_MIN) {
          this->tableRow[i].tableEntry[j].targetPosition += averageOffset;
        }
      }
    }
    // set the flag so it isn't loaded again this session.
    this->_hasBeenLoadedThisSession = true;
    SS2K_LOG(POWERTABLE_LOG_TAG, "Power Table loaded with an offset of %d.", averageOffset);
  }

  // Implement saving on a timer
  if ((millis() - lastSaveTime) > POWER_TABLE_SAVE_INTERVAL) {
    this->_save();
    lastSaveTime = millis();
  }
  return true;
}

bool PowerTable::_save() {
  // Delete existing file to avoid appending
  LittleFS.remove(POWER_TABLE_FILENAME);

  // Open file for writing
  SS2K_LOG(POWERTABLE_LOG_TAG, "Writing File: %s", POWER_TABLE_FILENAME);
  File file = LittleFS.open(POWER_TABLE_FILENAME, FILE_WRITE);
  if (!file) {
    SS2K_LOG(POWERTABLE_LOG_TAG, "Failed to create file");
    return false;
  }

  // Write version and size
  int version = TABLE_VERSION;
  file.write((uint8_t*)&version, sizeof(version));

  int size = getEntries();
  file.write((uint8_t*)&size, sizeof(size));

  // Write table entries
  for (int i = 0; i < POWERTABLE_CAD_SIZE; i++) {
    for (int j = 0; j < POWERTABLE_WATT_SIZE; j++) {
      file.write((uint8_t*)&this->tableRow[i].tableEntry[j].targetPosition, sizeof(this->tableRow[i].tableEntry[j].targetPosition));
      file.write((uint8_t*)&this->tableRow[i].tableEntry[j].readings, sizeof(this->tableRow[i].tableEntry[j].readings));
    }
  }

  // Close the file
  file.close();
  lastSaveTime                    = millis();
  this->_hasBeenLoadedThisSession = true;
  return true;  // return successful
}

void PowerTable::toLog() {
  int maxLen = 4;
  // Find the longest integer to dynamically size the table
  for (int i = 0; i < POWERTABLE_CAD_SIZE; i++) {
    for (int j = 0; j < POWERTABLE_WATT_SIZE; j++) {
      if (this->tableRow[i].tableEntry[j].targetPosition == INT_MIN) {
        continue;
      }
      int len = snprintf(nullptr, 0, "%d", this->tableRow[i].tableEntry[j].targetPosition);
      if (maxLen < len) {
        maxLen = len;
      }
    }
  }

  char buffer[maxLen + 2];  // Buffer for formatting
  // Print header row
  String headerRow = "CAD\\W ";
  for (int j = 0; j < POWERTABLE_WATT_SIZE; j++) {
    snprintf(buffer, sizeof(buffer), "%*d", maxLen, j * POWERTABLE_WATT_INCREMENT);
    headerRow += String(" | ") + buffer;
  }
  SS2K_LOG(POWERTABLE_LOG_TAG, "%s", headerRow.c_str());

  // Print each row of the table
  for (int i = 0; i < POWERTABLE_CAD_SIZE; i++) {
    String logString = String(i * POWERTABLE_CAD_INCREMENT + MINIMUM_TABLE_CAD) + " rpm";
    for (int j = 0; j < POWERTABLE_WATT_SIZE; j++) {
      int targetPosition = this->tableRow[i].tableEntry[j].targetPosition;
      if (targetPosition == INT_MIN) {
        snprintf(buffer, sizeof(buffer), "%*s", maxLen, " ");
      } else {
        snprintf(buffer, sizeof(buffer), "%*d", maxLen, targetPosition);
      }
      logString += String(" | ") + buffer;
    }
    SS2K_LOG(POWERTABLE_LOG_TAG, "%s", logString.c_str());
  }
}

int PowerTable::getEntries() {
  int ret = 0;
  for (int i = 0; i < POWERTABLE_CAD_SIZE; i++) {
    for (int j = 0; j < POWERTABLE_WATT_SIZE; j++) {
      if (this->tableRow[i].tableEntry[j].readings > 0) {
        ret++;
      }
    }
  }
  return ret;
}

// compute position for resistance control mode
void ErgMode::computeResistance() {
  static int stepChangePerResistance = userConfig->getShiftStep();
  static Measurement oldResistance;

  if (rtConfig->resistance.getTimestamp() == oldResistance.getTimestamp()) {
    SS2K_LOG(ERG_MODE_LOG_TAG, "Resistance previously processed.");
    return;
  }

  int newSetPoint = rtConfig->resistance.getTarget();
  int actualDelta = rtConfig->resistance.getTarget() - rtConfig->resistance.getValue();
  rtConfig->setTargetIncline(rtConfig->getTargetIncline() + (100 * actualDelta));
  if (actualDelta == 0) {
    rtConfig->setTargetIncline(rtConfig->getCurrentIncline());
  }
  oldResistance = rtConfig->resistance;
}
// as a note, Trainer Road sends 50w target whenever the app is connected.
void ErgMode::computeErg() {
  Measurement newWatts = rtConfig->watts;
  int newCadence       = rtConfig->cad.getValue();

  // check for new torque value or new set point, if watts < 10 treat as faulty
  if ((this->watts.getTimestamp() == newWatts.getTimestamp() && this->setPoint == newWatts.getTarget()) || newWatts.getValue() < 10) {
    SS2K_LOGW(ERG_MODE_LOG_TAG, "Watts previously processed.");
    return;
  }

  // set minimum set point to minimum bike watts if app sends set point lower than minimum bike watts.
  if (newWatts.getTarget() < userConfig->getMinWatts()) {
    SS2K_LOG(ERG_MODE_LOG_TAG, "ERG Target Below Minumum Value.");
    newWatts.setTarget(userConfig->getMinWatts());
  }

  bool isUserSpinning = this->_userIsSpinning(newCadence, rtConfig->getCurrentIncline());
  if (!isUserSpinning) {
    SS2K_LOG(ERG_MODE_LOG_TAG, "ERG Mode but no User Spin");
    return;
  }

  // SetPoint changed
  if (abs(this->setPoint - newWatts.getTarget()) > 20) {
    _setPointChangeState(newCadence, newWatts);
    return;
  }

  // Setpoint unchanged
  _inSetpointState(newCadence, newWatts);
}

void ErgMode::_setPointChangeState(int newCadence, Measurement& newWatts) {
  int32_t tableResult = powerTable->lookup(newWatts.getTarget(), newCadence);
  if (tableResult == RETURN_ERROR) {
    int wattChange  = newWatts.getTarget() - newWatts.getValue();
    float deviation = ((float)wattChange * 100.0) / ((float)newWatts.getTarget());
    float factor    = abs(deviation) > 10 ? userConfig->getERGSensitivity() : userConfig->getERGSensitivity() / 2;
    tableResult     = rtConfig->getCurrentIncline() + (wattChange * factor);
  }

  SS2K_LOG(ERG_MODE_LOG_TAG, "SetPoint changed:%dw PowerTable Result: %d", newWatts.getTarget(), tableResult);
  _updateValues(newCadence, newWatts, tableResult);

  int i = 0;
  while (rtConfig->getTargetIncline() != rtConfig->getCurrentIncline()) {  // wait while the knob moves to target position.
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (i > 50) {  // failsafe for infinite loop
      SS2K_LOG(ERG_MODE_LOG_TAG, "Stepper didn't reach target position");
      break;
    }
    i++;
  }

  vTaskDelay((ERG_MODE_DELAY * 3) / portTICK_PERIOD_MS);  // Wait for power meter to register new watts
}

void ErgMode::_inSetpointState(int newCadence, Measurement& newWatts) {
  int watts = newWatts.getValue();

  int wattChange  = newWatts.getTarget() - watts;  // setpoint_form_trainer - current_torque => Amount to increase or decrease incline
  float deviation = ((float)wattChange * 100.0) / ((float)newWatts.getTarget());

  float factor     = abs(deviation) > 10 ? userConfig->getERGSensitivity() : userConfig->getERGSensitivity() / 2;
  float newIncline = rtConfig->getCurrentIncline() + (wattChange * factor);

  _updateValues(newCadence, newWatts, newIncline);
}

void ErgMode::_updateValues(int newCadence, Measurement& newWatts, float newIncline) {
  rtConfig->setTargetIncline(newIncline);
  _writeLog(rtConfig->getCurrentIncline(), newIncline, this->setPoint, newWatts.getTarget(), this->watts.getValue(), newWatts.getValue(), this->cadence, newCadence);

  this->watts    = newWatts;
  this->setPoint = newWatts.getTarget();
  this->cadence  = newCadence;
}

bool ErgMode::_userIsSpinning(int cadence, float incline) {
  if (cadence <= MIN_ERG_CADENCE) {
    if (!this->engineStopped) {                               // Test so motor stop command only happens once.
      ss2k->motorStop();                                      // release tension
      rtConfig->setTargetIncline(incline - WATTS_PER_SHIFT);  // release incline
      this->engineStopped = true;
    }
    return false;  // Cadence too low, nothing to do here
  }

  this->engineStopped = false;
  return true;
}

void ErgMode::_writeLogHeader() {
  SS2K_LOGW(ERG_MODE_LOG_CSV_TAG, "current incline;new incline;current setpoint;new setpoint;current watts;new watts;current cadence;new cadence;");
}

void ErgMode::_writeLog(float currentIncline, float newIncline, int currentSetPoint, int newSetPoint, int currentWatts, int newWatts, int currentCadence, int newCadence) {
  SS2K_LOGW(ERG_MODE_LOG_CSV_TAG, "%d;%.2f;%.2f;%d;%d;%d;%d;%d", currentIncline, newIncline, currentSetPoint, newSetPoint, currentWatts, newWatts, currentCadence, newCadence);
}
