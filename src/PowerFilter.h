#pragma once
#include <deque>
#include <numeric>
#include <algorithm>

// Simple smoothing filter for power and cadence values
class PowerFilter {
public:
    PowerFilter(size_t window = 3) : windowSize(window) {}

    void addCadence(int cad) {
        cadenceBuffer.push_back(cad);
        if (cadenceBuffer.size() > windowSize) {
            cadenceBuffer.pop_front();
        }
    }

    void addWatts(int watts) {
        wattsBuffer.push_back(watts);
        if (wattsBuffer.size() > windowSize) {
            wattsBuffer.pop_front();
        }
    }

    int getFilteredCadence() const {
        if (cadenceBuffer.empty()) return 0;
        return std::accumulate(cadenceBuffer.begin(), cadenceBuffer.end(), 0) / cadenceBuffer.size();
    }

    int getFilteredWatts() const {
        if (wattsBuffer.empty()) return 0;
        return std::accumulate(wattsBuffer.begin(), wattsBuffer.end(), 0) / wattsBuffer.size();
    }

private:
    size_t windowSize;
    std::deque<int> cadenceBuffer;
    std::deque<int> wattsBuffer;
};