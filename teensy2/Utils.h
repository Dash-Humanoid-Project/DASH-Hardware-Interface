#pragma once


// Bound v to [lo, hi]. Used to enforce per-joint position/velocity/torque
// limits on both the PC (Leg.cpp) and Teensy (teensy.ino) sides.
inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// CRC-8 polynomial (Dallas/Maxim)
const uint8_t CRC8_POLYNOMIAL = 0x31;

// Calculate CRC-8 checksum
inline uint8_t calculate_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc <<= 1;
        }
    }
    return crc;
}
