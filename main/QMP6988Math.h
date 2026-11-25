// QMP6988 calibration and compensation.
// Adapted for ESPresense on 2026-11-25.
// Reference: M5Stack/M5Unit-ENV, revision e3384a7.
// SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
// SPDX-License-Identifier: MIT
//
// MIT License
//
// Copyright (c) 2021 M5Stack
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <cstdint>
#include <limits>

namespace QMP6988Math
{
    struct Calibration
    {
        int32_t b00, a0;
        int64_t a1, a2;
        int64_t bt1, bt2, bp1, b11;
        int64_t bp2, b12, b21, bp3;
    };

    struct Reading
    {
        float temperature;
        float pressure;
    };

    // Validate compensated values before publishing.
    inline bool isPlausible(const Reading& reading)
    {
        return reading.temperature >= -40.0F
            && reading.temperature <= 85.0F
            && reading.pressure >= 300.0F
            && reading.pressure <= 1100.0F;
    }

    inline int32_t signExtend(uint32_t value, unsigned bits)
    {
        const uint32_t sign = uint32_t{1} << (bits - 1);
        return static_cast<int32_t>(value)
            - ((value & sign) ? static_cast<int32_t>(sign * 2) : 0);
    }

    inline int32_t read16(const uint8_t* data)
    {
        return signExtend(
            (uint32_t(data[0]) << 8) | data[1], 16);
    }

    inline uint32_t read24(const uint8_t* data)
    {
        return (uint32_t(data[0]) << 16)
             | (uint32_t(data[1]) << 8)
             | uint32_t(data[2]);
    }

    inline bool decodeCalibration(
        const uint8_t (&data)[25], Calibration& c)
    {
        bool allZero = true;
        bool allOnes = true;

        for (uint8_t byte : data) {
            allZero &= byte == 0x00;
            allOnes &= byte == 0xFF;
        }

        if (allZero || allOnes) return false;

        c.b00 = signExtend(
            (uint32_t(data[0]) << 12)
            | (uint32_t(data[1]) << 4)
            | (data[24] >> 4), 20);

        c.a0 = signExtend(
            (uint32_t(data[18]) << 12)
            | (uint32_t(data[19]) << 4)
            | (data[24] & 0x0F), 20);

        c.a1  = 3608LL   * read16(data + 20) - 1731677965LL;
        c.a2  = 16889LL  * read16(data + 22) - 87619360LL;
        c.bt1 = 2982LL   * read16(data + 2)  + 107370906LL;
        c.bt2 = 329854LL * read16(data + 4)  + 108083093LL;
        c.bp1 = 19923LL  * read16(data + 6)  + 1133836764LL;
        c.b11 = 2406LL   * read16(data + 8)  + 118215883LL;
        c.bp2 = 3079LL   * read16(data + 10) - 181579595LL;
        c.b12 = 6846LL   * read16(data + 12) + 85590281LL;
        c.b21 = 13836LL  * read16(data + 14) + 79333336LL;
        c.bp3 = 2915LL   * read16(data + 16) + 157155561LL;

        return true;
    }

    // Compensated temperature in units of 1/256 degree Celsius.
    inline int64_t temperature256(
        int32_t dt, const Calibration& c)
    {
        int64_t wk1 = c.a1 * dt;
        int64_t wk2 = (c.a2 * dt) >> 14;

        wk2 = (wk2 * dt) >> 10;
        wk2 = ((wk1 + wk2) / 32767) >> 19;

        return (c.a0 + wk2) >> 4;
    }

    // Compensated pressure in units of 1/16 Pascal.
    inline int64_t pressure16(
        int32_t dp, int16_t tx, const Calibration& c)
    {
        int64_t wk1 = c.bt1 * tx;
        int64_t wk2 = (c.bp1 * dp) >> 5;
        wk1 += wk2;

        wk2 = (c.bt2 * tx) >> 1;
        wk2 = (wk2 * tx) >> 8;
        int64_t wk3 = wk2;

        wk2 = (c.b11 * tx) >> 4;
        wk2 = (wk2 * dp) >> 1;
        wk3 += wk2;

        wk2 = (c.bp2 * dp) >> 13;
        wk2 = (wk2 * dp) >> 1;
        wk3 += wk2;

        wk1 += wk3 >> 14;

        wk2 = c.b12 * tx;
        wk2 = (wk2 * tx) >> 22;
        wk2 = (wk2 * dp) >> 1;
        wk3 = wk2;

        wk2 = (c.b21 * tx) >> 6;
        wk2 = (wk2 * dp) >> 23;
        wk2 = (wk2 * dp) >> 1;
        wk3 += wk2;

        wk2 = (c.bp3 * dp) >> 12;
        wk2 = (wk2 * dp) >> 23;
        wk2 *= dp;
        wk3 += wk2;

        wk1 += wk3 >> 15;
        wk1 /= 32767;
        wk1 >>= 11;
        wk1 += c.b00;

        return wk1;
    }

    inline bool decodeMeasurement(
        const uint8_t (&data)[6],
        const Calibration& c,
        Reading& result)
    {
        const uint32_t rawPressure = read24(data);
        const uint32_t rawTemperature = read24(data + 3);

        if (rawPressure == 0 || rawPressure == 0xFFFFFF
            || rawTemperature == 0 || rawTemperature == 0xFFFFFF)
            return false;

        const int32_t dp =
            static_cast<int32_t>(rawPressure) - 8388608;

        const int32_t dt =
            static_cast<int32_t>(rawTemperature) - 8388608;

        const int64_t tx = temperature256(dt, c);

        if (tx < std::numeric_limits<int16_t>::min()
            || tx > std::numeric_limits<int16_t>::max())
            return false;

        const int64_t pressure =
            pressure16(dp, static_cast<int16_t>(tx), c);

        if (pressure <= 0
            || pressure > std::numeric_limits<int32_t>::max())
            return false;

        result.temperature = static_cast<float>(tx) / 256.0F;
        result.pressure = static_cast<float>(pressure) / 1600.0F;

        return true;
    }
}
