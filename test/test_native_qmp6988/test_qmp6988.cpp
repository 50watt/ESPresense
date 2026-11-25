// Host-side tests for QMP6988 calibration and compensation.

#include <unity.h>
#include <cmath>
#include <random>

#include "QMP6988Math.h"

using namespace QMP6988Math;

void test_signed_coefficients()
{
    TEST_ASSERT_EQUAL_INT32(32767, signExtend(0x7FFF, 16));
    TEST_ASSERT_EQUAL_INT32(-32768, signExtend(0x8000, 16));
    TEST_ASSERT_EQUAL_INT32(-1, signExtend(0xFFFF, 16));

    TEST_ASSERT_EQUAL_INT32(524287, signExtend(0x7FFFF, 20));
    TEST_ASSERT_EQUAL_INT32(-524288, signExtend(0x80000, 20));
    TEST_ASSERT_EQUAL_INT32(-1, signExtend(0xFFFFF, 20));
}

void test_invalid_calibration()
{
    Calibration c{};
    uint8_t data[25]{};

    TEST_ASSERT_FALSE(decodeCalibration(data, c));

    for (auto& b : data) b = 0xFF;

    TEST_ASSERT_FALSE(decodeCalibration(data, c));
}

void test_pressure_unit_conversion()
{
    Calibration c{};
    Reading r{};

    uint8_t calibration[25]{};
    calibration[0] = 0x27;
    calibration[1] = 0x10;

    uint8_t measurement[6] = {
        0x80, 0x00, 0x00,
        0x80, 0x00, 0x00
    };

    TEST_ASSERT_TRUE(decodeCalibration(calibration, c));
    TEST_ASSERT_TRUE(decodeMeasurement(measurement, c, r));

    TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, r.temperature);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 100.0F, r.pressure);
}

void test_temperature_compensation()
{
    Calibration c{};
    Reading r{};

    uint8_t calibration[25]{};
    calibration[0] = 0x27;
    calibration[1] = 0x10;
    calibration[18] = 0x19;

    uint8_t measurement[6] = {
        0x80, 0x00, 0x00,
        0x80, 0x00, 0x00
    };

    TEST_ASSERT_TRUE(decodeCalibration(calibration, c));
    TEST_ASSERT_TRUE(decodeMeasurement(measurement, c, r));

    TEST_ASSERT_FLOAT_WITHIN(0.001F, 25.0F, r.temperature);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 106.404375F, r.pressure);
}

void test_nonzero_raw_offsets()
{
    Calibration c{};
    Reading r{};

    uint8_t calibration[25]{};
    calibration[0] = 0x27;
    calibration[1] = 0x10;
    calibration[18] = 0x19;

    // Synthetic vector: dp = 2500000, dt = -100000.
    uint8_t measurement[6] = {
        0xA6, 0x25, 0xA0,
        0x7E, 0x79, 0x60
    };

    TEST_ASSERT_TRUE(decodeCalibration(calibration, c));
    TEST_ASSERT_TRUE(decodeMeasurement(measurement, c, r));

    TEST_ASSERT_FLOAT_WITHIN(0.001F, 27.45703125F, r.temperature);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 951.155F, r.pressure);
}

void test_plausibility()
{
    TEST_ASSERT_TRUE(isPlausible({23.5F, 980.0F}));
    TEST_ASSERT_TRUE(isPlausible({-40.0F, 300.0F}));
    TEST_ASSERT_TRUE(isPlausible({85.0F, 1100.0F}));

    TEST_ASSERT_FALSE(isPlausible({-40.1F, 980.0F}));
    TEST_ASSERT_FALSE(isPlausible({85.1F, 980.0F}));
    TEST_ASSERT_FALSE(isPlausible({20.0F, 299.9F}));
    TEST_ASSERT_FALSE(isPlausible({20.0F, 1100.1F}));
    TEST_ASSERT_FALSE(isPlausible({20.0F, 0.0F}));
    TEST_ASSERT_FALSE(isPlausible({NAN, 980.0F}));
    TEST_ASSERT_FALSE(isPlausible({20.0F, NAN}));
    TEST_ASSERT_FALSE(isPlausible({20.0F, INFINITY}));
}

void test_invalid_measurements()
{
    Calibration c{};
    Reading r{};

    uint8_t calibration[25]{};
    calibration[0] = 0x27;
    calibration[1] = 0x10;

    TEST_ASSERT_TRUE(decodeCalibration(calibration, c));

    uint8_t measurement[6]{};

    TEST_ASSERT_FALSE(decodeMeasurement(measurement, c, r));

    for (auto& b : measurement) b = 0xFF;

    TEST_ASSERT_FALSE(decodeMeasurement(measurement, c, r));
}

void test_randomized_measurements()
{
    Calibration c{};
    Reading r{};

    uint8_t calibration[25]{};
    uint8_t measurement[6]{};

    std::mt19937 rng(6988);

    unsigned accepted = 0;
    unsigned rejected = 0;

    for (unsigned i = 0; i < 100000; ++i) {
        for (auto& b : calibration)
            b = static_cast<uint8_t>(rng());

        for (auto& b : measurement)
            b = static_cast<uint8_t>(rng());

        if (!decodeCalibration(calibration, c))
            continue;

        if (decodeMeasurement(measurement, c, r)) {
            TEST_ASSERT_TRUE(std::isfinite(r.temperature));
            TEST_ASSERT_TRUE(std::isfinite(r.pressure));
            TEST_ASSERT_TRUE(r.pressure > 0);
            ++accepted;
        } else {
            ++rejected;
        }
    }

    TEST_ASSERT_TRUE(accepted > 0);
    TEST_ASSERT_TRUE(rejected > 0);
}

int main(int, char**)
{
    UNITY_BEGIN();

    RUN_TEST(test_signed_coefficients);
    RUN_TEST(test_invalid_calibration);
    RUN_TEST(test_pressure_unit_conversion);
    RUN_TEST(test_temperature_compensation);
    RUN_TEST(test_nonzero_raw_offsets);
    RUN_TEST(test_plausibility);
    RUN_TEST(test_invalid_measurements);
    RUN_TEST(test_randomized_measurements);

    return UNITY_END();
}
