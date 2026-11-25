#ifdef SENSORS

#include "QMP6988.h"
#include "QMP6988Math.h"
#include "I2C.h"
#include "Settings.h"
#include "defaults.h"
#include "globals.h"
#include "mqtt.h"

namespace QMP6988
{
    namespace
    {
        constexpr uint8_t REG_CHIP_ID = 0xD1;
        constexpr uint8_t REG_RESET = 0xE0;
        constexpr uint8_t REG_STATUS = 0xF3;
        constexpr uint8_t REG_CONTROL = 0xF4;
        constexpr uint8_t REG_FILTER = 0xF1;
        constexpr uint8_t REG_CALIBRATION = 0xA0;
        constexpr uint8_t REG_DATA = 0xF7;

        constexpr uint8_t CHIP_ID = 0x5C;
        constexpr uint8_t RESET_COMMAND = 0xE6;
        constexpr uint8_t STATUS_MEASURING = 0x08;

        // Temperature x1, pressure x8, sleep mode.
        constexpr uint8_t CONTROL_SLEEP = 0x30;
        constexpr uint8_t CONTROL_FORCED = 0x31;

        constexpr unsigned long SENSOR_INTERVAL = 60000;
        constexpr unsigned long MEASUREMENT_DELAY = 20;
        constexpr unsigned long MEASUREMENT_TIMEOUT = 100;
        constexpr unsigned long POLL_INTERVAL = 10;

        int configuredBus = 0;
        std::string configuredAddress;

        int bus = 0;
        uint8_t address = 0;

        bool initialized = false;
        bool measurementPending = false;
        bool firstMeasurement = true;

        unsigned long lastMeasurement = 0;
        unsigned long measurementStarted = 0;
        unsigned long lastPoll = 0;

        QMP6988Math::Calibration calibration{};
    }

    void ConnectToWifi(bool updating)
    {
        configuredBus = I2C::busSetting("QMP6988_I2c_Bus");

        configuredAddress = Settings::string(
            "QMP6988_I2c",
            "",
            "I2C address (optional, default 0x70)"
        );
    }

    void SerialReport()
    {
        if (!I2C::started(configuredBus)) return;

        Log.print("QMP6988:      ");
        Log.println(
            configuredAddress + " on bus " + toStr(configuredBus)
        );
    }

    void Setup()
    {
        initialized = false;
        measurementPending = false;
        firstMeasurement = true;

        if (!I2C::started(configuredBus)) return;

        if (!I2C::pickAddress(
                configuredAddress, {0x70, 0x56}, address)) {
            Log.println("[QMP6988] Invalid I2C address");
            return;
        }

        bus = configuredBus;

        uint8_t id = 0;

        if (!I2C::readReg8(bus, address, REG_CHIP_ID, id)
            || id != CHIP_ID) {
            Log.println("[QMP6988] Sensor not found");
            return;
        }

        // A reset may interrupt the I2C acknowledgement.
        // Verify communication again before configuration.
        I2C::writeReg(bus, address, REG_RESET, RESET_COMMAND);
        delay(20);

        if (!I2C::readReg8(bus, address, REG_CHIP_ID, id)
            || id != CHIP_ID) {
            Log.println("[QMP6988] Sensor unavailable after reset");
            return;
        }

        uint8_t data[25];

        if (!I2C::readReg(
                bus, address, REG_CALIBRATION, data, sizeof(data))
            || !QMP6988Math::decodeCalibration(data, calibration)) {
            Log.println("[QMP6988] Invalid calibration data");
            return;
        }

        if (!I2C::writeReg(
                bus, address, REG_CONTROL, CONTROL_SLEEP)
            || !I2C::writeReg(
                bus, address, REG_FILTER, 0x00)) {
            Log.println("[QMP6988] Configuration failed");
            return;
        }

        initialized = true;

        Log.println("[QMP6988] Initialized");
    }

    void Loop()
    {
        if (!initialized || !I2C::started(bus)) return;

        const unsigned long now = millis();

        if (!measurementPending) {
            if (!firstMeasurement
                && now - lastMeasurement < SENSOR_INTERVAL)
                return;

            firstMeasurement = false;
            lastMeasurement = now;

            if (!I2C::writeReg(
                    bus, address, REG_CONTROL, CONTROL_FORCED)) {
                Log.println("[QMP6988] Measurement start failed");
                return;
            }

            measurementStarted = millis();
            lastPoll = measurementStarted;
            measurementPending = true;

            return;
        }

        if (now - measurementStarted < MEASUREMENT_DELAY)
            return;

        if (now - lastPoll < POLL_INTERVAL)
            return;

        lastPoll = now;

        uint8_t status = 0;

        if (!I2C::readReg8(bus, address, REG_STATUS, status)) {
            measurementPending = false;
            Log.println("[QMP6988] Status read failed");
            return;
        }

        if (status & STATUS_MEASURING) {
            if (now - measurementStarted >= MEASUREMENT_TIMEOUT) {
                measurementPending = false;
                Log.println("[QMP6988] Measurement timeout");
            }

            return;
        }

        measurementPending = false;

        uint8_t data[6];
        QMP6988Math::Reading reading{};

        if (!I2C::readReg(
                bus, address, REG_DATA, data, sizeof(data))
            || !QMP6988Math::decodeMeasurement(data, calibration, reading)) {
            Log.println("[QMP6988] Invalid measurement");
            return;
        }

        if (!QMP6988Math::isPlausible(reading)) {
            Log.println("[QMP6988] Measurement outside plausible range");
            return;
        }

        pub(
            (roomsTopic + "/qmp6988_temperature").c_str(),
            0, 1, toStr(reading.temperature).c_str()
        );

        pub(
            (roomsTopic + "/qmp6988_pressure").c_str(),
            0, 1, toStr(reading.pressure).c_str()
        );
    }

    bool SendDiscovery()
    {
        if (configuredBus < 1) return true;

        return sendSensorDiscovery(
                   "QMP6988 Temperature", EC_NONE, "temperature", "°C")
            && sendSensorDiscovery(
                   "QMP6988 Pressure", EC_NONE, "pressure", "hPa");
    }
}

#endif
