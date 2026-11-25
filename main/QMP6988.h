#pragma once

#ifdef SENSORS

namespace QMP6988
{
    void ConnectToWifi(bool updating);
    void SerialReport();
    bool SendDiscovery();
    void Setup();
    void Loop();
}

#endif
