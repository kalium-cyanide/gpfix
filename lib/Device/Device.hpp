#pragma once

// ── Выбор устройства на этапе компиляции ─────────────────────
//    -DDEVICE_BIKE  → BikeDevice
//    -DDEVICE_CAR   → CarDevice (по умолчанию)

#if defined(DEVICE_BIKE)
    #include "BikeDevice.hpp"
    using CurrentDevice = BikeDevice;
#else
    #include "CarDevice.hpp"
    using CurrentDevice = CarDevice;
#endif
