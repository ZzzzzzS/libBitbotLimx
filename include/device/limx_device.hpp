#pragma once

#include "bitbot_kernel/device/device.hpp"
#include "../limxsdk/apibase.h"

namespace bitbot
{
    class LimxBus;

    enum class LimxDeviceType : uint32_t
    {
        LIMX_DEVICE = 1000,
        LIMX_JOINT,
        LIMX_IMU,
        LIMX_LED
    };

    class LimxDevice : public Device
    {
    public:
        LimxDevice(const pugi::xml_node& device_node)
            : Device(device_node) {
        }
        virtual ~LimxDevice() = default;
        void UpdateRuntimeData() = 0;
    };
};