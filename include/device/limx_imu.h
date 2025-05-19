#pragma once

#include "limx_device.hpp"
#include "../limxsdk/apibase.h"
#include "../limxsdk/datatypes.h"
#include "atomic"
#include <fstream>

namespace bitbot
{
    class LimxBus;

    struct ImuRuntimeData
    {

        std::atomic<float> roll = 0;
        std::atomic<float> pitch = 0;
        std::atomic<float> yaw = 0;
        std::atomic<float> a_x = 0;
        std::atomic<float> a_y = 0;
        std::atomic<float> a_z = 0;
        std::atomic<float> w_x = 0;
        std::atomic<float> w_y = 0;
        std::atomic<float> w_z = 0;
    };

    struct ImuData
    {
        ImuRuntimeData runtime;
    };

    class LimxImu : public LimxDevice
    {
        friend class LimxBus;
    public:
        LimxImu(const pugi::xml_node& imu_node);

        virtual ~LimxImu() = default;

        float GetRoll();
        float GetPitch();
        float GetYaw();
        float GetAccX();
        float GetAccY();
        float GetAccZ();
        float GetGyroX();
        float GetGyroY();
        float GetGyroZ();

    private:
        void UpdateRuntimeData();
        void UpdateImuData(limxsdk::ImuDataConstPtr ImuDataPtr);

        std::array<float, 3> cvtEuler(const float* quat);

    private:
        ImuData imu_data_;
    };
}