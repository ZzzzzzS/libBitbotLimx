#pragma once
#include "bitbot_kernel/bus/bus_manager.hpp"
#include "device/limx_device.hpp"
#include "device/limx_joint.h"
#include "device/limx_imu.h"
#include "device/limx_LED.h"
#include "limxsdk/apibase.h"
#include "atomic"

namespace bitbot
{
    class LimxBus : public BusManagerTpl<LimxBus, LimxDevice>
    {
    public:
        LimxBus();
        virtual ~LimxBus();
        void Init(limxsdk::ApiBase* DeviceHandle);

        //interface
        void WriteBus();
        void ReadBus();
        void RegisterDevices();
        void PowerOnDevice();
        void SafeTorqueOff();

        std::atomic_bool StateUpdatedFlag;
        std::atomic_bool ErrorFlag;
        std::atomic_bool ImuUpdatedFlag;
        std::atomic_bool SystemCalibratedFlag;

    private:
        using JointStateCallbackFunction = std::function<void(const limxsdk::RobotStateConstPtr&)>;
        using ImuStateCallbackFunction = std::function<void(const limxsdk::ImuDataConstPtr&)>;
        using DiagnosticInfoCallbackFunction = std::function<void(const limxsdk::DiagnosticValueConstPtr&)>;
        void JointStateCallback(const limxsdk::RobotStateConstPtr& state);
        void ImuStateCallback(const limxsdk::ImuDataConstPtr& state);
        void DiagnosticInfoCallback(const limxsdk::DiagnosticValueConstPtr& state);


    private:
        limxsdk::RobotCmd RobotCmd__;
        limxsdk::ApiBase* DeviceHandle__;
        std::vector<LimxJoint*> Joints__;
        LimxImu* Imu__;
        size_t WriteBusCnt__;
        //TODO: add led device
    };
};