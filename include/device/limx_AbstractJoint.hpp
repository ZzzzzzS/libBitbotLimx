#pragma once
#include "limx_device.hpp"
#include "../limxsdk/apibase.h"
#include "../limxsdk/datatypes.h"
#include "atomic"

namespace bitbot
{
    class LimxAbstractJoint : public LimxDevice
    {
        friend class LimxBus;
    public:
        LimxAbstractJoint(const pugi::xml_node& joint_node) :LimxDevice(joint_node) {
            this->basic_type_ = static_cast<uint32_t>(BasicDeviceType::MOTOR);
            this->type_ = static_cast<uint32_t>(LimxDeviceType::Abstract_JOINT);
            this->monitor_header_.headers = { "NO_DATA" };
            this->monitor_data_.resize(monitor_header_.headers.size());
        }
        virtual ~LimxAbstractJoint() = default;

    public: //interface
        virtual float GetActualPosition() { return 0; }
        virtual float GetActualVelocity() { return 0; }
        virtual float GetActualTorque() { return 0; }
        virtual float GetTargetPosition() { return 0; }
        virtual float GetTargetVelocity() { return 0; }
        virtual float GetTargetTorque() { return 0; }
        virtual void SetTargetPosition(float pos) {}
        virtual void SetTargetVelocity(float vel) {}
        virtual void SetTargetTorque(float torque) {}
    private:
        void UpdateRuntimeData() {}
    };
};