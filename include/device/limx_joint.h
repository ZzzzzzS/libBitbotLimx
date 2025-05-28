#pragma once
#include "limx_device.hpp"
#include "../limxsdk/apibase.h"
#include "../limxsdk/datatypes.h"
#include "atomic"
#include "limx_AbstractJoint.hpp"

namespace bitbot
{
    class LimxBus;

    enum class LimxJointMode : uint8_t
    {
        CST = 0,
        CSV = 1,
        CSP = 2,
    };

    struct LimxJointState
    {
        std::atomic<float> q;
        std::atomic<float> dq;
        std::atomic<float> tau;
        std::atomic<uint64_t> stamp;
        LimxJointState(float q, float dq, float tau, uint64_t stamp)
            : q(q), dq(dq), tau(tau), stamp(stamp) {
        }
    };

    struct LimxJointCmd
    {
        std::atomic<uint8_t> mode;
        std::atomic<float> q;
        std::atomic<float> dq;
        std::atomic<float> tau;
        std::atomic<float> Kp;
        std::atomic<float> Kd;
        std::atomic<uint64_t> stamp;

        LimxJointCmd(uint8_t mode, float q, float dq, float tau, float Kp, float Kd, uint64_t stamp)
            : mode(mode), q(q), dq(dq), tau(tau), Kp(Kp), Kd(Kd), stamp(stamp) {
        }
    };

    class LimxJoint : public LimxAbstractJoint
    {
        friend class LimxBus;
    public:
        LimxJoint(const pugi::xml_node& joint_node);
        virtual ~LimxJoint() = default;

    public: //interface
        void Enable();
        void Disable();
        bool isEnable();
        float GetActualPosition();
        float GetActualVelocity();
        float GetActualTorque();
        float GetTargetPosition();
        float GetTargetVelocity();
        float GetTargetTorque();
        void SetTargetPosition(float pos);
        void SetTargetVelocity(float vel);
        void SetTargetTorque(float torque);
        void SetMode(LimxJointMode mode);
        LimxJointMode GetMode();
        void setKp(float kp);
        void setKd(float kd);
        float getKp();
        float getKd();

    private:
        void UpdateRuntimeData();
        void UpdateJointState(const LimxJointState& state);
        void UpdateJointCmd(LimxJointCmd& cmd);

    private:
        LimxJointState JointState__;
        LimxJointCmd JointCmd__;
        LimxJointMode JointMode__;
        //bool Enable__;
        std::atomic_bool Enable__;
        std::atomic_bool EnableInCfg__;
        std::atomic_bool FirstReceive__;
        int MotorDirection__;
        int TorqueDirection__;
    };
};