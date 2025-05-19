#include "bus/limx_bus.h"
#include "algorithm"
#include "iostream"

namespace bitbot
{
    static std::atomic_bool isBusFreed;

    LimxBus::LimxBus()
        :DeviceHandle__(nullptr),
        Imu__(nullptr),
        StateUpdatedFlag(false),
        SystemCalibratedFlag(false),
        ImuUpdatedFlag(false),
        ErrorFlag(false)
    {
        isBusFreed.store(false);
    }

    LimxBus::~LimxBus()
    {
        isBusFreed.store(true);
    }

    void LimxBus::Init(limxsdk::ApiBase* DeviceHandle)
    {
        this->DeviceHandle__ = DeviceHandle;


        for (auto&& dev : this->devices_)
        {
            if (dev->Type() == static_cast<uint32_t>(LimxDeviceType::LIMX_JOINT))
            {
                LimxJoint* ptr = dynamic_cast<LimxJoint*>(dev);
                if (ptr == nullptr)
                {
                    this->logger_->error("Failed to cast LimxJoint");
                    this->ErrorFlag.store(true);
                }
                this->Joints__.push_back(ptr);
            }
            else if (dev->Type() == static_cast<uint32_t>(LimxDeviceType::LIMX_IMU))
            {
                this->Imu__ = dynamic_cast<LimxImu*>(dev);
                if (this->Imu__ == nullptr)
                {
                    this->logger_->error("Failed to cast LimxImu");
                    this->ErrorFlag.store(true);
                }
            }
            else
            {
                this->logger_->error("Unknown device type, check your configuration xml.");
                this->ErrorFlag.store(true);
            }
            this->DeviceHandle__->subscribeRobotState(std::bind(&LimxBus::JointStateCallback, this, std::placeholders::_1));
            this->DeviceHandle__->subscribeImuData(std::bind(&LimxBus::ImuStateCallback, this, std::placeholders::_1));
            this->DeviceHandle__->subscribeDiagnosticValue(std::bind(&LimxBus::DiagnosticInfoCallback, this, std::placeholders::_1));
        }

        std::sort(this->Joints__.begin(), this->Joints__.end(), [](LimxJoint* a, LimxJoint* b) {return a->Id() < b->Id(); });

        this->RobotCmd__.resize(this->DeviceHandle__->getMotorNumber());
        if (this->DeviceHandle__->getMotorNumber() != this->Joints__.size())
        {
            this->logger_->error("The number of joints in the configuration file does not match the number of joints in the robot.");
            this->ErrorFlag.store(true);
        }

    }

    void LimxBus::WriteBus()
    {
        this->WriteBusCnt__++;
        if (this->WriteBusCnt__ % 4 != 0) //4分频到500hz
        {
            return;
        }

        LimxJointCmd cmd(0, 0, 0, 0, 0, 0, 0);
        for (size_t i = 0; i < this->Joints__.size(); i++)
        {
            this->Joints__[i]->UpdateJointCmd(cmd);
            this->RobotCmd__.mode[i] = cmd.mode;
            this->RobotCmd__.q[i] = cmd.q;
            this->RobotCmd__.dq[i] = cmd.dq;
            this->RobotCmd__.tau[i] = cmd.tau;
            this->RobotCmd__.Kp[i] = cmd.Kp;
            this->RobotCmd__.Kd[i] = cmd.Kd;
            this->RobotCmd__.stamp = cmd.stamp;
        }
        auto ok = this->DeviceHandle__->publishRobotCmd(this->RobotCmd__);
        if (!ok)
        {
            this->logger_->error("Failed to publish robot command");
        }
    }

    void LimxBus::ReadBus()
    {
        //place holder do nothing
    }

    void LimxBus::PowerOnDevice()
    {
        for (auto&& joint : this->Joints__)
        {
            joint->Enable();
        }
    }

    void LimxBus::SafeTorqueOff()
    {
        for (auto&& joint : this->Joints__)
        {
            joint->Disable();
        }
        this->WriteBus();
        this->WriteBus(); //多杀几次，解决分频问题
        this->WriteBus();
        this->WriteBus();
        this->WriteBus();
    }

    void LimxBus::RegisterDevices()
    {
        static DeviceRegistrar<LimxDevice, LimxJoint> JointReg(static_cast<uint32_t>(LimxDeviceType::LIMX_JOINT), "LimxJoint");
        static DeviceRegistrar<LimxDevice, LimxImu> ImuReg(static_cast<uint32_t>(LimxDeviceType::LIMX_IMU), "LimxImu");
    }

    void LimxBus::JointStateCallback(const limxsdk::RobotStateConstPtr& state)
    {
        if (isBusFreed.load())
        {
            //std::cout << "bus is freed";
            return;
        }
        for (size_t i = 0; i < this->Joints__.size(); i++)
        {
            LimxJointState JS(state->q[i], state->dq[i], state->tau[i], state->stamp);
            this->Joints__[i]->UpdateJointState(JS);
        }
        this->StateUpdatedFlag.store(true);
    }

    void LimxBus::ImuStateCallback(const limxsdk::ImuDataConstPtr& state)
    {
        if (isBusFreed.load())
        {
            //std::cout << "bus is freed";
            return;
        }
        //std::cout << "imu call" << std::endl;
        this->Imu__->UpdateImuData(state);
        this->ImuUpdatedFlag.store(true);
    }

    void LimxBus::DiagnosticInfoCallback(const limxsdk::DiagnosticValueConstPtr& state)
    {
        if (isBusFreed.load())
        {
            //std::cout << "bus is freed";
            return;
        }
        //std::cout << "diag call" << std::endl;
        std::string InfoStr = state->name;
        InfoStr += ": ";
        InfoStr += state->message;
        InfoStr += " code: ";
        InfoStr += std::to_string(state->code);
        if (state->level == limxsdk::DiagnosticValue::OK)
        {
            this->logger_->info(InfoStr);
        }
        else if (state->level == limxsdk::DiagnosticValue::WARN)
        {
            this->logger_->warn(InfoStr);
        }
        else
        {
            this->logger_->error(InfoStr);
            this->ErrorFlag.store(true);
        }

        if (state->name == "calibration" && state->level == limxsdk::DiagnosticValue::OK)
        {
            this->SystemCalibratedFlag.store(true);
        }
    }

};

