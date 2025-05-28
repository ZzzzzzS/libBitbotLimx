#include "device/limx_joint.h"
#define _USE_MATH_DEFINES
#include "cmath"
#include "math.h"
#include "iostream"


namespace bitbot
{
    LimxJoint::LimxJoint(const pugi::xml_node& joint_node)
        :LimxAbstractJoint(joint_node),
        JointState__(0, 0, 0, 0),
        JointCmd__(0, 0, 0, 0, 0, 0, 0),
        JointMode__(LimxJointMode::CST),
        Enable__(false),
        FirstReceive__(true)
    {
        this->basic_type_ = static_cast<uint32_t>(BasicDeviceType::MOTOR);
        this->type_ = static_cast<uint32_t>(LimxDeviceType::LIMX_JOINT);
        monitor_header_.headers = { "status","mode","actual_position", "target_position", "actual_velocity","target_velocity","actual_current","target_torque","Kp","Kd" };
        monitor_data_.resize(monitor_header_.headers.size());

        std::string mode;
        ConfigParser::ParseAttribute2s(mode, joint_node.attribute("mode"));
        if (mode.compare("CSP") == 0)
        {
            this->SetMode(LimxJointMode::CSP);
        }
        else if (mode.compare("CST") == 0)
        {
            this->SetMode(LimxJointMode::CST);
        }
        else if (mode.compare("CSV") == 0)
        {
            this->SetMode(LimxJointMode::CSV);
        }
        else
        {
            this->SetMode(LimxJointMode::CST);
        }

        bool en;
        ConfigParser::ParseAttribute2b(en, joint_node.attribute("enable"));
        this->EnableInCfg__.store(en);

        double kp, kd;
        ConfigParser::ParseAttribute2d(kp, joint_node.attribute("kp"));
        ConfigParser::ParseAttribute2d(kd, joint_node.attribute("kd"));

        ConfigParser::ParseAttribute2i(this->MotorDirection__, joint_node.attribute("motor_direction"));
        ConfigParser::ParseAttribute2i(this->TorqueDirection__, joint_node.attribute("torque_direction"));
        this->MotorDirection__ /= std::abs(this->MotorDirection__);
        this->TorqueDirection__ /= std::abs(this->TorqueDirection__);

        this->setKp(kp);
        this->setKd(kd);
    }
    void LimxJoint::Enable()
    {
        if (this->EnableInCfg__.load() == false)
        {
            std::string msg = "LimxJoint " + this->name_ + " is not enabled in config file, will not enable it!";
            this->logger_->warn(msg);
        }
        else
        {
            std::string msg = "LimxJoint " + this->name_ + " is enabled!";
            this->logger_->info(msg);
            this->Enable__ = true;
        }
    }
    void LimxJoint::Disable()
    {
        this->Enable__ = false;
    }
    bool LimxJoint::isEnable()
    {
        return this->Enable__.load();
    }
    float LimxJoint::GetActualPosition()
    {
        return this->JointState__.q.load();
    }
    float LimxJoint::GetActualVelocity()
    {
        return this->JointState__.dq.load();
    }
    float LimxJoint::GetActualTorque()
    {
        return this->JointState__.tau.load();
    }
    float LimxJoint::GetTargetPosition()
    {
        return this->JointCmd__.q.load();
    }
    float LimxJoint::GetTargetVelocity()
    {
        return this->JointCmd__.dq.load();
    }
    float LimxJoint::GetTargetTorque()
    {
        return this->JointCmd__.tau.load();
    }
    void LimxJoint::SetTargetPosition(float pos)
    {
        return this->JointCmd__.q.store(pos);
    }
    void LimxJoint::SetTargetVelocity(float vel)
    {
        return this->JointCmd__.dq.store(vel);
    }
    void LimxJoint::SetTargetTorque(float torque)
    {
        return this->JointCmd__.tau.store(torque);
    }
    void LimxJoint::SetMode(LimxJointMode mode)
    {
        this->JointMode__ = mode;
        this->JointCmd__.mode.store(static_cast<uint8_t>(mode));
    }
    LimxJointMode LimxJoint::GetMode()
    {
        return this->JointMode__;
    }
    void LimxJoint::setKp(float kp)
    {
        this->JointCmd__.Kp.store(kp);
    }
    void LimxJoint::setKd(float kd)
    {
        this->JointCmd__.Kd.store(kd);
    }
    float LimxJoint::getKp()
    {
        return this->JointCmd__.Kp.load();
    }
    float LimxJoint::getKd()
    {
        return this->JointCmd__.Kd.load();
    }
    void LimxJoint::UpdateRuntimeData()
    {
        constexpr float r2d = 180.0f / M_PI;
        this->monitor_data_[0] = this->Enable__.load() == true ? 1.0 : 0.0;
        this->monitor_data_[1] = static_cast<uint8_t>(this->JointMode__);
        this->monitor_data_[2] = this->JointState__.q.load() * r2d;
        this->monitor_data_[3] = this->JointCmd__.q.load() * r2d;
        this->monitor_data_[4] = this->JointState__.dq.load();
        this->monitor_data_[5] = this->JointCmd__.dq.load();
        this->monitor_data_[6] = this->JointState__.tau.load();
        this->monitor_data_[7] = this->JointCmd__.tau.load();
        this->monitor_data_[8] = this->JointCmd__.Kp.load();
        this->monitor_data_[9] = this->JointCmd__.Kd.load();

    }
    void LimxJoint::UpdateJointState(const LimxJointState& state)
    {
        this->JointState__.q.store(state.q * this->MotorDirection__);
        this->JointState__.dq.store(state.dq * this->MotorDirection__);
        this->JointState__.tau.store(state.tau * this->TorqueDirection__);
        this->JointState__.stamp.store(state.stamp);
        if (this->FirstReceive__.load())
        {
            this->FirstReceive__.store(false);
            this->JointCmd__.q.store(state.q.load() * this->MotorDirection__);
            this->JointCmd__.dq = 0;
            this->JointCmd__.tau = 0;
        }
    }
    void LimxJoint::UpdateJointCmd(LimxJointCmd& cmd)
    {
        if (this->isEnable())
        {
            cmd.mode = this->JointCmd__.mode.load();
            cmd.q = this->JointCmd__.q.load() * this->MotorDirection__;
            cmd.dq = this->JointCmd__.dq.load() * this->MotorDirection__;
            cmd.tau = this->JointCmd__.tau.load() * this->TorqueDirection__;
            cmd.Kp = this->JointCmd__.Kp.load();
            cmd.Kd = this->JointCmd__.Kd.load();
            cmd.stamp = this->JointCmd__.stamp.load();


            LimxJointMode mod = static_cast<LimxJointMode>(cmd.mode.load());
            switch (mod)
            {
            case LimxJointMode::CSP:
                cmd.dq.store(0.0);
                cmd.tau.store(0.0);
                break;
            case LimxJointMode::CSV:
                cmd.q.store(0.0);
                cmd.Kp.store(0.0);
                cmd.tau.store(0.0);
                break;
            case LimxJointMode::CST:
                cmd.dq.store(0.0);
                cmd.q.store(0.0);
                cmd.Kp.store(0.0);
                cmd.Kd.store(0.0);
                break;
            default:
                cmd.dq.store(0.0);
                cmd.q.store(0.0);
                cmd.Kp.store(0.0);
                cmd.Kd.store(0.0);
                break;
            }
        }
        else
        {
            cmd.mode = static_cast<uint8_t>(LimxJointMode::CST);
            cmd.q = 0;
            cmd.dq = 0;
            cmd.tau = 0;
            cmd.Kp = 0;
            cmd.Kd = 0;
            cmd.stamp = 0;
        }

    }
};