#include "device/limx_imu.h"

namespace bitbot
{
    LimxImu::LimxImu(const pugi::xml_node& imu_node)
        :LimxDevice(imu_node)
    {
        this->basic_type_ = static_cast<uint32_t>(BasicDeviceType::IMU);
        this->type_ = static_cast<uint32_t>(LimxDeviceType::LIMX_IMU);
        this->monitor_header_.headers = { "roll", "pitch", "yaw", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z" };
        this->monitor_data_.resize(monitor_header_.headers.size());
    }

    float LimxImu::GetRoll()
    {
        return this->imu_data_.runtime.roll;
    }
    float LimxImu::GetPitch()
    {
        return this->imu_data_.runtime.pitch;
    }
    float LimxImu::GetYaw()
    {
        return imu_data_.runtime.yaw;
    }
    float LimxImu::GetAccX()
    {
        return this->imu_data_.runtime.a_x;
    }
    float LimxImu::GetAccY()
    {
        return this->imu_data_.runtime.a_y;
    }
    float LimxImu::GetAccZ()
    {
        return this->imu_data_.runtime.a_z;
    }
    float LimxImu::GetGyroX()
    {
        return this->imu_data_.runtime.w_x;
    }
    float LimxImu::GetGyroY()
    {
        return this->imu_data_.runtime.w_y;
    }
    float LimxImu::GetGyroZ()
    {
        return this->imu_data_.runtime.w_z;
    }


    void LimxImu::UpdateRuntimeData()
    {
        //his->monitor_header_.headers = { "roll", "pitch", "yaw", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z" };
        this->monitor_data_[0] = this->imu_data_.runtime.roll.load();
        this->monitor_data_[1] = this->imu_data_.runtime.pitch.load();
        this->monitor_data_[2] = this->imu_data_.runtime.yaw.load();
        this->monitor_data_[3] = this->imu_data_.runtime.a_x.load();
        this->monitor_data_[4] = this->imu_data_.runtime.a_y.load();
        this->monitor_data_[5] = this->imu_data_.runtime.a_z.load();
        this->monitor_data_[6] = this->imu_data_.runtime.w_x.load();
        this->monitor_data_[7] = this->imu_data_.runtime.w_y.load();
        this->monitor_data_[8] = this->imu_data_.runtime.w_z.load();
    }
    std::array<float, 3> LimxImu::cvtEuler(const float* quat)
    {
        //FIXME: check the correctness of the conversion
        std::array<float, 3> euler;
        euler[0] = atan2(2 * (quat[0] * quat[1] + quat[2] * quat[3]), 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]));
        euler[1] = asin(2 * (quat[0] * quat[2] - quat[3] * quat[1]));
        euler[2] = atan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]), 1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]));
        return euler;
    }
    void LimxImu::UpdateImuData(limxsdk::ImuDataConstPtr ImuDataPtr)
    {
        auto euler = cvtEuler(ImuDataPtr->quat);
        this->imu_data_.runtime.a_x = ImuDataPtr->acc[0];
        this->imu_data_.runtime.a_y = ImuDataPtr->acc[1];
        this->imu_data_.runtime.a_z = ImuDataPtr->acc[2];
        this->imu_data_.runtime.w_x = ImuDataPtr->gyro[0];
        this->imu_data_.runtime.w_y = ImuDataPtr->gyro[1];
        this->imu_data_.runtime.w_z = ImuDataPtr->gyro[2];
        this->imu_data_.runtime.roll = euler[0];
        this->imu_data_.runtime.pitch = euler[1];
        this->imu_data_.runtime.yaw = euler[2];
    }
};
