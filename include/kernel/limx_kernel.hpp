#pragma once
#include "bitbot_kernel/kernel/kernel.hpp"
#include "bus/limx_bus.h"
#include "limxsdk/apibase.h"
#include "limxsdk/pointfoot.h"
#include "string"
#include "iostream"

namespace bitbot
{
    enum class RobotType :uint8_t
    {
        pf = 0,
        wf,
        df
    };

    /// @brief Bitbot Limx 内核事件类型，继承自EventId
    enum class LimxKernelEvent : EventId
    {
        POWER_ON = 100,
    };

    /// @brief Bitbot Limx 内核状态类型，继承自StateId
    enum class LimxKernelState : StateId
    {
        POWER_ON_FINISH = 100
    };

    template<typename UserData>
    class LimxKernel : public KernelTpl<LimxKernel<UserData>, LimxBus, UserData>
    {
    public:
        LimxKernel(std::string config_file)
            : KernelTpl<LimxKernel<UserData>, LimxBus, UserData>(config_file)
        {
            this->KernelRegisterEvent("power_on", static_cast<EventId>(LimxKernelEvent::POWER_ON), [this](EventValue, UserData&)
                {
                    this->logger_->info("joints power on");
                    this->busmanager_.PowerOnDevice();
                    this->logger_->info("joints power on finished");
                    return static_cast<StateId>(LimxKernelState::POWER_ON_FINISH); }, false);
            this->KernelRegisterState("power on finish", static_cast<StateId>(LimxKernelState::POWER_ON_FINISH),
                [this](const bitbot::KernelInterface& kernel, ExtraData& extra_data, UserData& user_data) {}, { static_cast<EventId>(KernelEvent::START) });

            this->InjectEventsToState(static_cast<StateId>(KernelState::IDLE), { static_cast<EventId>(LimxKernelEvent::POWER_ON) });


            //parse config
            pugi::xml_node kernel_node = this->parser_->GetBitbotNode();
            pugi::xml_node limx_node = kernel_node.child("limx");
            std::string limx_ip;
            std::string RobotName;
            ConfigParser::ParseAttribute2s(limx_ip, limx_node.attribute("ip"));
            ConfigParser::ParseAttribute2s(RobotName, limx_node.attribute("name"));

            //set env var
            setenv("ROBOT_TYPE", RobotName.c_str(), 1);

            this->limx_handle_ = limxsdk::PointFoot::getInstance();
            if (this->limx_handle_ == nullptr)
            {
                std::cerr << "Error: Failed to create limx handle!" << std::endl;
                exit(-1);
            }
            //create limx handle

            auto ok = this->limx_handle_->init(limx_ip);
            if (!ok)
            {
                std::cerr << "Error: Failed to init limx handle!" << std::endl;
                exit(-1);
            }
            this->busmanager_.Init(this->limx_handle_);

            //print welcome message
            this->PrintWelcomeMessage();
        }

        ~LimxKernel()
        {
            std::cout << "\033[32mGood bye from Bitbot Limx. Make Bitbot Everywhere! \033[0m" << std::endl;
        }

    protected:
        void doStart()
        {
            this->logger_->info("LimxKernel Start");
        }

        void doRun()
        {
            //init limx bus
            //loop
            this->logger_->info("waiting for calibration...");
            while (!this->busmanager_.SystemCalibratedFlag)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            this->logger_->info("calibration done, waiting for robot state...");
            while (!this->busmanager_.ImuUpdatedFlag || !this->busmanager_.StateUpdatedFlag)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            this->logger_->info("robot init done, start to run...");

            std::chrono::high_resolution_clock::time_point this_time = std::chrono::high_resolution_clock::now();
            std::chrono::high_resolution_clock::time_point last_time = this_time;
            std::chrono::high_resolution_clock::time_point end_time = this_time;
            while (!this->kernel_config_data_.stop_flag)
            {
                constexpr double ns_to_ms = 1 / 1e6;
                constexpr double s_to_ms = 1e3;


                if (this->busmanager_.ErrorFlag)
                {
                    this->logger_->error("Error occurred, Bitbot Limx kernel is stopping!");
                    this->kernel_config_data_.stop_flag = true;
                    break;
                }

                while (!this->busmanager_.StateUpdatedFlag.load())
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(10));
                }
                this->busmanager_.StateUpdatedFlag.store(false);

                this->kernel_runtime_data_.periods_count++;
                this_time = std::chrono::high_resolution_clock::now();
                this->kernel_runtime_data_.period = std::chrono::duration_cast<std::chrono::nanoseconds>(this_time - last_time).count() * ns_to_ms;
                last_time = this_time;

                this->HandleEvents();
                this->KernelLoopTask();

                end_time = std::chrono::high_resolution_clock::now();
                this->kernel_runtime_data_.process_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - this_time).count() * ns_to_ms;

                this->KernelPrivateLoopEndTask();
            }
            this->busmanager_.SafeTorqueOff();
        }

    private:
        void PrintWelcomeMessage()
        {
            std::string line0 = "\033[32m================================================================================\033[0m";
            std::string line1 = "\033[32m|| BBBB    III  TTTTT   BBBB     OOO    TTTTT      L      III   M   M   X   X ||\033[0m";
            std::string line2 = "\033[32m|| B   B    I     T     B   B   O   O     T        L       I    MM MM    X X  ||\033[0m";
            std::string line3 = "\033[32m|| BBBB     I     T     BBBB    O   O     T        L       I    M M M     X   ||\033[0m";
            std::string line4 = "\033[32m|| B   B    I     T     B   B   O   O     T        L       I    M   M    X X  ||\033[0m";
            std::string line5 = "\033[32m|| BBBB    III    T     BBBB     OOO      T        LLLLL  III   M   M   X   X ||\033[0m";
            std::string line6 = "\033[32m================================================================================\033[0m";

            std::cout << std::endl << std::endl;
            std::cout << "\033[31mWelcome to use Bitbot Limx. Make Bitbot Everywhere! \033[0m" << std::endl;
            std::cout << line0 << std::endl;
            std::cout << line1 << std::endl;
            std::cout << line2 << std::endl;
            std::cout << line3 << std::endl;
            std::cout << line4 << std::endl;
            std::cout << line5 << std::endl;
            std::cout << line6 << std::endl << std::endl;
        }
        //function
    private:
        limxsdk::PointFoot* limx_handle_;
        bool is_calibration__;
    };
};