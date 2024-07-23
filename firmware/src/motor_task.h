#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include <vector>

#include "configuration.h"
#include "logger.h"
#include "proto_gen/smartknob.pb.h"
#include "task.h"


///Enums for what kind of command will occur
enum class CommandType {
    CALIBRATE,
    CONFIG,
    HAPTIC,
};

///True/False for pressing
struct HapticData {
    bool press;
};


///Store what kind of data for each command
struct Command {
    CommandType command_type;
    union CommandData {
        uint8_t unused;                 ///data place holder
        PB_SmartKnobConfig config;      ///appropriate smartknob config
        HapticData haptic;              ///True false for press
    };
    CommandData data;
};

class MotorTask : public Task<MotorTask> {  ///Utilizing RTOS defined in Task.h
    friend class Task<MotorTask>;           /// Allow base Task to invoke protected run()

    public:
        
        ///Data needed for a Motor task to be performed
        MotorTask(const uint8_t task_core, Configuration& configuration); 
        ~MotorTask();

        ///Configuration needed for motor
        void setConfig(const PB_SmartKnobConfig& config);
        
        ///Play haptic given true/false
        void playHaptic(bool press);
        
        /// @brief Calibrate motor
        void runCalibration();

        void addListener(QueueHandle_t queue);
        void setLogger(Logger* logger);

    protected:
        void run();

    private:
    ///Configuration instance for morot
        Configuration& configuration_;
        QueueHandle_t queue_;
        Logger* logger_;
        std::vector<QueueHandle_t> listeners_;
        char buf_[72];

        // BLDC motor & driver instance
        BLDCMotor motor = BLDCMotor(1);
        BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);

        void publish(const PB_SmartKnobState& state);
        void calibrate();
        void checkSensorError();
        void log(const char* msg);
};
