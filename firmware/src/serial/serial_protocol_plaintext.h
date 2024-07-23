#pragma once

#include "../proto_gen/smartknob.pb.h"

#include "interface_callbacks.h"
#include "motor_task.h"
#include "serial_protocol.h"
#include "uart_stream.h"
#include "connectivity_task.h"


typedef std::function<void(void)> DemoConfigChangeCallback;
typedef std::function<void(void)> StrainCalibrationCallback;


        //LightsPage(ConnectivityTask& connectivity_task) : Page(), connectivity_task_(connectivity_task) {}

class SerialProtocolPlaintext : public SerialProtocol {
    public:
        SerialProtocolPlaintext(Stream& stream, MotorCalibrationCallback motor_calibration_callback, ConnectivityTask& connectivity_task) : SerialProtocol(), stream_(stream), motor_calibration_callback_(motor_calibration_callback), connectivity_task_(connectivity_task) {}
        ~SerialProtocolPlaintext(){}
        void log(const char* msg) override;
        void loop() override;
        void handleState(const PB_SmartKnobState& state) override;

        void init(DemoConfigChangeCallback demo_config_change_callback, StrainCalibrationCallback strain_calibration_callback);
    
    private:
        Stream& stream_;
        MotorCalibrationCallback motor_calibration_callback_;
        PB_SmartKnobState latest_state_ = {};
        DemoConfigChangeCallback demo_config_change_callback_;
        ConnectivityTask& connectivity_task_; 
        StrainCalibrationCallback strain_calibration_callback_;
};
