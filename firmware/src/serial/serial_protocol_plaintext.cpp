//Primarily for sending logs to the system

#include "../proto_gen/smartknob.pb.h"

#include "serial_protocol_plaintext.h"

#include "../connectivity_task.h"


/// @brief If a change in the knob happens between new and old state, send a meesage on current state values
/// I think this triggers when you press the knob

/// @param state Defined in interface.cpp
void SerialProtocolPlaintext::handleState(const PB_SmartKnobState& state) {
    bool substantial_change = (latest_state_.current_position != state.current_position)
        || (latest_state_.config.detent_strength_unit != state.config.detent_strength_unit)
        || (latest_state_.config.endstop_strength_unit != state.config.endstop_strength_unit)
        || (latest_state_.config.min_position != state.config.min_position)
        || (latest_state_.config.max_position != state.config.max_position);
    latest_state_ = state;

    if (substantial_change) {
        stream_.printf("<STATE: %d>\n current config: %f\n",       // (detent strength: %0.2f, width: %0.0f deg, endstop strength: %0.2f)\n", 
            state.current_position, state.config.endstop_strength_unit);
            //Send State with MQTT here

            //state.config.min_position,
            //state.config.max_position,
            //state.config.detent_strength_unit,
            //degrees(state.config.position_width_radians),
            //state.config.endstop_strength_unit);
            
        int current_position = state.current_position;
        double current_config = state.config.endstop_strength_unit;
        
        
         Message msg = {
            .trigger_name = current_config,
            .trigger_value = current_position
        };
        
        connectivity_task_.sendMqttMessage(msg);
        
            

        
        }
    }



/// @brief Appends log to whatever log is going to be sent
/// @param msg log message to be sent back to host
void SerialProtocolPlaintext::log(const char* msg) {
    //stream_.print("LOG: ");
    stream_.println(msg);
}

/// @brief Reads input from host and and performs actions dependent on whats read
void SerialProtocolPlaintext::loop() {
    //while UART streaming is available
    while (stream_.available() > 0) {
        int b = stream_.read();
        if (b == 0) {
            if (protocol_change_callback_) {
                protocol_change_callback_(SERIAL_PROTOCOL_PROTO);
            }
            break;
        }
        if (b == ' ') {
            if (demo_config_change_callback_) {
                demo_config_change_callback_();
            }
        } else if (b == 'C') {
            motor_calibration_callback_();
        } else if (b == 'S') {
            if (strain_calibration_callback_) {
                strain_calibration_callback_();
            }
        }
    }
}

void SerialProtocolPlaintext::init(DemoConfigChangeCallback demo_config_change_callback, StrainCalibrationCallback strain_calibration_callback) {
    demo_config_change_callback_ = demo_config_change_callback;
    strain_calibration_callback_ = strain_calibration_callback;
    stream_.println("SmartKnob starting!\n\nSerial mode: plaintext\nPress 'C' at any time to calibrate motor/sensor.\nPress 'S' at any time to calibrate strain sensors.\nPress <Space> to change haptic modes.");
}
