#if SK_LEDS
#include <FastLED.h>
#endif

#if SK_STRAIN
#include <HX711.h>
#endif

#if SK_ALS
#include <Adafruit_VEML7700.h>
#endif

#include "interface_task.h"
#include "semaphore_guard.h"
#include "util.h"
// interface_task.cpp



#if SK_LEDS
CRGB leds[NUM_LEDS];
#endif

#if SK_STRAIN
HX711 scale;
#endif

#if SK_ALS
Adafruit_VEML7700 veml = Adafruit_VEML7700();
#endif

int current_config_ = 0;

/// @brief Predefined configurations/modes
static PB_SmartKnobConfig configs[] = {
    // int32_t position;
    // float sub_position_unit;
    // uint8_t position_nonce;
    // int32_t min_position;
    // int32_t max_position;
    // float position_width_radians;
    // float detent_strength_unit;
    // float endstop_strength_unit;
    // float snap_point;
    // char text[51];
    // pb_size_t detent_positions_count;
    // int32_t detent_positions[5];
    // float snap_point_bias;
    // int8_t led_hue;

    {
        0,
        0,
        0,
        0,
        4,
        7 * PI / 180,
        2,
        0.1,
        1.1,
        "Brightness",
        0,
        {},
        0,
        120,
    },
    {
        0,
        0,
        1,
        0,
        4,
        7 * PI / 180,
        2,
        0.2,
        1.1,
        "Saturation",
        0,
        {},
        0,
        130,
    },
    {
        0,
        0,
        2,
        0,
        4,
        7 * PI / 180,
        2,
        0.3,
        1.1,
        "Contrast",
        0,
        {},
        0,
        140,
    },
    {
        0,
        0,
        3,
        0,
        1,
        60 * PI / 180,
        1,
        0.4,
        0.55, // Note the snap point is slightly past the midpoint (0.5); compare to normal detents which use a snap point *past* the next value (i.e. > 1)
        "White Balance",
        0,
        {},
        0,
        150,
    },
    {
        127,
        0,
        5,
        0,
        255,
        1 * PI / 180,
        1,
        0.5,
        1.1,
        "Left-Right Servo",
        0,
        {},
        0,
        25,
    }
};

/// @brief takes int Motor, Display, and Interface task and starts them. Then starts calibration for motor
/// @param task_core        I think this is the name
/// @param motor_task       Initialized motor task
/// @param display_task     Initialized display task
InterfaceTask::InterfaceTask(const uint8_t task_core, MotorTask& motor_task, DisplayTask* display_task, ConnectivityTask& connectivity_task) : 
        Task("Interface", 3600, 1, task_core),
        stream_(),
        motor_task_(motor_task),
        display_task_(display_task),
        connectivity_task_(connectivity_task),
        plaintext_protocol_(stream_, [this] () {
            motor_task_.runCalibration();
        }, connectivity_task),
        proto_protocol_(stream_, [this] (PB_SmartKnobConfig& config) {
            applyConfig(config, true);
        }) 
        
{
    #if SK_DISPLAY
        assert(display_task != nullptr);
    #endif

    // Create Queue for logs 
    log_queue_ = xQueueCreate(10, sizeof(std::string *));
    assert(log_queue_ != NULL);

    // Queue for the knobs current state
    knob_state_queue_ = xQueueCreate(1, sizeof(PB_SmartKnobState));
    assert(knob_state_queue_ != NULL);

    // Mutex created
    mutex_ = xSemaphoreCreateMutex();
    assert(mutex_ != NULL);
}


InterfaceTask::~InterfaceTask() {
    vSemaphoreDelete(mutex_);
}

void InterfaceTask::run() {
    stream_.begin();
    

    //Led Initialization
    #if SK_LEDS
        FastLED.addLeds<SK6812, PIN_LED_DATA, GRB>(leds, NUM_LEDS);
    #endif

    #if SK_ALS && PIN_SDA >= 0 && PIN_SCL >= 0
        Wire.begin(PIN_SDA, PIN_SCL);
        Wire.setClock(400000);
    #endif
    #if SK_STRAIN
        scale.begin(PIN_STRAIN_DO, PIN_STRAIN_SCK);
    #endif

    #if SK_ALS
        if (veml.begin()) {
            veml.setGain(VEML7700_GAIN_2);
            veml.setIntegrationTime(VEML7700_IT_400MS);
        } else {
            log("ALS sensor not found!");
        }
    #endif

    //Apply the first configuraton made up top
    applyConfig(configs[0], false);

    //Allocate space for the knob state in Motor task
    motor_task_.addListener(knob_state_queue_);

    //initialize plaintext protocol
    plaintext_protocol_.init([this] () {
        changeConfig(true);
    }, [this] () {
        if (!configuration_loaded_) {
            return;
        }
        if (strain_calibration_step_ == 0) {
            log("Strain calibration step 1: Don't touch the knob, then press 'S' again");
            strain_calibration_step_ = 1;
        } else if (strain_calibration_step_ == 1) {
            configuration_value_.strain.idle_value = strain_reading_;
            snprintf(buf_, sizeof(buf_), "  idle_value=%d", configuration_value_.strain.idle_value);
            log(buf_);
            log("Strain calibration step 2: Push and hold down the knob with medium pressure, and press 'S' again");
            strain_calibration_step_ = 2;
        } else if (strain_calibration_step_ == 2) {
            configuration_value_.strain.press_delta = strain_reading_ - configuration_value_.strain.idle_value;
            configuration_value_.has_strain = true;
            snprintf(buf_, sizeof(buf_), "  press_delta=%d", configuration_value_.strain.press_delta);
            log(buf_);
            log("Strain calibration complete! Saving...");
            strain_calibration_step_ = 0;
            if (configuration_->setStrainCalibrationAndSave(configuration_value_.strain)) {
                log("  Saved!");
            } else {
                log("  FAILED to save config!!!");
            }
        }
    });

    // Start in legacy protocol mode
    current_protocol_ = &plaintext_protocol_;

    ProtocolChangeCallback protocol_change_callback = [this] (uint8_t protocol) {
        switch (protocol) {
            case SERIAL_PROTOCOL_LEGACY:
                current_protocol_ = &plaintext_protocol_;
                break;
            case SERIAL_PROTOCOL_PROTO:
                current_protocol_ = &proto_protocol_;
                break;
            default:
                log("Unknown protocol requested");
                break;
        }
    };

    plaintext_protocol_.setProtocolChangeCallback(protocol_change_callback);
    proto_protocol_.setProtocolChangeCallback(protocol_change_callback);

    // Interface loop:
    while (1) {
        
        //If the knob state has changed (has been pressed) then update current configuration
        if (xQueueReceive(knob_state_queue_, &latest_state_, 0) == pdTRUE) {
            publishState();
        }

        //stream information using UART
        current_protocol_->loop();

        //deletes passed log strings
        std::string* log_string;
        while (xQueueReceive(log_queue_, &log_string, 0) == pdTRUE) {
            current_protocol_->log(log_string->c_str());
            delete log_string;
        }

        //LED control and configuration update
        updateHardware();

        //If a configuration is not currently loaded, lock the mutex and wait till it is
        if (!configuration_loaded_) {
            SemaphoreGuard lock(mutex_);
            if (configuration_ != nullptr) {
                configuration_value_ = configuration_->get();
                configuration_loaded_ = true;
            }
        }

        delay(1);
    }
}

void InterfaceTask::log(const char* msg) {
    // Allocate a string for the duration it's in the queue; it is free'd by the queue consumer
    std::string* msg_str = new std::string(msg);

    // Put string in queue (or drop if full to avoid blocking)
    xQueueSendToBack(log_queue_, &msg_str, 0);
}

void InterfaceTask::changeConfig(bool next) {
    if (next) {
        current_config_ = (current_config_ + 1) % 5;
    } else {
        if (current_config_ == 0) {
            current_config_ = 5;
        } else {
            current_config_ --;
        }
    }

//COUNT_OF(configs)
    
    //snprintf(buf_, sizeof(buf_), "config %d ", current_config_);
    //log(buf_);
    applyConfig(configs[current_config_], false);
}

/// @brief LED and state control when being pressed
void InterfaceTask::updateHardware() {
    // How far button is pressed, in range [0, 1]
    float press_value_unit = 0;

    //calculation for for average lux value (light level)
    #if SK_ALS
       const float LUX_ALPHA = 0.005;
       static float lux_avg;
        float lux = veml.readLux();
        lux_avg = lux * LUX_ALPHA + lux_avg * (1 - LUX_ALPHA);
        static uint32_t last_als;
        
        //Log this value every second unless youre calibrating
       // if (millis() - last_als > 1000 && strain_calibration_step_ == 0) {
       //     snprintf(buf_, sizeof(buf_), "millilux: %.2f", lux*1000);
        //    log(buf_);
        //    last_als = millis();
       // }
    #endif

    static bool pressed;
    #if SK_STRAIN
        //Is the HX ready
        if (scale.wait_ready_timeout(100)) {
            strain_reading_ = scale.read();

            static uint32_t last_reading_display;
            //Log the strain reading every second as long as youre not calibrating
            if (millis() - last_reading_display > 1000 && strain_calibration_step_ == 0) {
              // snprintf(buf_, sizeof(buf_), "HX711 reading: %d", strain_reading_);
               // log(buf_);
                last_reading_display = millis();
            }

            //If the config is valid and alr loaded then map the strain between 0 to 1
            if (configuration_loaded_ && configuration_value_.has_strain && strain_calibration_step_ == 0) {
                // TODO: calibrate and track (long term moving average) idle point (lower)
                press_value_unit = lerp(strain_reading_, configuration_value_.strain.idle_value, configuration_value_.strain.idle_value + configuration_value_.strain.press_delta, 0, 1);

                // If the reading is within expected bounds
                if (-1 < press_value_unit && press_value_unit < 2) {
                    static uint8_t press_readings;
                    if (!pressed && press_value_unit > 1) {
                        press_readings++;
                        if (press_readings > 2) {
                            motor_task_.playHaptic(true);
                            pressed = true;
                           
                           //This updates to next configuration
                            press_count_++;
                            publishState();
                            //If the configuration wasn't changed remotely, make sure you update it here
                            if (!remote_controlled_) {
                                changeConfig(true);
                            }
                        }
                    } else if (pressed && press_value_unit < 0.5) {
                        press_readings++;
                        if (press_readings > 2) {
                            motor_task_.playHaptic(false);
                            pressed = false;
                        }
                    } else {
                        press_readings = 0;
                    }
                }
            }
        } else {
            log("HX711 not found.");

            //Leds turn red if HX not found
            #if SK_LEDS
                for (uint8_t i = 0; i < NUM_LEDS; i++) {
                    leds[i] = CRGB::Red;
                }
                FastLED.show();
            #endif
        }
    #endif

    uint16_t brightness = UINT16_MAX;
    // TODO: brightness scale factor should be configurable (depends on reflectivity of surface)
    #if SK_ALS
        brightness = (uint16_t)CLAMP(lux_avg * 13000, (float)1280, (float)UINT16_MAX);
    #endif

    #if SK_DISPLAY
        display_task_->setBrightness(brightness); // TODO: apply gamma correction
    #endif

     #if SK_LEDS
        for (uint8_t i = 0; i < NUM_LEDS; i++) {
            //set color based on latest configuration and press value
            leds[i].setHSV(latest_config_.led_hue, 255 - 180*CLAMP(press_value_unit, (float)0, (float)1) - 75*pressed, brightness >> 8);

            // Gamma adjustment
            leds[i].r = dim8_video(leds[i].r);
            leds[i].g = dim8_video(leds[i].g);
            leds[i].b = dim8_video(leds[i].b);
        }
        FastLED.show();
    #endif
}

void InterfaceTask::setConfiguration(Configuration* configuration) {
    SemaphoreGuard lock(mutex_);
    configuration_ = configuration;
}

/// @brief Press_count is initialized as an integer, and is incremented each press. 
/// When knob is pressed, we publish the state and update it here before sending it to serial
void InterfaceTask::publishState() {
    // Apply local state before publishing to serial
    latest_state_.press_nonce = press_count_;
    current_protocol_->handleState(latest_state_);
}

void InterfaceTask::applyConfig(PB_SmartKnobConfig& config, bool from_remote) {
    remote_controlled_ = from_remote;
    latest_config_ = config;
    motor_task_.setConfig(config);
}
