#pragma once

#include <Arduino.h>
#include "task.h"


struct Message {
    double trigger_name;
    int trigger_value;
};

class ConnectivityTask : public Task<ConnectivityTask> {
    friend class Task<ConnectivityTask>; // Allow base Task to invoke protected run()

    public:
        ConnectivityTask(const uint8_t task_core);
        ~ConnectivityTask();

        void sendMqttMessage(Message message);

    protected:
        void run();

    private:
        QueueHandle_t queue_;

        void initWiFi();
        void connectToMqttBroker();
};
