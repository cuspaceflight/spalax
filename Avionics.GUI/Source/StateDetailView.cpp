#include "StateDetailView.h"
#include <Rendering/Camera/FTCamera2D.h>
#include <Rendering/Text/FTLabel.h>
#include <Rendering/FTDirector.h>
#include <sstream>
#include <Util/FTStringUtils.h>
#include <Rendering/FTWindowSizeNode.h>
#include <telemetry.h>
#include <messaging.h>
#include <ms5611_config.h>
#include <mpu9250_config.h>

extern "C" {
#include <math_utils.h>
}

static StateDetailView* s_instance = nullptr;
static const int num_labels = 12;
uint32_t values[num_labels];

static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
    if (s_instance == nullptr)
        return false;
    if (packet->header.id == telemetry_id_mpu9250_data) {
        FTAssert(packet->header.length == 20, "Incorrect Packet Size");
        mpu9250_data_t* data = (mpu9250_data_t*)packet->payload;

        values[2] = data->accel[0];
        values[3] = data->accel[1];
        values[4] = data->accel[2];

        values[5] = data->gyro[0];
        values[6] = data->gyro[1];
        values[7] = data->gyro[2];

        values[8] = data->magno[0];
        values[9] = data->magno[1];
        values[10] = data->magno[2];

        values[11] = data->temp;
    }
    else if (packet->header.id == telemetry_id_ms5611_data) {
        FTAssert(packet->header.length == sizeof(ms5611data_t), "Incorrect Packet Size");
        auto data = (ms5611data_t*)packet->payload;

        values[0] = data->pressure;
        values[1] = data->temperature;
    }
    return true;
}

MESSAGING_CONSUMER(messaging_consumer, 0b00000001000, 0b11111111000, 0, 0, getPacket, 1024);

StateDetailView::StateDetailView() {
    FTAssert(s_instance == nullptr, "Only one StateDetailView instance can exist at once");
    

    setCamera(std::make_shared<FTCamera2D>());

    static const wchar_t* label_names[num_labels] = { 
        L"MS5611 Pressure", L"MS5611 Temperature", 
        L"MPU9250 Accel X", L"MPU9250 Accel Y", L"MPU9250 Accel Z", 
        L"MPU9250 Gyro X", L"MPU9250 Gyro Y", L"MPU9250 Gyro Z", 
        L"MPU9250 Magno X", L"MPU9250 Magno Y", L"MPU9250 Magno Z", 
        L"MPU9250 Temp"};

    auto window_size_node = std::make_shared<FTWindowSizeNode>();
    window_size_node->setAnchorPoint(glm::vec2(0, -1.0f));
    addChild(window_size_node);

    
    const float y_padding = 25.0f;
    float y = -y_padding;
    for (int i = 0; i < num_labels; i++) {
        auto label = std::make_shared<FTLabel>("Fonts/Vera.ttf", label_names[i], 14);
        window_size_node->addChild(label);
        label->setPosition(glm::vec2(30, y));
        label->setAnchorPoint(glm::vec2(0, 0.5f));

        label = std::make_shared<FTLabel>("Fonts/Vera.ttf", L"0", 14, true);
        window_size_node->addChild(label);
        label->setPosition(glm::vec2(250, y));
        label->setAnchorPoint(glm::vec2(1, 0.5f));

        value_labels_.push_back(label.get());
        y -= y_padding;

        values[i] = 0;
    }

    messaging_consumer_init(&messaging_consumer);

    s_instance = this;
}

StateDetailView::~StateDetailView() {
    FTLog("State Detail View Destroyed");
}

void StateDetailView::updateDisplay() {
    while (messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok);

    static wchar_t buff[24];
    for (int i = 0; i < num_labels; i++)
        value_labels_[i]->setString(FTStringUtil<wchar_t>::formatString(buff, 24, L"%i", values[i]));
}
