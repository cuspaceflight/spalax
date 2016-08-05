#include "StateDetailView.h"
#include <Rendering/Camera/FTCamera2D.h>
#include <Rendering/Text/FTLabel.h>
#include <sstream>
#include <Util/FTStringUtils.h>
#include <Rendering/FTWindowSizeNode.h>
#include <telemetry.h>
#include <messaging.h>
#include <ms5611_config.h>
#include <mpu9250_config.h>
#include <calibration.h>
#include <Rendering/Primitives/FTTexturedPlane.h>
#include <Rendering/Text/FTFontCache.h>
#include <state_estimate.h>


static StateDetailView* s_instance = nullptr;
static const int num_labels = 12;
float values[num_labels];
bool has_mpu9250_config = false;
mpu9250_config_t mpu9250_config;


static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
    if (s_instance == nullptr)
        return false;
    if (packet->header.id == telemetry_id_mpu9250_data) {
        if (!has_mpu9250_config)
            return true;
        mpu9250_data_t* data = (mpu9250_data_t*)packet->payload;

        mpu9250_calibrated_data_t calibrated;
        mpu9250_calibrate_data(&mpu9250_config, data, &calibrated);

        values[3] = calibrated.accel[0];
        values[4] = calibrated.accel[1];
        values[5] = calibrated.accel[2];

        values[6] = calibrated.gyro[0];
        values[7] = calibrated.gyro[1];
        values[8] = calibrated.gyro[2];

        values[9] = calibrated.magno[0];
        values[10] = calibrated.magno[1];
        values[11] = calibrated.magno[2];
    } else if (packet->header.id == telemetry_id_mpu9250_config) {
        if (has_mpu9250_config)
            return true;
        mpu9250_config_t* config = (mpu9250_config_t*)packet->payload;
        mpu9250_config = *config;
        has_mpu9250_config = true;
    }
    else if (packet->header.id == telemetry_id_ms5611_data) {
        FTAssert(packet->header.length == sizeof(ms5611data_t), "Incorrect Packet Size");
        auto data = (ms5611data_t*)packet->payload;

        values[0] = (float)data->pressure;
        values[1] = state_estimation_pressure_to_altitude((float)data->pressure);
        values[2] = (float)data->temperature;
    }
    return true;
}

MESSAGING_CONSUMER(messaging_consumer, telemetry_source_all, telemetry_source_all_mask, 0, 0, getPacket, 1024);

StateDetailView::StateDetailView() {
    FTAssert(s_instance == nullptr, "Only one StateDetailView instance can exist at once");
    
    auto camera = std::make_shared<FTCamera2D>();
    camera->setCullFaceEnabled(false);
    setCamera(std::move(camera));

    static const wchar_t* label_names[num_labels] = { 
        L"MS5611 Pressure", L"MS5611 Altitude", L"MS5611 Temperature",
        L"MPU9250 Accel X", L"MPU9250 Accel Y", L"MPU9250 Accel Z", 
        L"MPU9250 Gyro X", L"MPU9250 Gyro Y", L"MPU9250 Gyro Z", 
        L"MPU9250 Magno X", L"MPU9250 Magno Y", L"MPU9250 Magno Z"};

    auto window_size_node = std::make_shared<FTWindowSizeNode>();
    window_size_node->setAnchorPoint(glm::vec2(0, -1.0f));
    addChild(window_size_node);

    const float y_padding = 30.0f;
    float y = -y_padding;
    for (int i = 0; i < num_labels; i++) {
        auto label = std::make_shared<FTLabel>("Fonts/Vera.ttf", label_names[i], 16);
        window_size_node->addChild(label);
        label->setPosition(glm::vec2(30, y));
        label->setAnchorPoint(glm::vec2(0, 0.5f));
        label->setFillColor(glm::vec3(1, 1, 1));

        label = std::make_shared<FTLabel>("Fonts/Vera.ttf", L"0", 16, true);
        window_size_node->addChild(label);
        label->setPosition(glm::vec2(300, y));
        label->setAnchorPoint(glm::vec2(1, 0.5f));
        label->setFillColor(glm::vec3(1, 1, 1));

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
        value_labels_[i]->setString(FTWCharUtil::formatString(buff, 24, L"%.2f", values[i]));
}
