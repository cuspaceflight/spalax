#include "StateDetailView.h"
#include <Rendering/Camera/FTCamera2D.h>
#include <Rendering/Text/FTLabel.h>
#include <Util/FTStringUtils.h>
#include <Rendering/FTWindowSizeNode.h>
#include <telemetry.h>
#include <messaging.h>
#include <calibration/ms5611_calibration.h>
#include <calibration/mpu9250_calibration.h>
#include <calibration/adis16405_calibration.h>
#include <Event/Input/FTInputManager.h>


static StateDetailView* s_instance = nullptr;
static const int num_labels = 15;
float values[num_labels];

int mpu9250_update_count = 0;
int state_estimate_update_count = 0;


static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
    if (s_instance == nullptr)
        return false;
    if (packet->header.id == ts_mpu9250_data) {
        mpu9250_data_t* data = (mpu9250_data_t*)packet->payload;

        mpu9250_calibrated_data_t calibrated;
        mpu9250_calibrate_data(data, &calibrated);

        values[3] = calibrated.accel[0];
        values[4] = calibrated.accel[1];
        values[5] = calibrated.accel[2];

        values[6] = calibrated.gyro[0];
        values[7] = calibrated.gyro[1];
        values[8] = calibrated.gyro[2];

        values[9] = calibrated.magno[0];
        values[10] = calibrated.magno[1];
        values[11] = calibrated.magno[2];

        values[12] = mpu9250_get_heading(&calibrated);

        mpu9250_update_count++;
    }
    else if (packet->header.id == ts_ms5611_data) {
        FTAssert(packet->header.length == sizeof(ms5611data_t), "Incorrect Packet Size");
        auto data = (ms5611data_t*)packet->payload;

        values[0] = (float)data->pressure;
        values[1] = ms5611_get_altitude(data);
        values[2] = (float)data->temperature;
    } else if (packet->header.id == ts_state_estimate_data) {
        state_estimate_update_count++;
    }
    return true;
}

MESSAGING_CONSUMER(messaging_consumer, ts_all, ts_all_mask, 0, 0, getPacket, 1024);

StateDetailView::StateDetailView() {
    FTAssert(s_instance == nullptr, "Only one StateDetailView instance can exist at once");
    
    auto camera = std::make_shared<FTCamera2D>();
    camera->setCullFaceEnabled(false);
    setCamera(std::move(camera));

    static const wchar_t* label_names[num_labels] = { 
        L"MS5611 Pressure", L"MS5611 Altitude", L"MS5611 Temperature",
        L"MPU9250 Accel X", L"MPU9250 Accel Y", L"MPU9250 Accel Z", 
        L"MPU9250 Gyro X", L"MPU9250 Gyro Y", L"MPU9250 Gyro Z", 
        L"MPU9250 Magno X", L"MPU9250 Magno Y", L"MPU9250 Magno Z",
        L"MPU9250 Heading", L"MPU9250 Update Rate", L"State Estimate Update Rate"};

    auto window_size_node = std::make_shared<FTWindowSizeNode>();
    window_size_node->setAnchorPoint(glm::vec2(0, -1.0f));
    addChild(window_size_node);

    
	const float x_padding = 350;
    const float y_padding = 30.0f;
    float y = -y_padding;
	float x = 0;

    for (int i = 0; i < num_labels; i++) {
        auto label = std::make_shared<FTLabel>(label_names[i], 6);
        window_size_node->addChild(label);
        label->setPosition(glm::vec2(x + 30, y));
        label->setAnchorPoint(glm::vec2(0, 0.5f));

        label = std::make_shared<FTLabel>(L"0", 6, true);
        window_size_node->addChild(label);
        label->setPosition(glm::vec2(x + 350, y));
        label->setAnchorPoint(glm::vec2(1, 0.5f));

        value_labels_.push_back(label.get());
        y -= y_padding;

        values[i] = 0;
		if (i % 25 == 24) {
			x += x_padding;
			y = -y_padding;
		}
    }

    messaging_consumer_init(&messaging_consumer);

    s_instance = this;

	FTEngine::getInputManager()->getKeyState("ToggleDetails", GLFW_KEY_SEMICOLON)->registerOnPressDelegate(this, &StateDetailView::toggleDetails);
}

StateDetailView::~StateDetailView() {
    FTLog("State Detail View Destroyed");
	FTEngine::getInputManager()->getKeyState("ToggleDetails", GLFW_KEY_SEMICOLON)->unregisterOnPressDelegate(this, &StateDetailView::toggleDetails);
}

void StateDetailView::updateDisplay(const FTUpdateEvent& event) {
    static float accumulator = 1.0f;
    accumulator += (float)event.delta_time_;
    if (accumulator >= 5.0) {
        accumulator -= 5.0f;
        values[13] = mpu9250_update_count / 5.0f;
        values[14] = state_estimate_update_count / 5.0f;
        mpu9250_update_count = 0;
        state_estimate_update_count = 0;
    }

    while (messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok);

    static wchar_t buff[24];
    for (int i = 0; i < num_labels; i++)
        value_labels_[i]->setString(FTWCharUtil::formatString(buff, 24, L"%.2f", values[i]));
}

void StateDetailView::toggleDetails() {
	this->setHidden(!this->getHidden());
}
