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
static const int num_labels = 39;
float values[num_labels];
bool has_mpu9250_config = false;
mpu9250_config_t mpu9250_config;
bool has_adis16405_config = false;
adis16405_config_t adis16405_config;


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
        values[1] = ms5611_get_altitude((float)data->pressure);
        values[2] = (float)data->temperature;
    } else if (packet->header.id == telemetry_id_state_estimate_data) {
        FTAssert(packet->header.length == sizeof(state_estimate_t), "Incorrect Packet Size");
        auto estimate = (state_estimate_t*)packet->payload;

        glm::vec3 rot = glm::eulerAngles(glm::quat(estimate->orientation_q[3], estimate->orientation_q[0], estimate->orientation_q[1], estimate->orientation_q[2]));
        values[12] = rot.x;
        values[13] = rot.y;
        values[14] = rot.z;

        values[15] = estimate->angular_velocity[0];
        values[16] = estimate->angular_velocity[1];
        values[17] = estimate->angular_velocity[2];

//        values[18] = estimate->pos[0];
//        values[19] = estimate->pos[1];
//        values[20] = estimate->pos[2];
//
//        values[21] = estimate->vel[0];
//        values[22] = estimate->vel[1];
//        values[23] = estimate->vel[2];
//
//        values[24] = estimate->accel[0];
//        values[25] = estimate->accel[1];
//        values[26] = estimate->accel[2];
    } else if (packet->header.id == telemetry_id_state_estimate_status) {
        FTAssert(packet->header.length == sizeof(state_estimate_status_t), "Incorrect Packet Size");
        auto status = (state_estimate_status_t*)packet->payload;
        values[27] = (float)status->number_prediction_steps / status->sample_time;
        values[28] = (float)status->number_update_steps / status->sample_time;
    } else if (packet->header.id == telemetry_id_adis16405_config) {
		if (has_adis16405_config)
			return true;
		FTAssert(packet->header.length == sizeof(adis16405_config_t), "Incorrect Packet Size");
		adis16405_config_t* data = (adis16405_config_t*)packet->payload;
		adis16405_config = *data;
		has_adis16405_config = true;
    } else if (packet->header.id == telemetry_id_adis16405_data) {
		if (!has_adis16405_config)
			return true;
		FTAssert(packet->header.length == sizeof(adis16405_data_t), "Incorrect Packet Size");
		adis16405_data_t* data = (adis16405_data_t*)packet->payload;
		adis16405_calibrated_data_t cal;
		adis16405_calibrate_data(&adis16405_config, data, &cal);

		values[29] = cal.supply;
		values[30] = cal.gyro[0];
		values[31] = cal.gyro[1];
		values[32] = cal.gyro[2];
		values[33] = cal.accel[0];
		values[34] = cal.accel[1];
		values[35] = cal.accel[2];
		values[36] = cal.magno[0];
		values[37] = cal.magno[1];
		values[38] = cal.magno[2];
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
        L"MPU9250 Magno X", L"MPU9250 Magno Y", L"MPU9250 Magno Z",
        L"Estimate Rotation X", L"Estimate Rotation Y", L"Estimate Rotation Z",
        L"Estimate Ang. Velocity X", L"Estimate Ang. Velocity Y", L"Estimate Ang. Velocity Z", 
        L"Estimate Position X", L"Estimate Position Y", L"Estimate Position Z",
        L"Estimate Velocity X", L"Estimate Velocity Y", L"Estimate Velocity Z",
        L"Estimate Accel X", L"Estimate Accel Y", L"Estimate Accel Z",
        L"Estimate Prediction Rate", L"Estimate Update Rate",
		L"ADIS16405 Supply",
		L"ADIS16405 Gyro X", L"ADIS16405 Gyro Y", L"ADIS16405 Gyro Z",
		L"ADIS16405 Accel X", L"ADIS16405 Accel Y", L"ADIS16405 Accel Z",
		L"ADIS16405 Magno X", L"ADIS16405 Magno Y", L"ADIS16405 Magno Z"};

    auto window_size_node = std::make_shared<FTWindowSizeNode>();
    window_size_node->setAnchorPoint(glm::vec2(0, -1.0f));
    addChild(window_size_node);

    
	const float x_padding = 350;
    const float y_padding = 30.0f;
    float y = -y_padding;
	float x = 0;

    for (int i = 0; i < num_labels; i++) {
        auto label = std::make_shared<FTLabel>("Fonts/Vera.ftfont", label_names[i], 6);
        window_size_node->addChild(label);
        label->setPosition(glm::vec2(x + 30, y));
        label->setAnchorPoint(glm::vec2(0, 0.5f));
        label->setFillColor(glm::vec3(1, 1, 1));

        label = std::make_shared<FTLabel>("Fonts/Vera.ftfont", L"0", 6, true);
        window_size_node->addChild(label);
        label->setPosition(glm::vec2(x + 350, y));
        label->setAnchorPoint(glm::vec2(1, 0.5f));
        label->setFillColor(glm::vec3(1, 1, 1));

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

void StateDetailView::updateDisplay() {
    while (messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok);

    static wchar_t buff[24];
    for (int i = 0; i < num_labels; i++)
        value_labels_[i]->setString(FTWCharUtil::formatString(buff, 24, L"%.2f", values[i]));
}

void StateDetailView::toggleDetails() {
	this->setHidden(!this->getHidden());
}
