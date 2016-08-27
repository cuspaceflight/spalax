#include "CalibrationLabel.h"
#include "messaging.h"
#include "config/telemetry_config.h"
#include <Event/Input/FTInputManager.h>

static CalibrationLabel* s_instance = nullptr;

static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
    if (packet->header.id == telemetry_id_calibration_magno_data) {
        s_instance->handleCalibrationData((magno_calibration_data_t*)packet->payload);
    }
    return true;
}

MESSAGING_CONSUMER(messaging_consumer, telemetry_id_calibration_magno_data, telemetry_source_packet_specific_mask, 0, 0, getPacket, 10);
MESSAGING_PRODUCER(messaging_producer, telemetry_id_calibration_control, sizeof(calibration_control_t), 2);

CalibrationLabel::CalibrationLabel(): FTLabel("Fonts/Vera.ttf", L"",16, true), current_procedure_(calibration_procedure_none) {
    FTAssert(s_instance == nullptr, "Only one instance of CalibrationLabel can exist at once");

    messaging_consumer_init(&messaging_consumer);
    messaging_producer_init(&messaging_producer);
        
    FTEngine::getInputManager()->getKeyState("CalibrationKey", GLFW_KEY_C)->registerOnPressDelegate(this, &CalibrationLabel::onCalibrationKeyPressed);
    
    s_instance = this;
}

CalibrationLabel::~CalibrationLabel() {
    s_instance = nullptr;
    FTEngine::getInputManager()->getKeyState("CalibrationKey", GLFW_KEY_C)->unregisterOnPressDelegate(this, &CalibrationLabel::onCalibrationKeyPressed);
}

void CalibrationLabel::handleCalibrationData(const magno_calibration_data_t* data) {
    if (data->procedure == calibration_procedure_mpu9250_bias) {
        FTLOG("Magno SF: {%i, %i, %i}", data->magno_sf[0], data->magno_sf[1], data->magno_sf[2]);
        FTLOG("Magno Bias: {%i, %i, %i}", data->magno_bias[0], data->magno_bias[1], data->magno_bias[2]);
    }
}

void CalibrationLabel::updateDisplay() const {
    while (messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok);
}

void CalibrationLabel::onCalibrationKeyPressed() {
    calibration_control_t msg;
    if (current_procedure_ == calibration_procedure_none) {
        current_procedure_ = calibration_procedure_mpu9250_bias;
        msg.procedure = current_procedure_;
        messaging_producer_send(&messaging_producer, message_flags_send_over_can, (const uint8_t*)&msg);
        setString(L"Performing MPU9250 Bias Calibration - Please rotate device slowly");
    }
    else if (current_procedure_ == calibration_procedure_mpu9250_bias) {
        current_procedure_ = calibration_procedure_none;
        msg.procedure = current_procedure_;
        messaging_producer_send(&messaging_producer, message_flags_send_over_can, (const uint8_t*)&msg);

        setString(L"");
    }
}
