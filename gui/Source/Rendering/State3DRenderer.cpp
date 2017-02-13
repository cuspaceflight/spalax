#include "State3DRenderer.h"
#include <Rendering/Camera/FTCameraFPS.h>
#include <calibration/mpu9250_calibration.h>
#include "RocketRenderer.h"
#include "RocketPathRenderer.h"
#include "messaging.h"

static State3DRenderer* s_instance = nullptr;
static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
    if (s_instance == nullptr)
        return false;
    return s_instance->handlePacket(packet);
    
}
MESSAGING_CONSUMER(messaging_consumer, ts_all, ts_all_mask, 0, 0, getPacket, 1024);

State3DRenderer::State3DRenderer() : 
    rocket_renderer_(new RocketRenderer()), 
    rocket_path_renderer_(new RocketPathRenderer()),
    mag_renderer_(new VectorRenderer(glm::vec3(1,0,0))),
    accel_renderer_(new VectorRenderer(glm::vec3(0, 1, 0))) {
    FTAssert(s_instance == nullptr, "Only one State3DRenderer instance can exist at once");

    auto camera = std::make_shared<FTCameraFPS>();
    camera->setPosition(glm::vec3(-30, 0, 0));
    // The state estimate uses z as up and x as forwards so we adjust to match
    camera->setAxes(glm::vec3(0, 0, 1), glm::vec3(1, 0, 0));
    setCamera(std::move(camera));
    

    addChild(rocket_renderer_);
    addChild(rocket_path_renderer_);

    addChild(mag_renderer_);
    addChild(accel_renderer_);
    s_instance = this;

    messaging_consumer_init(&messaging_consumer);

    LightDescriptor light;
    light.position = glm::normalize(glm::vec4(1, 2, 0.2, 0));
    light.ambientCoefficient = 0.1f;
    light.intensity = glm::vec3(0.2, 0.2, 0.2);
    //light.attenuation = 0.01f;
    //light.coneAngle = 15.0f;
    //light.coneDirection = glm::normalize(glm::vec3(0,1,-0.5));
    light_manager_->addLight(light);

    light.position = glm::normalize(glm::vec4(-1, 2, -0.2, 0));
    light.ambientCoefficient = 0.1f;
    light.intensity = glm::vec3(0.2, 0.2, 0.2);
    //light.attenuation = 0.01f;
    //light.coneAngle = 15.0f;
    //light.coneDirection = glm::normalize(glm::vec3(0,1,-0.5));
    light_manager_->addLight(light);
}

State3DRenderer::~State3DRenderer() {
    s_instance = nullptr;
}

bool State3DRenderer::handlePacket(const telemetry_t* packet) const {
    if (packet->header.id == ts_state_estimate_data) {
        auto data = (state_estimate_t*)packet->payload;
        rocket_renderer_->nextStateEstimate(*data);
        rocket_path_renderer_->nextStateEstimate(*data);
    } else if (packet->header.id == ts_mpu9250_data) {
        auto data = (mpu9250_data_t*)packet->payload;

        mpu9250_calibrated_data_t calibrated;
        mpu9250_calibrate_data(data, &calibrated);

        mag_renderer_->renderVector(glm::vec3(calibrated.magno[0], calibrated.magno[1], calibrated.magno[2]));
        accel_renderer_->renderVector(glm::vec3(calibrated.accel[0], calibrated.accel[1], calibrated.accel[2]));
    }
    return true;
}

void State3DRenderer::updateDisplay() {
    while (messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok);
}
