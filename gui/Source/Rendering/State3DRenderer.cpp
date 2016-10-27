#include "State3DRenderer.h"
#include <Rendering/Camera/FTCameraFPS.h>
#include "RocketRenderer.h"
#include "RocketPathRenderer.h"
#include "messaging.h"
#include <state_estimate.h>
#include <Rendering/Primitives/FTLine.h>
#include <mpu9250_config.h>

static State3DRenderer* s_instance = nullptr;
static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
    if (s_instance == nullptr)
        return false;
    return s_instance->handlePacket(packet);
    
}
MESSAGING_CONSUMER(messaging_consumer, telemetry_source_all, telemetry_source_all_mask, 0, 0, getPacket, 1024);

State3DRenderer::State3DRenderer() : 
    rocket_renderer_(new RocketRenderer()), 
    rocket_path_renderer_(new RocketPathRenderer()),
    mag_renderer_(new VectorRenderer(glm::vec3(1,0,0))),
    accel_renderer_(new VectorRenderer(glm::vec3(0, 1, 0))) {
    FTAssert(s_instance == nullptr, "Only one State3DRenderer instance can exist at once");

    auto camera = std::make_shared<FTCameraFPS>();
    camera->setPosition(glm::vec3(0, -50, 0));
    // The state estimate uses z as up and y as forwards so we adjust to match
    camera->setAxes(glm::vec3(0, 0, 1), glm::vec3(0, 1, 0));
    setCamera(std::move(camera));
    

    addChild(rocket_renderer_);
    addChild(rocket_path_renderer_);

    addChild(mag_renderer_);
    rocket_renderer_->addChild(accel_renderer_);
    s_instance = this;

    messaging_consumer_init(&messaging_consumer);
}

State3DRenderer::~State3DRenderer() {
    s_instance = nullptr;
}

bool State3DRenderer::handlePacket(const telemetry_t* packet) const {
    if (packet->header.id == telemetry_id_state_estimate_data) {
        auto data = (state_estimate_t*)packet->payload;
        rocket_renderer_->nextStateEstimate(*data);
        rocket_path_renderer_->nextStateEstimate(*data);
    } else if (packet->header.id == telemetry_id_mpu9250_data) {
        auto data = (mpu9250_data_t*)packet->payload;
        mag_renderer_->renderVector(glm::vec3(data->magno[0], data->magno[1], data->magno[2]));
        accel_renderer_->renderVector(glm::vec3(data->accel[0], data->accel[1], data->accel[2]));
    }
    return true;
}

void State3DRenderer::updateDisplay() {
    while (messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok);
}
