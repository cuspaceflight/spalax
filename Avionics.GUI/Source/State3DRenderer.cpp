#include "State3DRenderer.h"
#include <Rendering/Camera/FTCameraFPS.h>
#include "RocketRenderer.h"
#include "RocketPathRenderer.h"
#include "messaging.h"

extern "C" {
#include <state_estimate.h>
}

static State3DRenderer* s_instance = nullptr;
static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
    if (s_instance == nullptr)
        return false;
    if (packet->header.id == telemetry_id_state_estimate_data) {
        auto data = (state_estimate_t*)packet->payload;
        s_instance->nextStateEstimate(*data);
    }
    return true;
}
MESSAGING_CONSUMER(messaging_consumer, telemetry_source_state_estimation, telemetry_source_state_estimation_mask, 0, 0, getPacket, 1024);

State3DRenderer::State3DRenderer() : rocket_renderer_(new RocketRenderer()), rocket_path_renderer_(new RocketPathRenderer()) {
    FTAssert(s_instance == nullptr, "Only one State3DRenderer instance can exist at once");

    auto camera = std::make_shared<FTCameraFPS>();
    camera->setPosition(glm::vec3(0, -50, 0));
    // The state estimate uses z as up and y as forwards so we adjust to match
    camera->setAxes(glm::vec3(0, 0, 1), glm::vec3(0, 1, 0));
    setCamera(std::move(camera));
    

    addChild(rocket_renderer_);
    addChild(rocket_path_renderer_);
    s_instance = this;

    messaging_consumer_init(&messaging_consumer);
}

State3DRenderer::~State3DRenderer() {
    s_instance = nullptr;
}

void State3DRenderer::nextStateEstimate(state_estimate_t& current_state) {
    glm::quat rotation(current_state.orientation_q[3], current_state.orientation_q[0], current_state.orientation_q[1], current_state.orientation_q[2]);
    rocket_renderer_->setRotationQuaternion(rotation);
    rocket_renderer_->setPosition(glm::vec3(current_state.pos[0], current_state.pos[1], current_state.pos[2]));
    rocket_path_renderer_->nextStateEstimate(current_state);
}

void State3DRenderer::updateDisplay() {
    while (messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok);
}
