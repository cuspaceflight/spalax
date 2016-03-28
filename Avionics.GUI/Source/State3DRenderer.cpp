#include "State3DRenderer.h"
#include <Rendering/Camera/FTCameraFPS.h>
#include "RocketRenderer.h"
#include "RocketPathRenderer.h"

extern "C" {
#include <state_estimate.h>
}


State3DRenderer::State3DRenderer() : rocket_renderer_(new RocketRenderer()), rocket_path_renderer_(new RocketPathRenderer()) {
    auto camera = std::make_shared<FTCameraFPS>();

    camera->setPosition(glm::vec3(0, 0, -50));
    camera->setRotationDegrees(glm::vec2(180, 0));
    setCamera(std::move(camera));
    

    addChild(rocket_renderer_);
    addChild(rocket_path_renderer_);
}

State3DRenderer::~State3DRenderer() {
    
}

void State3DRenderer::nextStateEstimate(state_estimate_t& current_state) {
    
    
    glm::quat rotation(current_state.orientation_q[3], current_state.orientation_q[0], current_state.orientation_q[1], current_state.orientation_q[2]);
    //rocket_renderer_->setRotationRadians(glm::vec3(current_state.orientation_euler[0], current_state.orientation_euler[1], current_state.orientation_euler[2]));
    rocket_renderer_->setRotationQuaternion(rotation);
    //rocket_renderer_->setPosition(glm::vec3(current_state.pos[0], current_state.pos[1], current_state.pos[2]));
    //rocket_path_renderer_->nextStateEstimate(current_state);
}