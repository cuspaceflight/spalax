#include <Rendering/Shader/FTVertexShaderProgramLighting.h>
#include "RocketRenderer.h"

RocketRenderer::RocketRenderer() {
    auto renderer = std::make_shared<FTCuboid>(glm::vec3(0, 0, 0), glm::vec3(8, 5, 0.25f), glm::vec3(0.6f, 0.6f, 0.6f), FTCuboid::getShaderUtil<FTVertexShaderProgramLighting>());
    renderer->setAnchorPoint(glm::vec3(0.5f, 0.5f, 0.5f));
    addChild(std::move(renderer));
}

RocketRenderer::~RocketRenderer() {
}

void RocketRenderer::nextStateEstimate(const state_estimate_t& current_state) {
    glm::quat rotation(current_state.orientation_q[3], current_state.orientation_q[0], current_state.orientation_q[1], current_state.orientation_q[2]);
    setRotationQuaternion(rotation);
    //setPosition(glm::vec3(current_state.pos[0], current_state.pos[1], current_state.pos[2]));
}
