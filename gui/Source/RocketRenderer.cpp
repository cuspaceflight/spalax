#include "RocketRenderer.h"

static const glm::vec3 colors[6] = {
    glm::vec3(0.5f, 0.5f, 0.5f),
    glm::vec3(0.5f, 0.5f, 0.5f),
    glm::vec3(0.7f, 0.7f, 0.7f),
    glm::vec3(0.4f, 0.4f, 0.4f),
    glm::vec3(0.6f, 0.6f, 0.6f),
    glm::vec3(0.65f, 0.65f, 0.65f)};

RocketRenderer::RocketRenderer() {
    auto renderer = std::make_shared<FTCuboid>(glm::vec3(0, 0, 0), glm::vec3(1, 1, 10), colors);
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
