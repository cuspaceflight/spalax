#include "VectorRenderer.h"

VectorRenderer::VectorRenderer(const glm::vec3& color) 
    : FTCuboid(glm::vec3(0, 0, 0), glm::vec3(0.25f, 0.25f, 15), color) {

}

VectorRenderer::~VectorRenderer() {
}

void VectorRenderer::renderVector(const glm::vec3& delta) {
    auto length = glm::length(delta);
    auto norm_delta = delta / length;

    glm::vec3 up(0, 0, 1);
    glm::quat q;

    auto cross = glm::cross(up, norm_delta);
    q.x = cross.x;
    q.y = cross.y;
    q.z = cross.z;
    q.w = 1 + glm::dot(up, norm_delta);

    q = glm::normalize(q);

    setRotationQuaternion(q);
}
