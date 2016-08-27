#include "RocketPathRenderer.h"
#include <Rendering/Shader/FTShaderCache.h>

RocketPathRenderer::RocketPathRenderer() {
    setShaderProgram<FTVertexShaderProgram>();
    loadEmptyMesh(100);
    setPrimitiveType(GL_LINE_STRIP);

    addVertex(FTVertex<glm::vec3>());
}

RocketPathRenderer::~RocketPathRenderer() {
    
}

void RocketPathRenderer::nextStateEstimate(const state_estimate_t& current_state) {
//    FTVertex<glm::vec3> vertex;
//    vertex.position_.x = current_state.pos[0];
//    vertex.position_.y = current_state.pos[1];
//    vertex.position_.z = current_state.pos[2];
//    if (vertices_.back() == vertex)
//        return;
//
//    addVertex(vertex);
}

void RocketPathRenderer::addVertex(const FTVertex<glm::vec3>& vertex) {
    vertices_.push_back(vertex);
    if (num_vertices_ < max_num_vertices_)
        modifyVertices(num_vertices_, 1, &vertex);
    else {
        resizeVertexBuffer(max_num_vertices_ * 2, nullptr);
        modifyVertices(0, (GLuint)vertices_.size(), vertices_.data());
    }
}