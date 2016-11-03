#pragma once
#include <Rendering/Mesh/FTMesh.h>
#include <config/telemetry_packets.h>


class RocketPathRenderer : public FTMesh<FTVertex<glm::vec3>> {
public:
    RocketPathRenderer();
    ~RocketPathRenderer();

    void nextStateEstimate(const state_estimate_t& current_state);

protected:
    void addVertex(const FTVertex<glm::vec3>& vertex);

    std::vector<FTVertex<glm::vec3>> vertices_;
};
