#pragma once
#include <Rendering/Mesh/FTMesh.h>
#include <Rendering/Scene/Transform/FTTransformUtil.h>

extern "C" {
#include <state_estimate.h>
}

class RocketPathRenderer : public FTMesh<FTVertexShaderProgram, FTVertex<glm::vec3>> {
public:
	RocketPathRenderer();
	~RocketPathRenderer();

	void nextStateEstimate(state_estimate_t& current_state);

protected:
	void addVertex(const FTVertex<glm::vec3>& vertex);

	std::vector<FTVertex<glm::vec3>> vertices_;
};
