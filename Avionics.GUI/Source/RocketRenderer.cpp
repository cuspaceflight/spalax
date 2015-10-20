#include "RocketRenderer.h"

static const glm::vec3 colors[6] = {
	glm::vec3(0.5f, 0.5f, 0.5f),
	glm::vec3(0.5f, 0.5f, 0.5f),
	glm::vec3(0.7f, 0.7f, 0.7f),
	glm::vec3(0.4f, 0.4f, 0.4f),
	glm::vec3(0.6f, 0.6f, 0.6f),
	glm::vec3(0.65f, 0.65f, 0.65f)};

RocketRenderer::RocketRenderer() : FTCube(colors) {
	setPosition(glm::vec3(0, 0, 0));
	//setRotationDegrees(glm::vec3(0, 0, 0));
	setScale(glm::vec3(1, 10, 1));
}

RocketRenderer::~RocketRenderer() {
}
