#pragma once
#include <config/telemetry_packets.h>
#include <Rendering/Primitives/FTCuboid.h>

class VectorRenderer : public FTCuboid {
public:
    explicit VectorRenderer(const glm::vec3& color);
    ~VectorRenderer();
    void renderVector(const glm::vec3& vector);
};
