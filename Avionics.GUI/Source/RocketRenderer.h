#pragma once
#include <state_estimate_config.h>
#include <Rendering/Primitives/FTCuboid.h>

class RocketRenderer : public FTNode {
public:
    RocketRenderer();
    ~RocketRenderer();
    void nextStateEstimate(const state_estimate_t& current_state);
};
