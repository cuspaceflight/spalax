#pragma once
#include "config/telemetry_packets.h"
#include <Rendering/Primitives/FTCuboid.h>

class RocketRenderer : public FTNode {
public:
    RocketRenderer();
    ~RocketRenderer();
    void nextStateEstimate(const state_estimate_t& current_state);
};
