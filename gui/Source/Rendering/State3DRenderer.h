#pragma once
#include <Rendering/FTView.h>
#include <Rendering/Primitives/FTCube.h>
#include <Rendering/Primitives/FTCuboid.h>
#include "VectorRenderer.h"

class RocketRenderer;
class RocketPathRenderer;
struct state_estimate_t;
struct telemetry_t;

class State3DRenderer : public FTView {
public:
    State3DRenderer();
    ~State3DRenderer();


    bool handlePacket(const telemetry_t* packet) const;
    void updateDisplay();

protected:
    std::shared_ptr<RocketRenderer> rocket_renderer_;
    std::shared_ptr<RocketPathRenderer> rocket_path_renderer_;
    std::shared_ptr<VectorRenderer> mag_renderer_;
    std::shared_ptr<VectorRenderer> accel_renderer_;
};
