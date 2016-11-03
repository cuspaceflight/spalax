#pragma once
#include <Rendering/FTScene.h>
#include <config/telemetry_packets.h>

class StateDetailView;
class State3DRenderer;

class MainScene : public FTScene {
public:
    MainScene();
    ~MainScene();

protected:
    state_estimate_t state_estimate_;
    
    double time_left_after_ticks_;

    std::shared_ptr<StateDetailView> state_detail_view_;
    std::shared_ptr<State3DRenderer> state_3d_renderer_;

    void update(const FTUpdateEvent& event);
    //void loadFromBinaryFile(const char* file);
    
};
