#pragma once
#include <Rendering/Scene/FTScene.h>

extern "C" {
#include <state_estimate.h>
#include <telemetry.h>
}


class StateDetailView;
class State3DRenderer;
enum KeyName;
enum KeyState;
class DataSource;

class MainScene : public FTScene {
public:
	MainScene();
	~MainScene();

protected:
	state_estimate_t state_estimate_;
	std::unique_ptr<DataSource> data_source_;
	
	double time_left_after_ticks_;

	std::shared_ptr<StateDetailView> state_detail_view_;
    std::shared_ptr<State3DRenderer> state_3d_renderer_;

	void update(const FTUpdateEvent& event);
	//void loadFromBinaryFile(const char* file);
	
};
