#pragma once
#include <Rendering/FTView.h>

class RocketRenderer;
class RocketPathRenderer;
struct state_estimate_t;

class State3DRenderer : public FTView {
public:
	State3DRenderer();
	~State3DRenderer();

	void nextStateEstimate(state_estimate_t& current_state);

protected:
	std::shared_ptr<RocketRenderer> rocket_renderer_;
	std::shared_ptr<RocketPathRenderer> rocket_path_renderer_;
};
