#pragma once
#include <Rendering/Scene/FTView.h>
#include <Rendering/Scene/FTNode.h>

class FTLabel;
struct state_estimate_t;

class StateDetailView : public FTView {
public:
	StateDetailView();
	~StateDetailView();

	void updateDisplay(state_estimate_t& current_state);

protected:
	std::vector<FTLabel*> value_labels_;
};
