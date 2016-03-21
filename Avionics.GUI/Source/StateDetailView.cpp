#include "StateDetailView.h"
#include <Rendering/Camera/FTCamera2D.h>
#include <Rendering/Text/FTLabel.h>
#include <Rendering/FTDirector.h>
#include <sstream>
#include <Util/FTStringUtils.h>
#include <Rendering/FTWindowSizeNode.h>

extern "C" {
#include <state_estimate.h>
#include <math_utils.h>
}

StateDetailView::StateDetailView() {
    setCamera(std::make_shared<FTCamera2D>());

	const int num_labels = 21;
	static const wchar_t* label_names[num_labels] = { L"Position X", L"Position Y", L"Position Z", L"Velocity X", L"Velocity Y", L"Velocity Z", L"Velocity Mag", L"Accel X", L"Accel Y", L"Accel Z", L"Accel Mag", L"Quat X", L"Quat Y", L"Quat Z", L"Quat W", L"Orientation Euler X", L"Orientation Euler Y", L"Orientation Euler Z", L"Angular Velocity X", L"Angular Velocity Y", L"Angular Velocity Z"};

    auto window_size_node = std::make_shared<FTWindowSizeNode>();
    window_size_node->setAnchorPoint(glm::vec2(0, -1.0f));
    addChild(window_size_node);

    
	const float y_padding = 25.0f;
    float y = -y_padding;
	for (int i = 0; i < num_labels; i++) {
		auto label = std::make_shared<FTLabel>("Fonts/Vera.ttf", label_names[i], 14);
        window_size_node->addChild(label);
		label->setPosition(glm::vec2(30, y));
		label->setAnchorPoint(glm::vec2(0, 0.5f));

        label = std::make_shared<FTLabel>("Fonts/Vera.ttf", L"0", 14, true);
        window_size_node->addChild(label);
		label->setPosition(glm::vec2(250, y));
		label->setAnchorPoint(glm::vec2(1, 0.5f));

		value_labels_.push_back(label.get());
		y -= y_padding;
	}
}

StateDetailView::~StateDetailView() {
	FTLog("State Detail View Destroyed");
}

void StateDetailView::updateDisplay(state_estimate_t& current_state) {
	static wchar_t buff[1024];
	value_labels_[0]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.pos[0]));
	value_labels_[1]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.pos[1]));
	value_labels_[2]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.pos[2]));

	value_labels_[3]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.vel[0]));
	value_labels_[4]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.vel[1]));
	value_labels_[5]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.vel[2]));

	float mag = sqrtf(current_state.vel[0] * current_state.vel[0] + current_state.vel[1] * current_state.vel[1] + current_state.vel[2] * current_state.vel[2]);
	value_labels_[6]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", mag));


	value_labels_[7]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.accel[0]));
	value_labels_[8]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.accel[1]));
	value_labels_[9]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.accel[2]));

	mag = sqrtf(current_state.accel[0] * current_state.accel[0] + current_state.accel[1] * current_state.accel[1] + current_state.accel[2] * current_state.accel[2]);
	value_labels_[10]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", mag));

	value_labels_[11]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.orientation_q[0]));
	value_labels_[12]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.orientation_q[1]));
	value_labels_[13]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.orientation_q[2]));
	value_labels_[14]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.orientation_q[3]));

	float euler[3];
	quat_to_euler(current_state.orientation_q,euler);

	value_labels_[15]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", euler[0] * 57.2957795131f));
	value_labels_[16]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", euler[1] * 57.2957795131f));
	value_labels_[17]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", euler[2] * 57.2957795131f));

	value_labels_[18]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.angular_velocity[0]));
	value_labels_[19]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.angular_velocity[1]));
	value_labels_[20]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.angular_velocity[2]));


	//value_labels_[3]->setString(FTStringUtil<wchar_t>::formatString(buff, 1024, L"%.2f", current_state.vel[0]));

}
