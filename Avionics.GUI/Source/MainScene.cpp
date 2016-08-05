#include "MainScene.h"
#include <Frontier.h>
#include "StateDetailView.h"
#include "State3DRenderer.h"
#include <Event/Engine/FTEngineEventDispatcher.h>
#include <time_utils.h>


MainScene::MainScene() : time_left_after_ticks_(0) {
    state_3d_renderer_ = std::make_shared<State3DRenderer>();
    addView(state_3d_renderer_);

    state_detail_view_ = std::make_shared<StateDetailView>();
    addView(state_detail_view_);

    //reset_state_estimate(&state_estimate_);

    FTEngine::getEventManager()->registerDelegate<FTEngineEventDispatcher>(this, &MainScene::update);
}

MainScene::~MainScene() {
    FTEngine::getEventManager()->unregisterDelegate<FTEngineEventDispatcher>(this, &MainScene::update);
}

void MainScene::update(const FTUpdateEvent& event) {
    state_detail_view_->updateDisplay();
    state_3d_renderer_->updateDisplay();
}


