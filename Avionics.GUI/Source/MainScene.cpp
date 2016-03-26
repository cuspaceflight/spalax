#include "MainScene.h"
#include <Frontier.h>
#include "StateDetailView.h"
#include "State3DRenderer.h"
#include <Event/Engine/FTEngineEventDispatcher.h>
#include <time_utils.h>

//#define RUN_SIMULATION // Whether to run the estimators locally or render the on board values (not implemented yet)

MainScene::MainScene() : time_left_after_ticks_(0) {
    state_3d_renderer_ = std::make_shared<State3DRenderer>();
    addView(state_3d_renderer_);

    state_detail_view_ = std::make_shared<StateDetailView>();
    addView(state_detail_view_);

    //reset_state_estimate(&state_estimate_);

#ifdef RUN_SIMULATION
    FTEngine::getEventManager()->registerDelegate<FTEngineEventDispatcher>(this, &MainScene::update);
#endif
}

MainScene::~MainScene() {
#ifdef RUN_SIMULATION
    FTEngine::getEventManager()->unregisterDelegate<FTEngineEventDispatcher>(this, &MainScene::update);
#endif
}

void MainScene::update(const FTUpdateEvent& event) {

    time_left_after_ticks_ += event.delta_time_;

    const float prediction_update_rate = 1000.0f; // At what frequency to run the prediction step
    const int prediction_update_rate_clocks = (int)(CLOCK_FREQUENCY / prediction_update_rate);
    // This is needed because this method only gets called at 60Hz whereas the prediction functions need to be called at ~1kHz
    int num_cycles = (int)(time_left_after_ticks_ * prediction_update_rate);

    time_left_after_ticks_ -= num_cycles/prediction_update_rate;
    //FTLOG("Num Cycles %i", num_cycles);

    // TODO add some sort of accumulating remainder as otherwise this will slowly fall behind
    for (int i = 0; i < num_cycles; ++i) {
        // Bring input data up to date        
        //data_source_->update(prediction_update_rate_clocks, &state_estimate_);
        
        //get_state_estimate(&state_estimate_);
        //FTLog("Running state estimators\n");
    }
    state_detail_view_->updateDisplay(state_estimate_);
    state_3d_renderer_->nextStateEstimate(state_estimate_);
}


