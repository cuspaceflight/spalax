#include <Frontier.h>
#include "FTEngine.h"
#include <Rendering/MainScene.h>
#include <Rendering/FTDirector.h>
#include <Util/FTFileManager.h>
#include <component_state.h>
#include <util/board_config.h>
#include <thread>
#include "messaging_all.h"
#include "state/state_estimate.h"

void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
	if (state == state_error)
		FTLogError("Error in component %i with line %i", component, line);
}

int main() {
    int ret = -1;

    if (FTEngine::setup()) {
        setBoardConfig(BoardConfigSpalax);

        component_state_start(update_handler, true);
        messaging_all_start();

        std::thread state_estimate(state_estimate_thread, nullptr);

        FTEngine::getFileManager()->addSearchPath("Resources");

        auto scene = std::static_pointer_cast<FTScene>(std::make_shared<MainScene>());
        FTEngine::getDirector()->setCurrentScene(scene);
        scene.reset();

        ret = FTEngine::run();

        state_estimate_terminate();

        state_estimate.join();

        FTEngine::cleanup();
    }

    return ret;
}
