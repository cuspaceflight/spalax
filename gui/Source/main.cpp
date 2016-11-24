#include <Frontier.h>
#include "FTEngine.h"
#include <Rendering/MainScene.h>
#include <Rendering/FTDirector.h>
#include <Util/FTFileManager.h>
#include "avionics_config.h"
#include "messaging_all.h"

void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
	if (state == state_error)
		FTLogError("Error in component %i with line %i", component, line);
}


avionics_config_t local_config = {telemetry_origin_avionics_gui, update_handler };

int main() {
    int ret = -1;

    if (FTEngine::setup()) {
        messaging_all_start();

        FTEngine::getFileManager()->addSearchPath("Resources");

        auto scene = std::static_pointer_cast<FTScene>(std::make_shared<MainScene>());
        FTEngine::getDirector()->setCurrentScene(scene);
        scene.reset();

        ret = FTEngine::run();
        FTEngine::cleanup();
    }

    return ret;
}
