﻿#include <Frontier.h>
#include "FTEngine.h"
#include <Rendering/MainScene.h>
#include <Rendering/FTDirector.h>
#include <Util/FTFileManager.h>
#include <component_state.h>
#include <util/board_config.h>
#include <thread>
#include <state/state_estimate.h>
#include "messaging_all.h"
#include "Rendering/Text/FTFontCache.h"

void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
	if (state == state_error)
		FTLogError("Error in component %i with line %i", component, line);
}

int main(int argc, char** argv) {
    int ret = -1;

    if (FTEngine::setup()) {
        setBoardConfig(BoardConfigSpalax);
        glClearColor(1,1,1, 0.0f);

        component_state_start(update_handler, true);
        messaging_all_start();

        const bool state_estimates = argc > 1 && std::string(argv[1]) == "-s";

        std::thread* state_estimate = nullptr;

        if (state_estimates)
            state_estimate = new std::thread(state_estimate_thread, nullptr);

        FTEngine::getFileManager()->addSearchPath("Resources");
        FTEngine::getDirector()->getFontCache()->loadFontStyle("DefaultText", "Resources/Fonts/Vera.ftfont", glm::vec3(0,0,0));
        FTEngine::getDirector()->getFontCache()->loadFontStyle("DefaultTextPlaceholder", "Resources/Fonts/Vera.ftfont", glm::vec3(0.2,0.2,0.2));

        auto scene = std::static_pointer_cast<FTScene>(std::make_shared<MainScene>());
        FTEngine::getDirector()->setCurrentScene(scene);
        scene.reset();

        ret = FTEngine::run();

        if (state_estimates) {
            state_estimate_terminate();
            state_estimate->join();
            delete state_estimate;
        }


        FTEngine::cleanup();
    }

    return ret;
}
