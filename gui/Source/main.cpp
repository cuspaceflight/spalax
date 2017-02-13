#include <Frontier.h>
#include "FTEngine.h"
#include <Rendering/MainScene.h>
#include <Rendering/FTDirector.h>
#include <Util/FTFileManager.h>
#include <component_state.h>
#include <util/board_config.h>
#include <thread>
#include "messaging_all.h"
#include "Rendering/Text/FTFontCache.h"

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

        FTEngine::getFileManager()->addSearchPath("Resources");
        FTEngine::getDirector()->getFontCache()->loadFontStyle("DefaultText", "Resources/Fonts/Vera.ftfont", glm::vec3(1,1,1));
        FTEngine::getDirector()->getFontCache()->loadFontStyle("DefaultTextPlaceholder", "Resources/Fonts/Vera.ftfont", glm::vec3(0.8,0.8,0.8));

        auto scene = std::static_pointer_cast<FTScene>(std::make_shared<MainScene>());
        FTEngine::getDirector()->setCurrentScene(scene);
        scene.reset();

        ret = FTEngine::run();

        FTEngine::cleanup();
    }

    return ret;
}
