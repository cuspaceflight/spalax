#include <Frontier.h>
#include "FTEngine.h"
#include <MainScene.h>
#include <Rendering/FTDirector.h>
#include <Util/FTFileManager.h>


#include "avionics_config.h"
#include <messaging.h>
#include <checksum.h>
#include <component_state.h>
#include <SerialDriver.h>
#include <CanSerialDriver.h>
#include <Rendering/Text/FTFont.h>
#include <can_interface.h>

#ifdef _WIN32
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
// Use for debugging allocations
//static int breakAlloc = (_crtBreakAlloc = 157);
#endif

void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
	if (state == state_error)
		FTLogError("Error in component %i with line %i", component, line);
}


const avionics_config_t local_config = {telemetry_origin_avionics_gui, update_handler };

void rocket_main() {
    component_state_start();
    checksum_init();
    telemetry_allocator_start();
    messaging_start();

	can_interface_init();
    
    
}

int main() {
    int ret = -1;

#ifdef _WIN32
    // We can't check manually as it returns false positives for static variables with custom initializers
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    if (FTEngine::setup()) {
        
        //FTEngine::setWindowsSize(glm::tvec2<int>(1024, 1024));

        rocket_main();

        //auto driver = std::make_unique<SerialDriver>("COM8", 38400);
		auto driver = std::make_unique<CanSerialDriver>("COM9", 38400);

        FTEngine::getFileManager()->addSearchPath("Resources");

        auto scene = std::static_pointer_cast<FTScene>(std::make_shared<MainScene>());
        FTEngine::getDirector()->setCurrentScene(scene);
        scene.reset();

        ret = FTEngine::run();
        FTEngine::cleanup();
    }

    return ret;
}
