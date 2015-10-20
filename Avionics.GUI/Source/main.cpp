#include "FTEngine.h"
#include <MainScene.h>
#include <Rendering/FTDirector.h>
#include <Util/FTFileManager.h>

#ifdef _WIN32
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#endif



int main() {
    int ret = -1;
    if (FTEngine::setup()) {
        FTEngine::getFileManager()->addSearchPath("Resources");

        auto scene = std::static_pointer_cast<FTScene>(std::make_shared<MainScene>());
        FTEngine::getDirector()->setCurrentScene(scene);
        scene.reset();

        ret = FTEngine::run();
        FTEngine::cleanup();
    }
#ifdef _WIN32
    FTAssert(_CrtDumpMemoryLeaks() == 0, "Leaks Detected");
#endif
    return ret;
}
