#include "gtest/gtest.h"
#include "gmock/gmock.h"

#ifdef _WIN32
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>

// Memory leak detector from https://github.com/ymx/gtest_mem
class MemoryLeakDetector : public testing::EmptyTestEventListener {

public:
    virtual void OnTestStart(const testing::TestInfo&) override {
        _CrtMemCheckpoint(&memState_);
    }

    virtual void OnTestEnd(const testing::TestInfo& test_info) override {
        if (test_info.result()->Passed()) {
            _CrtMemState stateNow, stateDiff;
            _CrtMemCheckpoint(&stateNow);
            int diffResult = _CrtMemDifference(&stateDiff, &memState_, &stateNow);
            if (diffResult) {
                //_CrtDumpMemoryLeaks();
                FAIL() << "Memory leak of " << stateDiff.lSizes[1] << " byte(s) detected.";
            }
        }
    }

private:
    _CrtMemState memState_;
};

class MutexAllocator {
public:
    // The function must have a type in order to create the g_gmock_mutex in UntypedFunctionMockerBase
    MOCK_METHOD1(doStuff, void(float));
};


#endif

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);


#ifdef _WIN32
    // We call log in order to allocate the g_log_mutex
    testing::internal::Log(testing::internal::kWarning, "Allocating Mutexes\n", -1);

    auto mutexAllocator = new MutexAllocator();
    // We need an expectation to allocate the g_gmock_implicit_sequence
    // NB as this is allocated in a ThreadLocal variable this fix might not work for multithreaded tests
    EXPECT_CALL(*mutexAllocator, doStuff(testing::_)).Times(0);
    delete mutexAllocator;

    testing::UnitTest::GetInstance()->listeners().Append(new MemoryLeakDetector());
#endif

    return RUN_ALL_TESTS();

}
