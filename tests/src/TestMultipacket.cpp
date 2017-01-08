#include <gmock/gmock.h>
#include <can_interface.h>

TEST(TestMultipacket, TestMultipacket) {
    can_interface_check_multipacket_definitions();
}