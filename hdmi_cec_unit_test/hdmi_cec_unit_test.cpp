/*
* Copyright (c) 2017 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Confidential and Proprietary - Qualcomm Technologies, Inc.
*/
#include <gtest/gtest.h>
#include <cutils/log.h>
#include <errno.h>
#include <fcntl.h>
#include <hardware/hdmi_cec.h>
#include <utils/Trace.h>
#include <cstdlib>

namespace testing {
namespace internal {
enum GTestColor {
  COLOR_DEFAULT,
};
extern void ColoredPrintf(GTestColor color, const char* fmt, ...);
}
}

#define PRINTF(...)  do {testing::internal::ColoredPrintf(testing::internal::COLOR_DEFAULT, \
"[          ] "); \
testing::internal::ColoredPrintf(testing::internal::COLOR_DEFAULT, __VA_ARGS__);\
} while (0);

class HDMICECTest: public ::testing::Test {
 public:
  HDMICECTest() {
  }
  const hw_module_t *module;
  hw_device_t* device;
  hdmi_cec_device_t* mdevice;
};

static const uint32_t vendorid_value = 0xA47733;
static const uint32_t version_value = 0x4;

TEST_F(HDMICECTest, OpenCloseTest) {
    PRINTF("Opening module...\n");
    ASSERT_EQ(0, hw_get_module(HDMI_CEC_HARDWARE_MODULE_ID,
        &module)) << "Failed to open hwc module";

    PRINTF("First open\n");
    ASSERT_EQ(0, module->methods->open(module, HDMI_CEC_HARDWARE_INTERFACE,
        &device)) << "Failed first open";
    PRINTF("First close\n");
    ASSERT_EQ(0, device->close(device)) << "Failed first close";

    PRINTF("Second open\n");
    ASSERT_EQ(0, module->methods->open(module, HDMI_CEC_HARDWARE_INTERFACE,
        &device)) << "Failed first open";
    PRINTF("Second close\n");
    ASSERT_EQ(0, device->close(device)) << "Failed first close";

    mdevice = reinterpret_cast<hdmi_cec_device*>(device);
}

TEST_F(HDMICECTest, getvendor) {
    uint32_t vendorId = 0;
    mdevice->get_vendor_id(mdevice, &vendorId);
    ASSERT_EQ(vendorid_value, vendorId) << "failed to get_vendor_id";
    PRINTF("getvendor returned = %d\n", vendorId);
}

TEST_F(HDMICECTest, getversion) {
    int version = 0;
    mdevice->get_version(mdevice, &version);
    ASSERT_EQ(version_value, version) << "failed to get_version";
    PRINTF("getversion returned = %d \n", version);
}

TEST_F(HDMICECTest, isconnected) {
    int port = 0;
    ASSERT_LT(0, mdevice->is_connected(mdevice, port)) << "Failed is_connected()";
}

TEST_F(HDMICECTest, getphyaddr) {
    uint16_t addr;
    ASSERT_EQ(0, mdevice->get_physical_address(mdevice,
        &addr)) << "failed to get physical address";
    ASSERT_GT(0xFFFF,addr) << "physical address is greater than the expected range";
    PRINTF("Physical address is %d. Success\n", addr);
}

TEST_F(HDMICECTest, addlogicaladdr) {
    ASSERT_EQ(0, mdevice->add_logical_address(mdevice,
        static_cast<cec_logical_address_t>(4))) << "failed to  add logical address";
    PRINTF("add logical address success...\n");
}

TEST_F(HDMICECTest, getportinfo) {
    hdmi_port_info* ports;
    int numPorts;
    mdevice->get_port_info(mdevice, &ports, &numPorts);
    PRINTF("get port info success...\n");
}

TEST_F(HDMICECTest, setoption) {
    int value = 1;
    int flag = 2;
    mdevice->set_option(mdevice, flag, value);
    PRINTF("HDMI_OPTION_ENABLE_CEC enable successfully...\n");
}

TEST_F(HDMICECTest, setaudiochannel) {
    int port = 1;
    int enable = 1;
    mdevice->set_audio_return_channel(mdevice, port, enable);
    PRINTF("set audio channel success...\n");
}

TEST_F(HDMICECTest, sendmessge) {
    cec_message_t message;
    message.initiator = static_cast<cec_logical_address_t>(4);
    message.destination = static_cast<cec_logical_address_t>(0);
    message.body[0] = 0x36;
    message.length = 1;
    PRINTF("Sending STANDBY command ...\n");
    ASSERT_EQ(0, mdevice->send_message(mdevice,
        &message)) << "failed to send STANDBY Command";
    sleep(10);

    message.body[0] = 0x0D;
    message.length = 1;
    PRINTF("Sending Text view on command ...\n");
    ASSERT_EQ(0, mdevice->send_message(mdevice,
        &message)) << "failed to send Text View ON";

    message.body[0] = 0x82;
    message.body[1] = 0x20;
    message.body[2] = 0x00;
    message.length = 3;
    PRINTF("Sending Active source command ...\n");
    ASSERT_EQ(0, mdevice->send_message(mdevice,
        &message)) << "failed to send WAKEUP Command";
}

TEST_F(HDMICECTest, clearlogicaladdr) {
    mdevice->clear_logical_address(mdevice);
    PRINTF("clear logical address success...\n");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
