// ak8963-reg.h contains human-readable name to register mapping for the AK8963
// which is physically located within the AK8963 package.
#pragma once
#ifndef AK8963_REG_H
#define AK8963_REG_H

// AK8963 registers
enum {
    // Read-only
    AK8963_REG_WIA = 0,
    AK8963_REG_INFO,
    AK8963_REG_ST1,
    AK8963_REG_HXL,
    AK8963_REG_HXH,
    AK8963_REG_HYL,
    AK8963_REG_HYH,
    AK8963_REG_HZL,
    AK8963_REG_HZH,
    AK8963_REG_ST2,

    // Read/write
    AK8963_REG_CNTL1,
    AK8963_REG_CNTL2,
    AK8963_REG_ASTC,
    AK8963_REG_TS1,
    AK8963_REG_TS2,
    AK8963_REG_I2CDIS,

    // Read-only
    AK8963_REG_ASAX,
    AK8963_REG_ASAY,
    AK8963_REG_ASAZ,
    AK8963_REG_RSV,
};

// The AK8963 device id
#define AK8963_DEVICE_ID 0x48

#endif // AK8963_REG_H
