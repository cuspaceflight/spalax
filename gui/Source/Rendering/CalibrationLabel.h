#pragma once
#include <Rendering/Text/FTLabel.h>
#include <util/calibration.h>
#include "telemetry_packets.h"

class CalibrationLabel : public FTLabel {
public:
    CalibrationLabel();
    virtual ~CalibrationLabel();
    void handleCalibrationData(const magno_calibration_data_t* data);
    void updateDisplay() const;

private:
    void onCalibrationKeyPressed();
    calibration_procedure_t current_procedure_;
};
