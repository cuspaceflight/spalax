#include "FileDataSource.h"
#include <fstream>
#include <stdio.h>
#include <Util/FTMath.h>
#include <FTEngine.h>
#include <Util/FTFileManager.h>

extern "C" {
#include "calibration.h"
}


FileDataSource::FileDataSource(const char* filename) : simulation_index_(0), data_input_(nullptr) {
    loadFromBinaryFile(filename);
}


FileDataSource::~FileDataSource() {
    if (data_input_ != nullptr)
        delete data_input_;
}

void FileDataSource::update(uint64_t dt, state_estimate_t* state) {
    DataSource::update(dt, state);
    if (simulation_index_ == num_samples_)
        return;
    //if (FTInputManager::getSharedInstance()->isKeyDown(KeyNameUp))
    //    return;
    //FTLOG("Simulation Time %lu", simulation_time_);

    // Bring input data up to date
    while (true) {
        if (simulation_index_ == num_samples_)
            break;
        telemetry_t data = data_input_[simulation_index_];
        
        if (isPacketInFuture(data))
            break;
        handlePacket(data);
        ++simulation_index_;
    }

}

void FileDataSource::loadFromBinaryFile(const char* file) {
    if (data_input_ != nullptr) {
        delete[] data_input_;
        data_input_ = nullptr;
    }

    auto path = FTEngine::getFileManager()->getPathToFile(file);
    FTAssert(path != "", "Data source not found");

    std::ifstream file_stream(path, std::ios::in | std::ios::binary | std::ios::ate);
    if (file_stream.is_open()) {
        file_stream.seekg(0, std::ios::end);
        std::streamoff size = file_stream.tellg();

        if (size % sizeof(telemetry_t) != 0) {
            file_stream.close();
            FTLogError("File stream size not of a valid size - it is %i", size);
            return;
        }
        file_stream.seekg(0, std::ios::beg);

        // Useful for debugging where you only want a snippet of the data
        size_t max_samples = -1;
        num_samples_ = (size_t)FTMIN(max_samples, size / sizeof(telemetry_t));
        FTAssert(num_samples_ > 0, "No samples found!");
        data_input_ = new telemetry_t[num_samples_];
        FTLog("Loading Telemetry data");
        file_stream.read((char*)data_input_, num_samples_ * sizeof(telemetry_t));
        FTLog("Finished Loading Telemetry data");
        file_stream.close();

        simulation_index_ = 0;
        // Skip forward to the configuration values (as all prior data is meaningless)
        while (simulation_index_ != num_samples_) {
            if (data_input_[simulation_index_].source == 0x1 && data_input_[simulation_index_].tag == 0x1)
                break;
            ++simulation_index_;
        }
        FTAssert(simulation_index_ != num_samples_, "No accelerometer config data found");

        // Find ignition state
        while (simulation_index_ != num_samples_) {
            /*if (data_input_[simulation_index_].channel_ == 0x20) {
                float cal[3];
                calibrate_accel(data_input_[simulation_index_].int16_data_, cal);

                FTLOG("%f %f %f", cal[0], cal[1], cal[2]);

            } else*/ if (data_input_[simulation_index_].source == 0x4 && data_input_[simulation_index_].tag == 0 && data_input_[simulation_index_].int32_data[1] == 1) {
                break;
            }
            ++simulation_index_;
        }
        FTAssert(simulation_index_ != num_samples_, "No ignition state found!");

        setStartOffset(data_input_[simulation_index_].timestamp);
        //simulation_time_ = data_input_[simulation_index_].timestamp_;
        //last_packet_time_ = data_input_[simulation_index_].timestamp_;
    }
    else {
        FTAssert(false, "Couldn't open input file at path %s", path.c_str());
        return;
    }
}