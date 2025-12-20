#pragma once

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <cstdint>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

namespace cr_camera_driver {

class JetsonH265Encoder {
public:
    JetsonH265Encoder(int width, int height, int bitrate = 4000000, int fps = 30, int group_len = 30);
    ~JetsonH265Encoder();

    bool initialize();
    bool encodeRGBFrame(const uint8_t* rgb_data, size_t buffer_size, std::vector<uint8_t>& output);
    std::string getLastError();

private:
    void cleanup();
    bool createPipeline();
    static void ensureGstInitialized();

    GstElement* pipeline_;
    GstElement* appsrc_;
    GstElement* appsink_;
    int width_;
    int height_;
    int bitrate_;
    int fps_;
    int group_len_;
    uint64_t frame_counter_;
    bool initialized_;
    std::string last_error_;
    std::mutex mutex_;
};

} // namespace cr_camera_driver
