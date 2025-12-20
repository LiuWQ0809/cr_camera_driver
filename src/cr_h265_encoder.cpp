#include "cr_camera_driver/cr_h265_encoder.hpp"
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <cstring>
#include <mutex>
#include <sstream>

namespace cr_camera_driver {

JetsonH265Encoder::JetsonH265Encoder(int width, int height, int bitrate, int fps, int group_len)
    : pipeline_(nullptr), appsrc_(nullptr), appsink_(nullptr),
      width_(width), height_(height), bitrate_(bitrate), fps_(fps), group_len_(group_len),
      frame_counter_(0), initialized_(false) {
    ensureGstInitialized();
}

JetsonH265Encoder::~JetsonH265Encoder() {
    std::lock_guard<std::mutex> guard(mutex_);
    cleanup();
}

bool JetsonH265Encoder::initialize() {
    std::lock_guard<std::mutex> guard(mutex_);
    if (initialized_) {
        return true;
    }
    if (!createPipeline()) {
        return false;
    }
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        last_error_ = "Failed to start H265 encoder pipeline";
        cleanup();
        return false;
    }
    initialized_ = true;
    return true;
}

bool JetsonH265Encoder::encodeRGBFrame(const uint8_t* rgb_data, size_t buffer_size, std::vector<uint8_t>& output) {
    std::lock_guard<std::mutex> guard(mutex_);
    output.clear();

    if (!initialized_ || !pipeline_ || !appsrc_ || !appsink_) {
        last_error_ = "Encoder pipeline is not initialized";
        return false;
    }

    if (!rgb_data || buffer_size == 0) {
        last_error_ = "Invalid RGB input buffer";
        return false;
    }

    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, buffer_size, nullptr);
    if (!buffer) {
        last_error_ = "Failed to allocate GstBuffer";
        return false;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        gst_buffer_unref(buffer);
        last_error_ = "Unable to map GstBuffer for writing";
        return false;
    }
    memcpy(map.data, rgb_data, buffer_size);
    gst_buffer_unmap(buffer, &map);

    GstClockTime pts = gst_util_uint64_scale(frame_counter_, GST_SECOND, fps_);
    GST_BUFFER_PTS(buffer) = pts;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1, GST_SECOND, fps_);
    frame_counter_++;

    GstFlowReturn flow_ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    if (flow_ret != GST_FLOW_OK) {
        gst_buffer_unref(buffer);
        last_error_ = "Failed to push buffer into appsrc";
        return false;
    }

    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 10 * GST_MSECOND);
    if (!sample) {
        return true;
    }

    GstBuffer* out_buffer = gst_sample_get_buffer(sample);
    GstMapInfo out_map;
    if (!gst_buffer_map(out_buffer, &out_map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        last_error_ = "Unable to map encoded buffer";
        return false;
    }

    output.assign(out_map.data, out_map.data + out_map.size);
    gst_buffer_unmap(out_buffer, &out_map);
    gst_sample_unref(sample);

    return true;
}

std::string JetsonH265Encoder::getLastError() {
    std::lock_guard<std::mutex> guard(mutex_);
    return last_error_;
}

void JetsonH265Encoder::cleanup() {
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        if (appsrc_) {
            gst_object_unref(appsrc_);
            appsrc_ = nullptr;
        }
        if (appsink_) {
            gst_object_unref(appsink_);
            appsink_ = nullptr;
        }
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
    initialized_ = false;
    frame_counter_ = 0;
}

bool JetsonH265Encoder::createPipeline() {
    std::ostringstream pipeline_desc;
    pipeline_desc << "appsrc name=src is-live=true block=false format=time caps=\"video/x-raw,format=RGB,width="
                  << width_ << ",height=" << height_ << ",framerate=" << fps_ << "/1\" "
                  << "! videoconvert ! video/x-raw,format=NV12 "
                  << "! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 "
                  << "! nvv4l2h265enc preset-level=1 bitrate=" << bitrate_
                  << " control-rate=variable insert-sps-pps=1 iframeinterval=" << group_len_
                  << " idrinterval=" << group_len_
                  << " num-B-frames=0 "
                  << "! video/x-h265,stream-format=byte-stream,alignment=au "
                  << "! h265parse config-interval=1 "
                  << "! appsink name=sink emit-signals=false sync=false max-buffers=5 drop=true";

    GError* error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_desc.str().c_str(), &error);
    if (!pipeline_) {
        last_error_ = error ? error->message : "Failed to construct encoder pipeline";
        if (error) {
            g_error_free(error);
        }
        return false;
    }

    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (!appsrc_ || !appsink_) {
        last_error_ = "Failed to find encoder elements";
        cleanup();
        return false;
    }

    gst_app_sink_set_max_buffers(GST_APP_SINK(appsink_), 5);
    gst_app_sink_set_drop(GST_APP_SINK(appsink_), TRUE);

    return true;
}

void JetsonH265Encoder::ensureGstInitialized() {
    static bool gst_initialized = false;
    static std::mutex init_mutex;
    std::lock_guard<std::mutex> guard(init_mutex);
    if (!gst_initialized) {
        gst_init(nullptr, nullptr);
        gst_initialized = true;
    }
}

} // namespace cr_camera_driver
