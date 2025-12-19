#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <cstring>

void query_camera_capabilities(int cam_id) {
    std::string device = "/dev/video" + std::to_string(cam_id);
    
    std::cout << "\n=== Camera " << cam_id << " (" << device << ") ===" << std::endl;
    
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        std::cout << "❌ Failed to open " << device << ": " << strerror(errno) << std::endl;
        return;
    }
    
    // 查询设备能力
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        std::cout << "Device: " << cap.card << std::endl;
        std::cout << "Driver: " << cap.driver << std::endl;
        std::cout << "Version: " << ((cap.version >> 16) & 0xFF) << "." 
                  << ((cap.version >> 8) & 0xFF) << "." << (cap.version & 0xFF) << std::endl;
    }
    
    // 查询当前格式
    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) == 0) {
        std::cout << "\n--- Current Format ---" << std::endl;
        std::cout << "Width: " << fmt.fmt.pix.width << std::endl;
        std::cout << "Height: " << fmt.fmt.pix.height << std::endl;
        std::cout << "Pixel Format: " << (char)(fmt.fmt.pix.pixelformat & 0xFF)
                  << (char)((fmt.fmt.pix.pixelformat >> 8) & 0xFF)
                  << (char)((fmt.fmt.pix.pixelformat >> 16) & 0xFF)
                  << (char)((fmt.fmt.pix.pixelformat >> 24) & 0xFF) << std::endl;
        std::cout << "Bytes per line: " << fmt.fmt.pix.bytesperline << std::endl;
        std::cout << "Image size: " << fmt.fmt.pix.sizeimage << std::endl;
    }
    
    // 查询当前帧率
    struct v4l2_streamparm parm = {};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_PARM, &parm) == 0) {
        std::cout << "\n--- Current Frame Rate ---" << std::endl;
        if (parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) {
            auto& tpf = parm.parm.capture.timeperframe;
            if (tpf.denominator > 0) {
                double fps = (double)tpf.denominator / tpf.numerator;
                std::cout << "Frame rate: " << fps << " fps" << std::endl;
                std::cout << "Time per frame: " << tpf.numerator << "/" << tpf.denominator 
                          << " seconds" << std::endl;
            } else {
                std::cout << "Frame rate: Not set or invalid" << std::endl;
            }
        } else {
            std::cout << "Frame rate control: Not supported" << std::endl;
        }
        std::cout << "Capture mode: " << 
            ((parm.parm.capture.capturemode & V4L2_MODE_HIGHQUALITY) ? "High Quality" : "Normal") << std::endl;
    }
    
    // 枚举支持的帧率
    std::cout << "\n--- Supported Frame Rates ---" << std::endl;
    struct v4l2_frmivalenum frmival = {};
    frmival.pixel_format = V4L2_PIX_FMT_UYVY;
    frmival.width = 1920;
    frmival.height = 1536;
    frmival.index = 0;
    
    while (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) == 0) {
        if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            double fps = (double)frmival.discrete.denominator / frmival.discrete.numerator;
            std::cout << "  " << fps << " fps (" 
                      << frmival.discrete.numerator << "/" << frmival.discrete.denominator << ")" << std::endl;
        } else if (frmival.type == V4L2_FRMIVAL_TYPE_STEPWISE) {
            double min_fps = (double)frmival.stepwise.max.denominator / frmival.stepwise.max.numerator;
            double max_fps = (double)frmival.stepwise.min.denominator / frmival.stepwise.min.numerator;
            std::cout << "  Range: " << min_fps << " - " << max_fps << " fps (stepwise)" << std::endl;
        }
        frmival.index++;
    }
    
    // 枚举支持的分辨率
    std::cout << "\n--- Supported Resolutions (UYVY) ---" << std::endl;
    struct v4l2_frmsizeenum frmsize = {};
    frmsize.pixel_format = V4L2_PIX_FMT_UYVY;
    frmsize.index = 0;
    
    while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
        if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
            std::cout << "  " << frmsize.discrete.width << "x" << frmsize.discrete.height << std::endl;
        } else if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
            std::cout << "  Range: " << frmsize.stepwise.min_width << "x" << frmsize.stepwise.min_height
                      << " to " << frmsize.stepwise.max_width << "x" << frmsize.stepwise.max_height
                      << " (step: " << frmsize.stepwise.step_width << "x" << frmsize.stepwise.step_height << ")" << std::endl;
        }
        frmsize.index++;
    }
    
    // 检查控制参数
    std::cout << "\n--- Camera Controls ---" << std::endl;
    
    // 查询帧率控制
    struct v4l2_queryctrl queryctrl = {};
    queryctrl.id = V4L2_CID_EXPOSURE_AUTO;
    if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) == 0) {
        std::cout << "Exposure Auto: " << queryctrl.name << std::endl;
        
        struct v4l2_control ctrl = {};
        ctrl.id = V4L2_CID_EXPOSURE_AUTO;
        if (ioctl(fd, VIDIOC_G_CTRL, &ctrl) == 0) {
            std::cout << "Current value: " << ctrl.value << std::endl;
        }
    }
    
    close(fd);
}

void set_camera_framerate(int cam_id, double target_fps) {
    std::string device = "/dev/video" + std::to_string(cam_id);
    
    std::cout << "\n=== Setting Camera " << cam_id << " to " << target_fps << " fps ===" << std::endl;
    
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        std::cout << "❌ Failed to open " << device << std::endl;
        return;
    }
    
    // 设置帧率
    struct v4l2_streamparm parm = {};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    // 先获取当前参数
    if (ioctl(fd, VIDIOC_G_PARM, &parm) == 0) {
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = (uint32_t)target_fps;
        
        if (ioctl(fd, VIDIOC_S_PARM, &parm) == 0) {
            double actual_fps = (double)parm.parm.capture.timeperframe.denominator / 
                               parm.parm.capture.timeperframe.numerator;
            std::cout << "✅ Frame rate set to: " << actual_fps << " fps" << std::endl;
        } else {
            std::cout << "❌ Failed to set frame rate: " << strerror(errno) << std::endl;
        }
    } else {
        std::cout << "❌ Failed to get stream parameters: " << strerror(errno) << std::endl;
    }
    
    close(fd);
}

int main(int argc, char* argv[]) {
    std::cout << "=== Camera Frame Rate Analysis Tool ===" << std::endl;
    
    if (argc > 1 && std::string(argv[1]) == "set") {
        if (argc < 4) {
            std::cout << "Usage: " << argv[0] << " set <camera_id> <fps>" << std::endl;
            return 1;
        }
        int cam_id = std::stoi(argv[2]);
        double fps = std::stod(argv[3]);
        set_camera_framerate(cam_id, fps);
        return 0;
    }
    
    // 查询所有摄像头
    for (int cam_id = 0; cam_id < 5; cam_id++) {
        query_camera_capabilities(cam_id);
    }
    
    std::cout << "\n=== Usage Examples ===" << std::endl;
    std::cout << "Set camera 0 to 30fps: " << argv[0] << " set 0 30" << std::endl;
    std::cout << "Set all cameras to 30fps:" << std::endl;
    for (int i = 0; i < 5; i++) {
        std::cout << "  " << argv[0] << " set " << i << " 30" << std::endl;
    }
    
    return 0;
}
