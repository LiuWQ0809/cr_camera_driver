#ifndef CR_CAMERA_DRIVER_CR_YAML_CONFIG_HPP
#define CR_CAMERA_DRIVER_CR_YAML_CONFIG_HPP

#include <string>
#include <vector>
#include <map>
#include <linux/videodev2.h>

namespace cr_camera_driver {

// Resize topic配置结构体
struct ResizeTopicConfig {
    std::string topic;
    int width;
    int height;
    std::string format;  // "BGR", "RGB", "GRAY"
    bool enabled;
    
    ResizeTopicConfig() : width(0), height(0), enabled(false) {}
};

struct YamlCameraConfig {
    int id;
    std::string device;
    int width;
    int height;
    bool enabled;
    std::vector<ResizeTopicConfig> resize_topics;  // 统一的topic配置
    std::string description;
    
    // V4L2配置（从全局设置继承）
    int pixel_format;
    int field_type;
    int colorspace;
    int num_buffers;
};

struct GlobalSettings {
    std::string pixel_format_str;
    std::string field_type_str; 
    std::string colorspace_str;
    int num_buffers;
    int frame_skip_ratio;
    bool use_vic_converter;
    int stats_interval;
    
    // 转换后的V4L2常量
    int pixel_format;
    int field_type;
    int colorspace;
};

class YamlConfigManager {
public:
    YamlConfigManager();
    ~YamlConfigManager() = default;
    
    // 加载配置文件
    bool loadConfig(const std::string& config_path);
    
    // 获取相机配置
    std::vector<YamlCameraConfig> getCameraConfigs() const;
    YamlCameraConfig getCameraConfig(int cam_id) const;
    
    // 获取全局设置
    GlobalSettings getGlobalSettings() const;
    
    // 获取启用的相机ID列表
    std::vector<int> getEnabledCameraIds() const;
    
    // 验证配置文件
    bool validateConfig();
    
    // 获取错误信息
    std::string getLastError() const;

private:
    std::vector<YamlCameraConfig> camera_configs_;
    GlobalSettings global_settings_;
    std::string last_error_;
    bool config_loaded_;
    
    // 解析辅助函数
    bool parseYamlFile(const std::string& content);
    
    // 字符串解析辅助函数
    std::string trim(const std::string& str) const;
    std::string extractValue(const std::string& line, const std::string& key) const;
    int extractIntValue(const std::string& line, const std::string& key) const;
    double extractDoubleValue(const std::string& line, const std::string& key) const;
    bool extractBoolValue(const std::string& line, const std::string& key) const;
    
    // V4L2常量转换
    int convertPixelFormat(const std::string& format_str) const;
    int convertFieldType(const std::string& field_str) const;
    int convertColorspace(const std::string& colorspace_str) const;
    
    // 设置默认值
    void setDefaultGlobalSettings();
    void applyGlobalSettingsToCamera(YamlCameraConfig& camera) const;
};

} // namespace cr_camera_driver

#endif // CR_CAMERA_DRIVER_CR_YAML_CONFIG_HPP
