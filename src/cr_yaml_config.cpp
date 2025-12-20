#include "cr_camera_driver/cr_yaml_config.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace cr_camera_driver {

YamlConfigManager::YamlConfigManager() : config_loaded_(false) {
    setDefaultGlobalSettings();
}

bool YamlConfigManager::loadConfig(const std::string& config_path) {
    std::ifstream file(config_path);
    if (!file.is_open()) {
        last_error_ = "Failed to open config file: " + config_path;
        return false;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();
    file.close();
    
    if (!parseYamlFile(content)) {
        return false;
    }
    
    // 应用全局设置到所有相机
    for (auto& camera : camera_configs_) {
        applyGlobalSettingsToCamera(camera);
    }
    
    if (!validateConfig()) {
        return false;
    }
    
    config_loaded_ = true;
    return true;
}

std::vector<YamlCameraConfig> YamlConfigManager::getCameraConfigs() const {
    return camera_configs_;
}

YamlCameraConfig YamlConfigManager::getCameraConfig(int cam_id) const {
    for (const auto& config : camera_configs_) {
        if (config.id == cam_id) {
            return config;
        }
    }
    
    // 返回默认配置
    YamlCameraConfig default_config;
    default_config.id = cam_id;
    default_config.device = "/dev/video" + std::to_string(cam_id);
    default_config.width = 1920;
    default_config.height = 1536;
    default_config.enabled = false;
    default_config.description = "Default camera " + std::to_string(cam_id);
    
    // 添加默认的resize topic
    ResizeTopicConfig default_topic;
    default_topic.topic = "/cr/camera/rgb/camera_" + std::to_string(cam_id);
    default_topic.width = 1920;
    default_topic.height = 1536;
    default_topic.format = "RGB";
    default_topic.enabled = true;
    default_config.resize_topics.push_back(default_topic);
    
    applyGlobalSettingsToCamera(default_config);
    
    return default_config;
}

GlobalSettings YamlConfigManager::getGlobalSettings() const {
    return global_settings_;
}

std::vector<int> YamlConfigManager::getEnabledCameraIds() const {
    std::vector<int> enabled_ids;
    for (const auto& config : camera_configs_) {
        if (config.enabled) {
            enabled_ids.push_back(config.id);
        }
    }
    return enabled_ids;
}

bool YamlConfigManager::validateConfig() {
    if (camera_configs_.empty()) {
        last_error_ = "No cameras configured";
        return false;
    }
    
    // 检查相机ID重复
    std::vector<int> ids;
    for (const auto& config : camera_configs_) {
        if (std::find(ids.begin(), ids.end(), config.id) != ids.end()) {
            last_error_ = "Duplicate camera ID: " + std::to_string(config.id);
            return false;
        }
        ids.push_back(config.id);
        
        // 检查基本参数
        if (config.width <= 0 || config.height <= 0) {
            last_error_ = "Invalid resolution for camera " + std::to_string(config.id);
            return false;
        }
        
        if (config.device.empty() || config.resize_topics.empty()) {
            last_error_ = "Empty device or no resize topics for camera " + std::to_string(config.id);
            return false;
        }
        
        // 检查resize topics
        for (const auto& topic : config.resize_topics) {
            if (topic.enabled && (topic.topic.empty() || topic.width <= 0 || topic.height <= 0)) {
                last_error_ = "Invalid resize topic configuration for camera " + std::to_string(config.id);
                return false;
            }
        }
    }
    
    return true;
}

std::string YamlConfigManager::getLastError() const {
    return last_error_;
}

bool YamlConfigManager::parseYamlFile(const std::string& content) {
    std::istringstream stream(content);
    std::string line;
    
    bool in_cameras_section = false;
    bool in_global_section = false;
    bool in_resize_topics = false;  // 标记是否在resize_topics段落中
    bool in_h265_section = false;   // 标记是否在h265_output段落中
    YamlCameraConfig current_camera;
    bool has_current_camera = false;
    ResizeTopicConfig current_resize_topic;  // 当前解析的resize topic
    YamlCameraConfig::H265StreamConfig current_h265_config;
    
    while (std::getline(stream, line)) {
        line = trim(line);
        
        // 跳过注释和空行
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        // 检查主要段落
        if (line == "cameras:") {
            in_cameras_section = true;
            in_global_section = false;
            continue;
        } else if (line == "global_settings:") {
            // 保存当前相机配置
            if (has_current_camera) {
                // 确保保存最后一个resize topic（如果存在）
                if (!current_resize_topic.topic.empty()) {
                    current_camera.resize_topics.push_back(current_resize_topic);
                    current_resize_topic = ResizeTopicConfig(); // 重置
                }
                
                // Debug output for current camera configuration
                RCLCPP_INFO(rclcpp::get_logger("cr_yaml_config"), 
                    "Camera %d loaded with %zu resize topics:", 
                    current_camera.id, current_camera.resize_topics.size());
                for (size_t i = 0; i < current_camera.resize_topics.size(); ++i) {
                    const auto& topic = current_camera.resize_topics[i];
                    RCLCPP_INFO(rclcpp::get_logger("cr_yaml_config"), 
                        "  Topic %zu: %s (%dx%d, %s)", 
                        i, topic.topic.c_str(), topic.width, topic.height, topic.format.c_str());
                }
                
                current_camera.h265_stream = current_h265_config;
                camera_configs_.push_back(current_camera);
                has_current_camera = false;
            }
            in_global_section = true;
            in_cameras_section = false;
            continue;
        }
        
        // 处理cameras段落
        if (in_cameras_section) {
            if (line.find("- id:") == 0) {
                // 保存前一个相机配置
                if (has_current_camera) {
                    if (!current_resize_topic.topic.empty()) {
                        current_camera.resize_topics.push_back(current_resize_topic);
                        current_resize_topic = ResizeTopicConfig();
                    }
                    
                    current_camera.h265_stream = current_h265_config;
                    
                    RCLCPP_INFO(rclcpp::get_logger("cr_yaml_config"), 
                        "Camera %d loaded with %zu resize topics:", 
                        current_camera.id, current_camera.resize_topics.size());
                    for (size_t i = 0; i < current_camera.resize_topics.size(); ++i) {
                        const auto& topic = current_camera.resize_topics[i];
                        RCLCPP_INFO(rclcpp::get_logger("cr_yaml_config"), 
                            "  Topic %zu: %s (%dx%d, %s)", 
                            i, topic.topic.c_str(), topic.width, topic.height, topic.format.c_str());
                    }
                    
                    camera_configs_.push_back(current_camera);
                }
                
                // 开始新的相机配置
                current_camera = YamlCameraConfig();
                current_camera.id = extractIntValue(line, "id");
                current_camera.width = 1920;  // 默认值
                current_camera.height = 1536; // 默认值
                current_camera.enabled = false; // 默认值
                std::string default_h265_topic = "/cr/camera/h265/camera_" + std::to_string(current_camera.id);
                current_h265_config = YamlCameraConfig::H265StreamConfig();
                current_h265_config.topic = default_h265_topic;
                in_resize_topics = false;
                in_h265_section = false;
                has_current_camera = true;
                
            } else if (has_current_camera && line.find("device:") != std::string::npos) {
                current_camera.device = extractValue(line, "device");
            } else if (has_current_camera && line.find("width:") != std::string::npos && !in_resize_topics) {
                current_camera.width = extractIntValue(line, "width");
            } else if (has_current_camera && line.find("height:") != std::string::npos && !in_resize_topics) {
                current_camera.height = extractIntValue(line, "height");
            } else if (has_current_camera && line.find("enabled:") != std::string::npos && !in_resize_topics) {
                current_camera.enabled = extractBoolValue(line, "enabled");
            } else if (has_current_camera && line.find("resize_topics:") != std::string::npos) {
                in_resize_topics = true;
                current_camera.resize_topics.clear();  // 清空现有的resize topics
            } else if (has_current_camera && line.find("h265_output:") != std::string::npos) {
                in_h265_section = true;
            } else if (has_current_camera && in_h265_section) {
                bool handled = false;
                if (line.find("enabled:") != std::string::npos) {
                    current_h265_config.enabled = extractBoolValue(line, "enabled");
                    handled = true;
                } else if (line.find("topic:") != std::string::npos) {
                    current_h265_config.topic = extractValue(line, "topic");
                    handled = true;
                } else if (line.find("bitrate:") != std::string::npos) {
                    current_h265_config.bitrate = extractIntValue(line, "bitrate");
                    handled = true;
                } else if (line.find("fps:") != std::string::npos) {
                    current_h265_config.fps = extractIntValue(line, "fps");
                    handled = true;
                } else if (line.find("group_len:") != std::string::npos) {
                    current_h265_config.group_len = extractIntValue(line, "group_len");
                    handled = true;
                } else if (line.find("description:") != std::string::npos || line.find("- topic:") != std::string::npos) {
                    in_h265_section = false;
                }
                if (handled) {
                    continue;
                }
            } else if (has_current_camera && line.find("description:") != std::string::npos) {
                current_camera.description = extractValue(line, "description");
                in_resize_topics = false;  // 结束resize_topics段落
                in_h265_section = false;
            } else if (has_current_camera && in_resize_topics) {
                // 处理resize_topics中的内容
                if (line.find("- topic:") != std::string::npos) {
                    // 如果有之前的resize topic，先保存它
                    if (!current_resize_topic.topic.empty()) {
                        current_camera.resize_topics.push_back(current_resize_topic);
                    }
                    // 开始新的resize topic
                    current_resize_topic = ResizeTopicConfig();
                    current_resize_topic.topic = extractValue(line, "topic");
                } else if (line.find("width:") != std::string::npos) {
                    current_resize_topic.width = extractIntValue(line, "width");
                } else if (line.find("height:") != std::string::npos) {
                    current_resize_topic.height = extractIntValue(line, "height");
                } else if (line.find("format:") != std::string::npos) {
                    current_resize_topic.format = extractValue(line, "format");
                } else if (line.find("enabled:") != std::string::npos) {
                    current_resize_topic.enabled = extractBoolValue(line, "enabled");
                }
            }
        }
        
        // 处理global_settings段落
        if (in_global_section) {
            if (line.find("pixel_format:") != std::string::npos) {
                global_settings_.pixel_format_str = extractValue(line, "pixel_format");
                global_settings_.pixel_format = convertPixelFormat(global_settings_.pixel_format_str);
            } else if (line.find("field_type:") != std::string::npos) {
                global_settings_.field_type_str = extractValue(line, "field_type");
                global_settings_.field_type = convertFieldType(global_settings_.field_type_str);
            } else if (line.find("colorspace:") != std::string::npos) {
                global_settings_.colorspace_str = extractValue(line, "colorspace");
                global_settings_.colorspace = convertColorspace(global_settings_.colorspace_str);
            } else if (line.find("num_buffers:") != std::string::npos) {
                global_settings_.num_buffers = extractIntValue(line, "num_buffers");
            } else if (line.find("frame_skip_ratio:") != std::string::npos) {
                global_settings_.frame_skip_ratio = extractIntValue(line, "frame_skip_ratio");
            } else if (line.find("use_vic_converter:") != std::string::npos) {
                global_settings_.use_vic_converter = extractBoolValue(line, "use_vic_converter");
            } else if (line.find("stats_interval:") != std::string::npos) {
                global_settings_.stats_interval = extractIntValue(line, "stats_interval");
            }
        }
    }
    
    // 保存最后一个相机配置
    if (has_current_camera) {
        // 确保保存最后一个resize topic（如果存在）
        if (!current_resize_topic.topic.empty()) {
            current_camera.resize_topics.push_back(current_resize_topic);
        }
        
        current_camera.h265_stream = current_h265_config;
        
        // Debug output for current camera configuration
        RCLCPP_INFO(rclcpp::get_logger("cr_yaml_config"), 
            "Camera %d loaded with %zu resize topics:", 
            current_camera.id, current_camera.resize_topics.size());
        for (size_t i = 0; i < current_camera.resize_topics.size(); ++i) {
            const auto& topic = current_camera.resize_topics[i];
            RCLCPP_INFO(rclcpp::get_logger("cr_yaml_config"), 
                "  Topic %zu: %s (%dx%d, %s)", 
                i, topic.topic.c_str(), topic.width, topic.height, topic.format.c_str());
        }
        
        camera_configs_.push_back(current_camera);
    }
    
    return true;
}

std::string YamlConfigManager::trim(const std::string& str) const {
    size_t start = str.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) return "";
    size_t end = str.find_last_not_of(" \t\n\r");
    return str.substr(start, end - start + 1);
}

std::string YamlConfigManager::extractValue(const std::string& line, const std::string& key) const {
    size_t pos = line.find(key + ":");
    if (pos == std::string::npos) return "";
    
    std::string value = line.substr(pos + key.length() + 1);
    value = trim(value);
    
    // 处理YAML注释 - 在#处截断
    size_t comment_pos = value.find('#');
    if (comment_pos != std::string::npos) {
        value = value.substr(0, comment_pos);
        value = trim(value);  // 重新修剪去除尾部空格
    }
    
    // 移除引号
    if (value.length() >= 2 && value[0] == '"' && value.back() == '"') {
        value = value.substr(1, value.length() - 2);
    }
    
    return value;
}

int YamlConfigManager::extractIntValue(const std::string& line, const std::string& key) const {
    std::string value = extractValue(line, key);
    try {
        return std::stoi(value);
    } catch (...) {
        return 0;
    }
}

bool YamlConfigManager::extractBoolValue(const std::string& line, const std::string& key) const {
    std::string value = extractValue(line, key);
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    return value == "true" || value == "yes" || value == "1";
}

double YamlConfigManager::extractDoubleValue(const std::string& line, const std::string& key) const {
    std::string value = extractValue(line, key);
    try {
        return std::stod(value);
    } catch (...) {
        return 0.0;
    }
}

int YamlConfigManager::convertPixelFormat(const std::string& format_str) const {
    if (format_str == "UYVY") return V4L2_PIX_FMT_UYVY;
    if (format_str == "YUYV") return V4L2_PIX_FMT_YUYV;
    if (format_str == "RGB24") return V4L2_PIX_FMT_RGB24;
    if (format_str == "BGR24") return V4L2_PIX_FMT_BGR24;
    if (format_str == "MJPEG") return V4L2_PIX_FMT_MJPEG;
    return V4L2_PIX_FMT_UYVY; // 默认值
}

int YamlConfigManager::convertFieldType(const std::string& field_str) const {
    if (field_str == "NONE") return V4L2_FIELD_NONE;
    if (field_str == "INTERLACED") return V4L2_FIELD_INTERLACED;
    if (field_str == "TOP") return V4L2_FIELD_TOP;
    if (field_str == "BOTTOM") return V4L2_FIELD_BOTTOM;
    return V4L2_FIELD_NONE; // 默认值
}

int YamlConfigManager::convertColorspace(const std::string& colorspace_str) const {
    if (colorspace_str == "SRGB") return V4L2_COLORSPACE_SRGB;
    if (colorspace_str == "REC709") return V4L2_COLORSPACE_REC709;
    if (colorspace_str == "JPEG") return V4L2_COLORSPACE_JPEG;
    return V4L2_COLORSPACE_SRGB; // 默认值
}

void YamlConfigManager::setDefaultGlobalSettings() {
    global_settings_.pixel_format_str = "UYVY";
    global_settings_.field_type_str = "NONE";
    global_settings_.colorspace_str = "SRGB";
    global_settings_.num_buffers = 2;
    global_settings_.frame_skip_ratio = 2;
    global_settings_.use_vic_converter = false;
    global_settings_.stats_interval = 5;
    
    global_settings_.pixel_format = V4L2_PIX_FMT_UYVY;
    global_settings_.field_type = V4L2_FIELD_NONE;
    global_settings_.colorspace = V4L2_COLORSPACE_SRGB;
}

void YamlConfigManager::applyGlobalSettingsToCamera(YamlCameraConfig& camera) const {
    camera.pixel_format = global_settings_.pixel_format;
    camera.field_type = global_settings_.field_type;
    camera.colorspace = global_settings_.colorspace;
    camera.num_buffers = global_settings_.num_buffers;
}

} // namespace cr_camera_driver
