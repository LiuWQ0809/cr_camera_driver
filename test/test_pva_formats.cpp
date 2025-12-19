#include <iostream>
#include <vector>
#include <vpi/VPI.h>
#include <vpi/algo/ConvertImageFormat.h>

struct FormatTest {
    VPIImageFormat input_format;
    VPIImageFormat output_format;
    const char* name;
};

int main() {
    std::cout << "=== PVA Format Conversion Support Test ===" << std::endl;
    
    VPIStream pva_stream = nullptr;
    VPIStatus status = vpiStreamCreate(VPI_BACKEND_PVA, &pva_stream);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create PVA stream" << std::endl;
        return -1;
    }
    
    const int width = 640;
    const int height = 480;
    
    // Test various format conversions that PVA might support
    std::vector<FormatTest> tests = {
        {VPI_IMAGE_FORMAT_UYVY, VPI_IMAGE_FORMAT_RGBA8, "UYVY → RGBA8"},
        {VPI_IMAGE_FORMAT_UYVY, VPI_IMAGE_FORMAT_RGB8, "UYVY → RGB8"},
        {VPI_IMAGE_FORMAT_UYVY, VPI_IMAGE_FORMAT_BGR8, "UYVY → BGR8"},
        {VPI_IMAGE_FORMAT_UYVY, VPI_IMAGE_FORMAT_NV12, "UYVY → NV12"},
        {VPI_IMAGE_FORMAT_NV12, VPI_IMAGE_FORMAT_RGBA8, "NV12 → RGBA8"},
        {VPI_IMAGE_FORMAT_NV12, VPI_IMAGE_FORMAT_RGB8, "NV12 → RGB8"},
        {VPI_IMAGE_FORMAT_NV12, VPI_IMAGE_FORMAT_BGR8, "NV12 → BGR8"},
        {VPI_IMAGE_FORMAT_RGBA8, VPI_IMAGE_FORMAT_RGB8, "RGBA8 → RGB8"},
        {VPI_IMAGE_FORMAT_RGBA8, VPI_IMAGE_FORMAT_BGR8, "RGBA8 → BGR8"},
        {VPI_IMAGE_FORMAT_RGB8, VPI_IMAGE_FORMAT_BGR8, "RGB8 → BGR8"},
        {VPI_IMAGE_FORMAT_U8, VPI_IMAGE_FORMAT_U16, "U8 → U16"},
        {VPI_IMAGE_FORMAT_U16, VPI_IMAGE_FORMAT_U8, "U16 → U8"},
    };
    
    std::cout << "Testing PVA format conversion support:" << std::endl;
    std::cout << "===========================================" << std::endl;
    
    for (const auto& test : tests) {
        VPIImage input_img = nullptr, output_img = nullptr;
        
        // Create images
        VPIStatus create_status1 = vpiImageCreate(width, height, test.input_format, 0, &input_img);
        VPIStatus create_status2 = vpiImageCreate(width, height, test.output_format, 0, &output_img);
        
        if (create_status1 == VPI_SUCCESS && create_status2 == VPI_SUCCESS) {
            // Test conversion
            VPIStatus conv_status = vpiSubmitConvertImageFormat(pva_stream, VPI_BACKEND_PVA, 
                                                              input_img, output_img, nullptr);
            
            if (conv_status == VPI_SUCCESS) {
                std::cout << "✅ " << test.name << " - SUPPORTED" << std::endl;
            } else {
                std::cout << "❌ " << test.name << " - NOT SUPPORTED (" << vpiStatusGetName(conv_status) << ")" << std::endl;
            }
        } else {
            std::cout << "⚠️  " << test.name << " - Cannot create images" << std::endl;
        }
        
        // Cleanup
        if (input_img) vpiImageDestroy(input_img);
        if (output_img) vpiImageDestroy(output_img);
    }
    
    vpiStreamDestroy(pva_stream);
    
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "PVA主要用于计算密集型算法(如立体匹配、光流等)" << std::endl;
    std::cout << "对于简单格式转换，VIC和CUDA更合适" << std::endl;
    
    return 0;
}
