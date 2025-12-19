#include <iostream>
#include <vpi/VPI.h>
#include <vpi/algo/ConvertImageFormat.h>

int main() {
    VPIStream stream = nullptr;
    VPIImage input_image = nullptr;
    VPIImage output_image = nullptr;
    
    // Create VPI stream with VIC backend
    VPIStatus status = vpiStreamCreate(VPI_BACKEND_VIC, &stream);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create VPI stream with VIC backend: " << vpiStatusGetName(status) << std::endl;
        return 1;
    }
    std::cout << "✅ VPI stream created successfully" << std::endl;
    
    // Test 1: RGBA8 -> RGB8 conversion
    std::cout << "\n🔍 Testing RGBA8 -> RGB8 conversion..." << std::endl;
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_RGBA8, 0, &input_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create RGBA8 input image: " << vpiStatusGetName(status) << std::endl;
        goto cleanup;
    }
    
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_RGB8, 0, &output_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create RGB8 output image: " << vpiStatusGetName(status) << std::endl;
        goto cleanup;
    }
    
    // Try the conversion
    status = vpiSubmitConvertImageFormat(stream, VPI_BACKEND_VIC, input_image, output_image, nullptr);
    if (status == VPI_SUCCESS) {
        std::cout << "✅ RGBA8 -> RGB8 conversion: SUPPORTED by VIC!" << std::endl;
    } else {
        std::cout << "❌ RGBA8 -> RGB8 conversion: NOT SUPPORTED by VIC (" << vpiStatusGetName(status) << ")" << std::endl;
    }
    
    // Clean up for next test
    if (input_image) { vpiImageDestroy(input_image); input_image = nullptr; }
    if (output_image) { vpiImageDestroy(output_image); output_image = nullptr; }
    
    // Test 2: RGBA8 -> BGR8 conversion
    std::cout << "\n🔍 Testing RGBA8 -> BGR8 conversion..." << std::endl;
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_RGBA8, 0, &input_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create RGBA8 input image: " << vpiStatusGetName(status) << std::endl;
        goto cleanup;
    }
    
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_BGR8, 0, &output_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create BGR8 output image: " << vpiStatusGetName(status) << std::endl;
        goto cleanup;
    }
    
    // Try the conversion
    status = vpiSubmitConvertImageFormat(stream, VPI_BACKEND_VIC, input_image, output_image, nullptr);
    if (status == VPI_SUCCESS) {
        std::cout << "✅ RGBA8 -> BGR8 conversion: SUPPORTED by VIC!" << std::endl;
    } else {
        std::cout << "❌ RGBA8 -> BGR8 conversion: NOT SUPPORTED by VIC (" << vpiStatusGetName(status) << ")" << std::endl;
    }
    
    // Clean up for next test
    if (input_image) { vpiImageDestroy(input_image); input_image = nullptr; }
    if (output_image) { vpiImageDestroy(output_image); output_image = nullptr; }
    
    // Test 3: BGRA8 -> RGB8 conversion
    std::cout << "\n🔍 Testing BGRA8 -> RGB8 conversion..." << std::endl;
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_BGRA8, 0, &input_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create BGRA8 input image: " << vpiStatusGetName(status) << std::endl;
        goto cleanup;
    }
    
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_RGB8, 0, &output_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create RGB8 output image: " << vpiStatusGetName(status) << std::endl;
        goto cleanup;
    }
    
    // Try the conversion
    status = vpiSubmitConvertImageFormat(stream, VPI_BACKEND_VIC, input_image, output_image, nullptr);
    if (status == VPI_SUCCESS) {
        std::cout << "✅ BGRA8 -> RGB8 conversion: SUPPORTED by VIC!" << std::endl;
    } else {
        std::cout << "❌ BGRA8 -> RGB8 conversion: NOT SUPPORTED by VIC (" << vpiStatusGetName(status) << ")" << std::endl;
    }
    
    // Clean up for next test
    if (input_image) { vpiImageDestroy(input_image); input_image = nullptr; }
    if (output_image) { vpiImageDestroy(output_image); output_image = nullptr; }
    
    // Test 4: BGRA8 -> BGR8 conversion
    std::cout << "\n🔍 Testing BGRA8 -> BGR8 conversion..." << std::endl;
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_BGRA8, 0, &input_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create BGRA8 input image: " << vpiStatusGetName(status) << std::endl;
        goto cleanup;
    }
    
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_BGR8, 0, &output_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create BGR8 output image: " << vpiStatusGetName(status) << std::endl;
        goto cleanup;
    }
    
    // Try the conversion
    status = vpiSubmitConvertImageFormat(stream, VPI_BACKEND_VIC, input_image, output_image, nullptr);
    if (status == VPI_SUCCESS) {
        std::cout << "✅ BGRA8 -> BGR8 conversion: SUPPORTED by VIC!" << std::endl;
    } else {
        std::cout << "❌ BGRA8 -> BGR8 conversion: NOT SUPPORTED by VIC (" << vpiStatusGetName(status) << ")" << std::endl;
    }
    
    std::cout << "\n📋 Summary:" << std::endl;
    std::cout << "If VIC supports RGBA->RGB conversion, we can eliminate the CPU" << std::endl;
    std::cout << "bottleneck and achieve much better performance!" << std::endl;

cleanup:
    if (input_image) vpiImageDestroy(input_image);
    if (output_image) vpiImageDestroy(output_image);
    if (stream) vpiStreamDestroy(stream);
    
    return 0;
}
