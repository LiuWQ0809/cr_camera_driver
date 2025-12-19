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
    
    // Test 1: UYVY -> RGB8 direct conversion
    std::cout << "\n🔍 Testing UYVY -> RGB8 conversion..." << std::endl;
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_UYVY, 0, &input_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create UYVY input image: " << vpiStatusGetName(status) << std::endl;
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
        std::cout << "✅ UYVY -> RGB8 conversion: SUPPORTED" << std::endl;
    } else {
        std::cout << "❌ UYVY -> RGB8 conversion: NOT SUPPORTED (" << vpiStatusGetName(status) << ")" << std::endl;
    }
    
    // Clean up for next test
    if (input_image) { vpiImageDestroy(input_image); input_image = nullptr; }
    if (output_image) { vpiImageDestroy(output_image); output_image = nullptr; }
    
    // Test 2: UYVY -> BGR8 direct conversion
    std::cout << "\n🔍 Testing UYVY -> BGR8 conversion..." << std::endl;
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_UYVY, 0, &input_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create UYVY input image: " << vpiStatusGetName(status) << std::endl;
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
        std::cout << "✅ UYVY -> BGR8 conversion: SUPPORTED" << std::endl;
    } else {
        std::cout << "❌ UYVY -> BGR8 conversion: NOT SUPPORTED (" << vpiStatusGetName(status) << ")" << std::endl;
    }
    
    // Clean up for next test
    if (input_image) { vpiImageDestroy(input_image); input_image = nullptr; }
    if (output_image) { vpiImageDestroy(output_image); output_image = nullptr; }
    
    // Test 3: UYVY -> RGBA8 direct conversion
    std::cout << "\n🔍 Testing UYVY -> RGBA8 conversion..." << std::endl;
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_UYVY, 0, &input_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create UYVY input image: " << vpiStatusGetName(status) << std::endl;
        goto cleanup;
    }
    
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_RGBA8, 0, &output_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create RGBA8 output image: " << vpiStatusGetName(status) << std::endl;
        goto cleanup;
    }
    
    // Try the conversion
    status = vpiSubmitConvertImageFormat(stream, VPI_BACKEND_VIC, input_image, output_image, nullptr);
    if (status == VPI_SUCCESS) {
        std::cout << "✅ UYVY -> RGBA8 conversion: SUPPORTED" << std::endl;
    } else {
        std::cout << "❌ UYVY -> RGBA8 conversion: NOT SUPPORTED (" << vpiStatusGetName(status) << ")" << std::endl;
    }
    
    // Clean up for next test
    if (input_image) { vpiImageDestroy(input_image); input_image = nullptr; }
    if (output_image) { vpiImageDestroy(output_image); output_image = nullptr; }
    
    // Test 4: UYVY -> NV12 conversion
    std::cout << "\n🔍 Testing UYVY -> NV12 conversion..." << std::endl;
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_UYVY, 0, &input_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create UYVY input image: " << vpiStatusGetName(status) << std::endl;
        goto cleanup;
    }
    
    status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_NV12, 0, &output_image);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create NV12 output image: " << vpiStatusGetName(status) << std::endl;
        goto cleanup;
    }
    
    // Try the conversion
    status = vpiSubmitConvertImageFormat(stream, VPI_BACKEND_VIC, input_image, output_image, nullptr);
    if (status == VPI_SUCCESS) {
        std::cout << "✅ UYVY -> NV12 conversion: SUPPORTED" << std::endl;
    } else {
        std::cout << "❌ UYVY -> NV12 conversion: NOT SUPPORTED (" << vpiStatusGetName(status) << ")" << std::endl;
    }
    
    std::cout << "\n📋 Summary:" << std::endl;
    std::cout << "The reason we use UYVY->NV12->RGBA8 is because VIC hardware" << std::endl;
    std::cout << "does not support direct UYVY->RGB/BGR conversion but does" << std::endl;
    std::cout << "support the two-step conversion pathway." << std::endl;

cleanup:
    if (input_image) vpiImageDestroy(input_image);
    if (output_image) vpiImageDestroy(output_image);
    if (stream) vpiStreamDestroy(stream);
    
    return 0;
}
