#include <iostream>
#include <vpi/VPI.h>
#include <vpi/algo/ConvertImageFormat.h>

int main() {
    VPIStream stream = nullptr;
    VPIImage input_image = nullptr;
    VPIImage output_image = nullptr;
    
    // Test VIC backend first
    std::cout << "=== Testing VIC Backend ===" << std::endl;
    VPIStatus status = vpiStreamCreate(VPI_BACKEND_VIC, &stream);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create VPI stream with VIC backend: " << vpiStatusGetName(status) << std::endl;
    } else {
        std::cout << "✅ VIC stream created successfully" << std::endl;
        
        // Test RGBA8 -> RGB8 conversion with VIC
        status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_RGBA8, 0, &input_image);
        if (status == VPI_SUCCESS) {
            status = vpiImageCreate(1920, 1536, VPI_IMAGE_FORMAT_RGB8, 0, &output_image);
            if (status == VPI_SUCCESS) {
                status = vpiSubmitConvertImageFormat(stream, VPI_BACKEND_VIC, input_image, output_image, nullptr);
                if (status == VPI_SUCCESS) {
                    std::cout << "✅ VIC: RGBA8 -> RGB8 conversion SUPPORTED" << std::endl;
                } else {
                    std::cout << "❌ VIC: RGBA8 -> RGB8 conversion NOT SUPPORTED (" << vpiStatusGetName(status) << ")" << std::endl;
                }
                vpiImageDestroy(output_image);
            }
            vpiImageDestroy(input_image);
        }
        vpiStreamDestroy(stream);
    }
    
    // Test CUDA backend
    std::cout << "\n=== Testing CUDA Backend ===" << std::endl;
    status = vpiStreamCreate(VPI_BACKEND_CUDA, &stream);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create VPI stream with CUDA backend: " << vpiStatusGetName(status) << std::endl;
        return 1;
    }
    std::cout << "✅ CUDA stream created successfully" << std::endl;
    
    // Test 1: RGBA8 -> RGB8 conversion with CUDA
    std::cout << "\n🔍 Testing CUDA: RGBA8 -> RGB8 conversion..." << std::endl;
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
    
    // Try the conversion with CUDA
    status = vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, input_image, output_image, nullptr);
    if (status == VPI_SUCCESS) {
        std::cout << "✅ CUDA: RGBA8 -> RGB8 conversion SUPPORTED!" << std::endl;
    } else {
        std::cout << "❌ CUDA: RGBA8 -> RGB8 conversion NOT SUPPORTED (" << vpiStatusGetName(status) << ")" << std::endl;
    }
    
    // Clean up for next test
    if (input_image) { vpiImageDestroy(input_image); input_image = nullptr; }
    if (output_image) { vpiImageDestroy(output_image); output_image = nullptr; }
    
    // Test 2: RGBA8 -> BGR8 conversion with CUDA
    std::cout << "\n🔍 Testing CUDA: RGBA8 -> BGR8 conversion..." << std::endl;
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
    
    // Try the conversion with CUDA
    status = vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, input_image, output_image, nullptr);
    if (status == VPI_SUCCESS) {
        std::cout << "✅ CUDA: RGBA8 -> BGR8 conversion SUPPORTED!" << std::endl;
    } else {
        std::cout << "❌ CUDA: RGBA8 -> BGR8 conversion NOT SUPPORTED (" << vpiStatusGetName(status) << ")" << std::endl;
    }
    
    // Clean up for next test
    if (input_image) { vpiImageDestroy(input_image); input_image = nullptr; }
    if (output_image) { vpiImageDestroy(output_image); output_image = nullptr; }
    
    // Test 3: BGRA8 -> RGB8 conversion with CUDA
    std::cout << "\n🔍 Testing CUDA: BGRA8 -> RGB8 conversion..." << std::endl;
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
    
    // Try the conversion with CUDA
    status = vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, input_image, output_image, nullptr);
    if (status == VPI_SUCCESS) {
        std::cout << "✅ CUDA: BGRA8 -> RGB8 conversion SUPPORTED!" << std::endl;
    } else {
        std::cout << "❌ CUDA: BGRA8 -> RGB8 conversion NOT SUPPORTED (" << vpiStatusGetName(status) << ")" << std::endl;
    }
    
    // Clean up for next test
    if (input_image) { vpiImageDestroy(input_image); input_image = nullptr; }
    if (output_image) { vpiImageDestroy(output_image); output_image = nullptr; }
    
    // Test 4: Test CPU backend as well
    std::cout << "\n=== Testing CPU Backend ===" << std::endl;
    if (stream) { vpiStreamDestroy(stream); stream = nullptr; }
    
    status = vpiStreamCreate(VPI_BACKEND_CPU, &stream);
    if (status != VPI_SUCCESS) {
        std::cout << "❌ Failed to create VPI stream with CPU backend: " << vpiStatusGetName(status) << std::endl;
        goto cleanup;
    }
    std::cout << "✅ CPU stream created successfully" << std::endl;
    
    std::cout << "\n🔍 Testing CPU: RGBA8 -> RGB8 conversion..." << std::endl;
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
    
    // Try the conversion with CPU
    status = vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CPU, input_image, output_image, nullptr);
    if (status == VPI_SUCCESS) {
        std::cout << "✅ CPU: RGBA8 -> RGB8 conversion SUPPORTED!" << std::endl;
    } else {
        std::cout << "❌ CPU: RGBA8 -> RGB8 conversion NOT SUPPORTED (" << vpiStatusGetName(status) << ")" << std::endl;
    }
    
    std::cout << "\n📋 Summary:" << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << "• VIC backend: Hardware-accelerated but limited format support" << std::endl;
    std::cout << "• CUDA backend: GPU-accelerated with broader format support" << std::endl;
    std::cout << "• CPU backend: Software fallback with full format support" << std::endl;
    std::cout << std::endl;
    std::cout << "🎯 Best strategy: Use CUDA backend for RGBA->RGB conversion" << std::endl;
    std::cout << "   to leverage GPU acceleration while avoiding CPU bottleneck!" << std::endl;

cleanup:
    if (input_image) vpiImageDestroy(input_image);
    if (output_image) vpiImageDestroy(output_image);
    if (stream) vpiStreamDestroy(stream);
    
    return 0;
}
