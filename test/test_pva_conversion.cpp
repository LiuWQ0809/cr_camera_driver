#include <iostream>
#include <vector>
#include <chrono>
#include <vpi/VPI.h>
#include <vpi/algo/ConvertImageFormat.h>

void print_backend_support() {
    std::cout << "=== VPI Backend Support Test ===" << std::endl;
    
    // Test backend availability by trying to create streams
    VPIStream test_stream = nullptr;
    
    std::cout << "Available backends:" << std::endl;
    
    // Test CPU
    if (vpiStreamCreate(VPI_BACKEND_CPU, &test_stream) == VPI_SUCCESS) {
        std::cout << "  - CPU ✅" << std::endl;
        vpiStreamDestroy(test_stream);
    } else {
        std::cout << "  - CPU ❌" << std::endl;
    }
    
    // Test CUDA
    if (vpiStreamCreate(VPI_BACKEND_CUDA, &test_stream) == VPI_SUCCESS) {
        std::cout << "  - CUDA ✅" << std::endl;
        vpiStreamDestroy(test_stream);
    } else {
        std::cout << "  - CUDA ❌" << std::endl;
    }
    
    // Test VIC
    if (vpiStreamCreate(VPI_BACKEND_VIC, &test_stream) == VPI_SUCCESS) {
        std::cout << "  - VIC ✅" << std::endl;
        vpiStreamDestroy(test_stream);
    } else {
        std::cout << "  - VIC ❌" << std::endl;
    }
    
    // Test PVA
    if (vpiStreamCreate(VPI_BACKEND_PVA, &test_stream) == VPI_SUCCESS) {
        std::cout << "  - PVA ✅" << std::endl;
        vpiStreamDestroy(test_stream);
    } else {
        std::cout << "  - PVA ❌" << std::endl;
    }
    
    std::cout << std::endl;
}

bool test_pva_uyvy_to_rgba() {
    std::cout << "=== Testing PVA UYVY->RGBA8 Conversion ===" << std::endl;
    
    const int width = 1920;
    const int height = 1080;
    
    VPIStream pva_stream = nullptr;
    VPIImage input_image = nullptr;
    VPIImage output_image = nullptr;
    
    try {
        // Create PVA stream
        VPIStatus status = vpiStreamCreate(VPI_BACKEND_PVA, &pva_stream);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ Failed to create PVA stream: " << vpiStatusGetName(status) << std::endl;
            return false;
        }
        std::cout << "✅ PVA stream created successfully" << std::endl;
        
        // Create input UYVY image
        status = vpiImageCreate(width, height, VPI_IMAGE_FORMAT_UYVY, 0, &input_image);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ Failed to create UYVY input image: " << vpiStatusGetName(status) << std::endl;
            goto cleanup;
        }
        std::cout << "✅ UYVY input image created" << std::endl;
        
        // Create output RGBA8 image
        status = vpiImageCreate(width, height, VPI_IMAGE_FORMAT_RGBA8, 0, &output_image);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ Failed to create RGBA8 output image: " << vpiStatusGetName(status) << std::endl;
            goto cleanup;
        }
        std::cout << "✅ RGBA8 output image created" << std::endl;
        
        // Test UYVY->RGBA8 conversion with PVA
        status = vpiSubmitConvertImageFormat(pva_stream, VPI_BACKEND_PVA, 
                                           input_image, output_image, nullptr);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ PVA UYVY->RGBA8 conversion NOT supported: " << vpiStatusGetName(status) << std::endl;
            goto cleanup;
        }
        
        // Wait for completion
        status = vpiStreamSync(pva_stream);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ PVA stream sync failed: " << vpiStatusGetName(status) << std::endl;
            goto cleanup;
        }
        
        std::cout << "✅ PVA UYVY->RGBA8 conversion SUPPORTED!" << std::endl;
        
    cleanup:
        if (output_image) vpiImageDestroy(output_image);
        if (input_image) vpiImageDestroy(input_image);
        if (pva_stream) vpiStreamDestroy(pva_stream);
        
        return status == VPI_SUCCESS;
        
    } catch (const std::exception& e) {
        std::cout << "❌ Exception: " << e.what() << std::endl;
        return false;
    }
}

bool test_pva_performance() {
    std::cout << "\n=== PVA UYVY->RGBA8 Performance Test ===" << std::endl;
    
    const int width = 1920;
    const int height = 1080;
    const int test_iterations = 100;
    
    VPIStream pva_stream = nullptr;
    VPIImage input_image = nullptr;
    VPIImage output_image = nullptr;
    
    try {
        // Create PVA stream
        VPIStatus status = vpiStreamCreate(VPI_BACKEND_PVA, &pva_stream);
        if (status != VPI_SUCCESS) {
            std::cout << "❌ Failed to create PVA stream" << std::endl;
            return false;
        }
        
        // Create images
        vpiImageCreate(width, height, VPI_IMAGE_FORMAT_UYVY, 0, &input_image);
        vpiImageCreate(width, height, VPI_IMAGE_FORMAT_RGBA8, 0, &output_image);
        
        // Fill input with test data
        VPIImageData input_data;
        vpiImageLockData(input_image, VPI_LOCK_WRITE, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &input_data);
        
        uint8_t* input_ptr = (uint8_t*)input_data.buffer.pitch.planes[0].data;
        int input_pitch = input_data.buffer.pitch.planes[0].pitchBytes;
        
        // Fill with test pattern
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width * 2; x++) {
                input_ptr[y * input_pitch + x] = (x + y) % 256;
            }
        }
        vpiImageUnlock(input_image);
        
        // Warm up
        for (int i = 0; i < 10; i++) {
            vpiSubmitConvertImageFormat(pva_stream, VPI_BACKEND_PVA, input_image, output_image, nullptr);
            vpiStreamSync(pva_stream);
        }
        
        // Performance test
        auto start_time = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < test_iterations; i++) {
            vpiSubmitConvertImageFormat(pva_stream, VPI_BACKEND_PVA, input_image, output_image, nullptr);
            vpiStreamSync(pva_stream);
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        double avg_time_ms = duration.count() / 1000.0 / test_iterations;
        
        std::cout << "PVA UYVY->RGBA8 average time: " << avg_time_ms << " ms" << std::endl;
        std::cout << "PVA performance: " << 1000.0 / avg_time_ms << " FPS" << std::endl;
        
        // Cleanup
        vpiImageDestroy(output_image);
        vpiImageDestroy(input_image);
        vpiStreamDestroy(pva_stream);
        
        return true;
        
    } catch (const std::exception& e) {
        std::cout << "❌ Performance test exception: " << e.what() << std::endl;
        return false;
    }
}

int main() {
    std::cout << "VPI PVA Conversion Test" << std::endl;
    
    // Print backend support
    print_backend_support();
    
    // Test PVA UYVY->RGBA conversion
    bool pva_supported = test_pva_uyvy_to_rgba();
    
    if (pva_supported) {
        test_pva_performance();
    }
    
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "PVA UYVY->RGBA8 support: " << (pva_supported ? "✅ YES" : "❌ NO") << std::endl;
    
    return 0;
}
