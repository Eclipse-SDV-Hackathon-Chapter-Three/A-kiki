#include <zenoh.h>
#include <zenohc.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/Image.h>
#include <carla/geom/Transform.h>
#include <carla/geom/Location.h>
#include <carla/geom/Rotation.h>
#include <carla/client/Vehicle.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

class CppZenohSHMProcessor {
private:
    z_owned_session_t session;
    z_owned_publisher_t publisher;
    carla::client::World* world;
    carla::client::Actor* vehicle;
    carla::client::Sensor* camera;
    bool running;
    std::thread processing_thread;
    
    // SHM
    int shm_fd;
    void* shm_ptr;
    size_t shm_size;
    std::string shm_name;
    
    // Performance monitoring
    int frame_count;
    std::chrono::high_resolution_clock::time_point start_time;
    
public:
    CppZenohSHMProcessor() : running(false), shm_fd(-1), shm_ptr(nullptr), 
                            shm_size(20 * 1024 * 1024), shm_name("carla_cpp_shm"), 
                            frame_count(0) {
        // Zenoh 설정 (SHM 활성화)
        z_owned_config_t config = zc_config_default();
        zc_config_insert_json5(z_loan(config), "plugins/shm/enabled", "true");
        zc_config_insert_json5(z_loan(config), "plugins/shm/shm_size", "52428800"); // 50MB
        
        // Zenoh 세션 열기
        session = z_open(z_move(config));
        if (!z_check(session)) {
            throw std::runtime_error("Failed to open Zenoh session");
        }
        
        // Publisher 생성
        publisher = z_declare_publisher(z_loan(session), z_keyexpr("carla/cpp/shm/camera"), NULL);
        if (!z_check(publisher)) {
            throw std::runtime_error("Failed to create publisher");
        }
        
        // SHM 설정
        setup_shm();
        
        std::cout << "✅ C++ Zenoh SHM Processor initialized" << std::endl;
    }
    
    ~CppZenohSHMProcessor() {
        stop();
        cleanup_shm();
        z_drop(z_move(publisher));
        z_drop(z_move(session));
    }
    
    void setup_shm() {
        // SHM 생성
        shm_fd = shm_open(shm_name.c_str(), O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1) {
            throw std::runtime_error("Failed to create shared memory");
        }
        
        // SHM 크기 설정
        if (ftruncate(shm_fd, shm_size) == -1) {
            throw std::runtime_error("Failed to set shared memory size");
        }
        
        // SHM 매핑
        shm_ptr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (shm_ptr == MAP_FAILED) {
            throw std::runtime_error("Failed to map shared memory");
        }
        
        std::cout << "✅ SHM created: " << shm_name << " (" << shm_size << " bytes)" << std::endl;
    }
    
    void cleanup_shm() {
        if (shm_ptr != nullptr && shm_ptr != MAP_FAILED) {
            munmap(shm_ptr, shm_size);
        }
        if (shm_fd != -1) {
            close(shm_fd);
            shm_unlink(shm_name.c_str());
        }
    }
    
    bool setup_carla() {
        try {
            // CARLA 클라이언트 연결
            carla::client::Client client("localhost", 2000);
            client.set_timeout(10.0);
            world = &client.get_world();
            
            // 차량 스폰
            auto vehicle_bp = world->get_blueprint_library().find("vehicle.tesla.model3");
            auto spawn_points = world->get_map().get_spawn_points();
            
            for (auto& spawn_point : spawn_points) {
                try {
                    vehicle = world->spawn_actor(vehicle_bp, spawn_point);
                    break;
                } catch (...) {
                    continue;
                }
            }
            
            if (!vehicle) {
                std::cerr << "❌ Failed to spawn vehicle" << std::endl;
                return false;
            }
            
            // 카메라 설정
            auto camera_bp = world->get_blueprint_library().find("sensor.camera.rgb");
            camera_bp->set_attribute("image_size_x", "1280");
            camera_bp->set_attribute("image_size_y", "720");
            camera_bp->set_attribute("fov", "90");
            
            carla::geom::Transform camera_transform(
                carla::geom::Location(1.5, 0.0, 1.4),
                carla::geom::Rotation(0.0, 0.0, 0.0)
            );
            
            camera = world->spawn_actor(camera_bp, camera_transform, *vehicle);
            
            std::cout << "✅ CARLA setup completed" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "❌ CARLA setup failed: " << e.what() << std::endl;
            return false;
        }
    }
    
    void start() {
        if (running) return;
        
        running = true;
        start_time = std::chrono::high_resolution_clock::now();
        processing_thread = std::thread(&CppZenohSHMProcessor::process_loop, this);
        std::cout << "🚀 C++ Zenoh SHM Processor started" << std::endl;
    }
    
    void stop() {
        if (!running) return;
        
        running = false;
        if (processing_thread.joinable()) {
            processing_thread.join();
        }
        
        if (camera) {
            camera->destroy();
        }
        if (vehicle) {
            vehicle->destroy();
        }
        
        std::cout << "🛑 C++ Zenoh SHM Processor stopped" << std::endl;
    }
    
private:
    void process_loop() {
        while (running) {
            try {
                // CARLA에서 이미지 데이터 가져오기 (시뮬레이션)
                // 실제로는 CARLA 콜백에서 받아야 함
                std::vector<uint8_t> image_data(1280 * 720 * 3, 0x42); // 테스트 데이터
                
                // SHM에 이미지 데이터 쓰기
                if (write_to_shm(image_data)) {
                    // Zenoh C++ SHM을 통한 메타데이터 전송
                    z_put_options_t options = z_put_options_default();
                    options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_OCTET_STREAM, NULL);
                    
                    // 메타데이터 생성
                    std::string metadata = "{\"frame_id\":" + std::to_string(frame_count) + 
                                         ",\"timestamp\":" + std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(
                                             std::chrono::high_resolution_clock::now().time_since_epoch()).count()) +
                                         ",\"width\":1280,\"height\":720,\"shm_name\":\"" + shm_name + "\"}";
                    
                    // Zenoh C++ SHM을 통한 Zero-copy 전송
                    z_put(z_loan(session), z_keyexpr("carla/cpp/shm/camera"), 
                           metadata.c_str(), metadata.length(), &options);
                }
                
                frame_count++;
                
                // 성능 모니터링
                if (frame_count % 100 == 0) {
                    auto current_time = std::chrono::high_resolution_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
                    double fps = frame_count * 1000.0 / elapsed.count();
                    std::cout << "📊 C++ FPS: " << fps << std::endl;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
                
            } catch (const std::exception& e) {
                std::cerr << "⚠️ Error in processing loop: " << e.what() << std::endl;
            }
        }
    }
    
    bool write_to_shm(const std::vector<uint8_t>& data) {
        if (shm_ptr == nullptr || shm_ptr == MAP_FAILED) {
            return false;
        }
        
        // 헤더 정보 (timestamp, width, height, channels, frame_id)
        uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        uint32_t width = 1280;
        uint32_t height = 720;
        uint32_t channels = 3;
        uint32_t frame_id = frame_count;
        
        // 헤더 쓰기
        memcpy(shm_ptr, &timestamp, sizeof(timestamp));
        memcpy((char*)shm_ptr + sizeof(timestamp), &width, sizeof(width));
        memcpy((char*)shm_ptr + sizeof(timestamp) + sizeof(width), &height, sizeof(height));
        memcpy((char*)shm_ptr + sizeof(timestamp) + sizeof(width) + sizeof(height), &channels, sizeof(channels));
        memcpy((char*)shm_ptr + sizeof(timestamp) + sizeof(width) + sizeof(height) + sizeof(channels), &frame_id, sizeof(frame_id));
        
        // 이미지 데이터 쓰기
        size_t header_size = sizeof(timestamp) + sizeof(width) + sizeof(height) + sizeof(channels) + sizeof(frame_id);
        if (header_size + data.size() <= shm_size) {
            memcpy((char*)shm_ptr + header_size, data.data(), data.size());
            return true;
        }
        
        return false;
    }
};

int main() {
    try {
        std::cout << "🎬 C++ Zenoh SHM Camera System" << std::endl;
        std::cout << "=================================" << std::endl;
        
        CppZenohSHMProcessor processor;
        
        if (!processor.setup_carla()) {
            return 1;
        }
        
        processor.start();
        
        // 10초 실행
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        processor.stop();
        
        std::cout << "✅ C++ Zenoh SHM system test completed" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Fatal error: " << e.what() << std::endl;
        return 1;
    }
}





