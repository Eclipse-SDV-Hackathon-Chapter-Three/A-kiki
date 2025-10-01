#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/Actor.h>
#include <carla/sensor/SensorData.h>
#include <carla/sensor/data/Image.h>
#include <opencv2/opencv.hpp>
#include <zenoh.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstring>
#include <iostream>
#include <vector>

class SharedMemoryManager {
private:
    int shm_fd;
    void* shm_ptr;
    size_t shm_size;
    std::string shm_name;
    
public:
    SharedMemoryManager(const std::string& name, size_t size) : shm_name(name), shm_size(size) {
        // Create shared memory
        shm_fd = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1) {
            throw std::runtime_error("Failed to create shared memory");
        }
        
        // Set size
        if (ftruncate(shm_fd, size) == -1) {
            throw std::runtime_error("Failed to set shared memory size");
        }
        
        // Map to memory
        shm_ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (shm_ptr == MAP_FAILED) {
            throw std::runtime_error("Failed to map shared memory");
        }
    }
    
    ~SharedMemoryManager() {
        if (shm_ptr != MAP_FAILED) {
            munmap(shm_ptr, shm_size);
        }
        if (shm_fd != -1) {
            close(shm_fd);
            shm_unlink(shm_name.c_str());
        }
    }
    
    void* get_ptr() { return shm_ptr; }
    size_t get_size() { return shm_size; }
    
    void write_data(const void* data, size_t size, size_t offset = 0) {
        if (offset + size > shm_size) {
            throw std::runtime_error("Write exceeds shared memory size");
        }
        std::memcpy(static_cast<char*>(shm_ptr) + offset, data, size);
    }
    
    void read_data(void* data, size_t size, size_t offset = 0) {
        if (offset + size > shm_size) {
            throw std::runtime_error("Read exceeds shared memory size");
        }
        std::memcpy(data, static_cast<char*>(shm_ptr) + offset, size);
    }
};

struct CameraFrame {
    uint64_t timestamp;
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    uint32_t frame_id;
    // Image data follows immediately after this struct
};

class HighPerformanceCameraProcessor {
private:
    carla::client::Client client;
    carla::client::World world;
    carla::client::Actor vehicle;
    carla::client::Actor camera;
    
    zenoh::Session session;
    zenoh::Publisher publisher;
    
    SharedMemoryManager* shm_manager;
    std::atomic<bool> running{true};
    std::thread camera_thread;
    std::thread detection_thread;
    
    // Performance monitoring
    std::atomic<uint64_t> frame_count{0};
    std::atomic<uint64_t> detection_count{0};
    std::chrono::steady_clock::time_point start_time;
    
    // Image processing
    cv::Mat current_frame;
    std::vector<cv::Rect> bounding_boxes;
    std::mutex frame_mutex;
    
public:
    HighPerformanceCameraProcessor() 
        : client("localhost", 2000), 
          world(client.GetWorld()),
          session(zenoh::open(zenoh::Config{})),
          publisher(session.declare_publisher("carla/camera/frames")),
          start_time(std::chrono::steady_clock::now()) {
        
        // Initialize shared memory (10MB for image data)
        shm_manager = new SharedMemoryManager("/carla_camera_shm", 10 * 1024 * 1024);
        
        setup_vehicle();
        setup_camera();
    }
    
    ~HighPerformanceCameraProcessor() {
        running = false;
        if (camera_thread.joinable()) camera_thread.join();
        if (detection_thread.joinable()) detection_thread.join();
        delete shm_manager;
    }
    
    void setup_vehicle() {
        try {
            auto vehicle_bp = world.GetBlueprintLibrary().Find("vehicle.tesla.model3");
            auto spawn_points = world.GetMap().GetSpawnPoints();
            if (spawn_points.empty()) {
                throw std::runtime_error("No spawn points available");
            }
            
            vehicle = world.SpawnActor(vehicle_bp, spawn_points[0]);
            std::cout << "✅ Vehicle spawned successfully" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "❌ Error spawning vehicle: " << e.what() << std::endl;
            throw;
        }
    }
    
    void setup_camera() {
        try {
            auto camera_bp = world.GetBlueprintLibrary().Find("sensor.camera.rgb");
            camera_bp.SetAttribute("image_size_x", "1280");
            camera_bp.SetAttribute("image_size_y", "720");
            camera_bp.SetAttribute("fov", "90");
            
            auto camera_transform = carla::Transform(
                carla::Location(1.5f, 0.0f, 1.4f),
                carla::Rotation(0.0f, 0.0f, 0.0f)
            );
            
            camera = world.SpawnActor(camera_bp, camera_transform, vehicle);
            
            // Set up camera callback
            camera.Listen([this](auto data) {
                this->process_camera_data(data);
            });
            
            std::cout << "📷 Camera attached successfully" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "❌ Error setting up camera: " << e.what() << std::endl;
            throw;
        }
    }
    
    void process_camera_data(const carla::sensor::SensorData& data) {
        try {
            auto image_data = carla::sensor::data::Image(data);
            
            // Convert to OpenCV Mat
            cv::Mat image = cv::Mat(image_data.GetHeight(), image_data.GetWidth(), CV_8UC4, 
                                  const_cast<uint8_t*>(image_data.GetData()));
            
            // Convert BGRA to BGR
            cv::Mat bgr_image;
            cv::cvtColor(image, bgr_image, cv::COLOR_BGRA2BGR);
            
            // Store frame for detection thread
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                current_frame = bgr_image.clone();
            }
            
            // Write to shared memory
            write_frame_to_shm(bgr_image, image_data.GetFrame());
            
            frame_count++;
            
        } catch (const std::exception& e) {
            std::cerr << "⚠️ Error processing camera data: " << e.what() << std::endl;
        }
    }
    
    void write_frame_to_shm(const cv::Mat& image, uint32_t frame_id) {
        try {
            CameraFrame header;
            header.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            header.width = image.cols;
            header.height = image.rows;
            header.channels = image.channels();
            header.frame_id = frame_id;
            
            size_t header_size = sizeof(CameraFrame);
            size_t image_size = image.total() * image.elemSize();
            size_t total_size = header_size + image_size;
            
            // Write header
            shm_manager->write_data(&header, header_size, 0);
            
            // Write image data
            shm_manager->write_data(image.data, image_size, header_size);
            
            // Publish frame metadata via Zenoh
            std::string metadata = "{\"frame_id\":" + std::to_string(frame_id) + 
                                 ",\"timestamp\":" + std::to_string(header.timestamp) +
                                 ",\"width\":" + std::to_string(header.width) +
                                 ",\"height\":" + std::to_string(header.height) +
                                 ",\"shm_name\":\"/carla_camera_shm\"}";
            
            publisher.put(metadata);
            
        } catch (const std::exception& e) {
            std::cerr << "⚠️ Error writing to shared memory: " << e.what() << std::endl;
        }
    }
    
    void detection_loop() {
        cv::HOGDescriptor hog;
        hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
        
        while (running) {
            try {
                cv::Mat frame;
                {
                    std::lock_guard<std::mutex> lock(frame_mutex);
                    if (current_frame.empty()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                    frame = current_frame.clone();
                }
                
                // Detect people
                std::vector<cv::Rect> people;
                std::vector<double> weights;
                hog.detectMultiScale(frame, people, weights, 0.0, cv::Size(8, 8), cv::Size(32, 32), 1.05, 2);
                
                // Detect vehicles (simple color-based detection)
                detect_vehicles(frame);
                
                detection_count++;
                
                // Limit detection rate
                std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
                
            } catch (const std::exception& e) {
                std::cerr << "⚠️ Error in detection loop: " << e.what() << std::endl;
            }
        }
    }
    
    void detect_vehicles(const cv::Mat& frame) {
        try {
            // Convert to HSV for better color detection
            cv::Mat hsv;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            
            // Define color ranges for vehicles
            cv::Scalar lower_blue(100, 50, 50);
            cv::Scalar upper_blue(130, 255, 255);
            
            cv::Mat mask;
            cv::inRange(hsv, lower_blue, upper_blue, mask);
            
            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            // Filter and draw bounding boxes
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (area > 1000) { // Minimum area threshold
                    cv::Rect bbox = cv::boundingRect(contour);
                    bounding_boxes.push_back(bbox);
                }
            }
            
        } catch (const std::exception& e) {
            std::cerr << "⚠️ Error detecting vehicles: " << e.what() << std::endl;
        }
    }
    
    void start_processing() {
        std::cout << "🚀 Starting high-performance camera processing..." << std::endl;
        
        // Start detection thread
        detection_thread = std::thread(&HighPerformanceCameraProcessor::detection_loop, this);
        
        // Main loop
        while (running) {
            // Performance monitoring
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
            
            if (elapsed > 0) {
                double fps = static_cast<double>(frame_count) / elapsed;
                double detection_fps = static_cast<double>(detection_count) / elapsed;
                
                std::cout << "\r📊 FPS: " << std::fixed << std::setprecision(1) << fps 
                         << " | Detection FPS: " << detection_fps 
                         << " | Frames: " << frame_count 
                         << " | Detections: " << detection_count << std::flush;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    void stop() {
        running = false;
    }
};

int main() {
    try {
        std::cout << "🎬 High-Performance CARLA Camera Processor" << std::endl;
        std::cout << "Using Zenoh C++ + Shared Memory" << std::endl;
        
        HighPerformanceCameraProcessor processor;
        processor.start_processing();
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Fatal error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

