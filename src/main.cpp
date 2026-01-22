#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <mutex>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

// window
#define WIN_W 1280
#define WIN_H 720
#define WIN_TITLE "P&C Sim Visualizer"

// render speed
#define FPS = 30.0

using geometry_msgs::msg::Point;
using nav_msgs::msg::Odometry;

// Message types
struct EntityState {
    float x, y, z;
    float gamma; // rotation about +z
};

struct LaneBoundaries {
    std::vector<Point> left;
    std::vector<Point> right;
};

// Visualizer state
struct VisState {
    Odometry ego;
    std::vector<EntityState> entities;
    LaneBoundaries lanes;
};

VisState g_state;
std::mutex g_mutex;

// OpenGL
// just using immediate mode since I'm not a masochist
void drawRedX(float x, float y, float z, float s = 0.5f) {
    glColor3f(1.f, 0.f, 0.f); // full red
    glBegin(GL_LINES);
    glVertex3f(x - s, y, z);
    glVertex3f(x + s, y, z);
    glVertex3f(x, y - s, z);
    glVertex3f(x, y + s, z);
    glEnd();
}

// chatted this icl i don't know the math
void setPerspective(float fov_deg, float aspect, float near, float far) {
    float fov_rad = fov_deg * M_PI / 180.0f;
    float f = 1.0f / std::tan(fov_rad / 2.0f);

    float m[16] = {
        f / aspect, 0, 0, 0,
        0, f, 0, 0,
        0, 0, (far + near) / (near - far), -1,
        0, 0, (2 * far * near) / (near - far), 0
    };

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(m);
}

// ROS Node
class VisualizerNode : public rclcpp::Node {
public:
    VisualizerNode() : Node("visualizer") {
        ego_sub_ = create_subscription<Odometry>(
            "/sim/ego_state", 10,
            [](const Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(g_mutex);
                g_state.ego = *msg;
            }
        );
    }
private:
    rclcpp::Subscription<Odometry>::SharedPtr ego_sub_;
};

int main(int argc, char** argv) {
    // start ros node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualizerNode>();

    // spin ROS in another thread
    std::thread ros_thread([&]() { // with ref to everything
        rclcpp::spin(node);
    });

    // init glfw
    if (!glfwInit()) {
        throw std::runtime_error("GLFW init failed");
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    GLFWwindow* window = glfwCreateWindow(WIN_W, WIN_H, WIN_TITLE, nullptr, nullptr);
    if (!window) {
        throw std::runtime_error("Window creation failed");
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        throw std::runtime_error("GLAD init failed");
    }
    
    glEnable(GL_DEPTH_TEST);

    constexpr double RENDER_DT = 1.0 / 30.0;

    while (rclcpp::ok() && !glfwWindowShouldClose(window)) {
        auto start = std::chrono::steady_clock::now();

        glfwPollEvents();

        VisState snapshot;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            snapshot = g_state;
        }

        // get actual window size and mac foolish business
        int w, h;
        glfwGetFramebufferSize(window, &w, &h);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 3D view (left)
        glViewport(0, 0, w / 2, h);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        setPerspective(60.0, (double) (w/2) / h, 0.1, 1000.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0, -5, -2); // car camera

        for (auto& e : snapshot.entities) {
            drawRedX(e.x, e.y, e.z);
        }

        // 2D bird's eye view (right)
        glViewport(w / 2, 0, w / 2, h);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-50, 50, -50, 50, -1, 1);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        for (auto& e : snapshot.entities) {
            drawRedX(e.x, e.y, 0.f);
        }

        glfwSwapBuffers(window);

        std::this_thread::sleep_until(start + std::chrono::duration<double>(RENDER_DT));
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    rclcpp::shutdown();
    ros_thread.join();
    return 0;
}