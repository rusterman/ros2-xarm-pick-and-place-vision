#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

// One cheese currently riding the belt. id/name are fixed at spawn time.
// x/y are the scripted belt-tracked position, advanced every tick. z/yaw
// are only the *initial* spawn pose — once a live GetEntityState reply has
// come back for this entity, CheeseSpawner::setPose uses Gazebo's own
// physics-computed z/orientation instead, so gravity/tipping/contact are
// still observable rather than being silently overwritten every tick.
struct Cheese {
    int id;
    std::string name;
    double x, y, z, yaw;
};

// Owns nothing but the Foxglove debug publisher. Kept separate from
// CheeseSpawner's spawn/motion bookkeeping because it only ever reads the
// active list — it has no mutable state to share with the rest of the node.
class CheeseMarkerPublisher {
public:
    explicit CheeseMarkerPublisher(rclcpp::Node *node)
        : pub_(node->create_publisher<visualization_msgs::msg::MarkerArray>("/cheese/markers", 10)) {}

    void publish(
        const rclcpp::Time &stamp, const std::vector<Cheese> &active,
        double length, double width, double height,
        double r, double g, double b, double a) {
        visualization_msgs::msg::MarkerArray arr;

        visualization_msgs::msg::Marker clear;
        clear.header.stamp = stamp;
        clear.header.frame_id = "world";
        clear.ns = "cheese";
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(clear);

        for (const auto &c : active) {
            visualization_msgs::msg::Marker m;
            m.header.stamp = stamp;
            m.header.frame_id = "world";
            m.ns = "cheese";
            m.id = c.id;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = c.x;
            m.pose.position.y = c.y;
            m.pose.position.z = c.z;
            m.pose.orientation.z = std::sin(c.yaw / 2.0);
            m.pose.orientation.w = std::cos(c.yaw / 2.0);
            m.scale.x = length;
            m.scale.y = width;
            m.scale.z = height;
            m.color.r = static_cast<float>(r);
            m.color.g = static_cast<float>(g);
            m.color.b = static_cast<float>(b);
            m.color.a = static_cast<float>(a);
            arr.markers.push_back(m);
        }

        pub_->publish(arr);
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};

class CheeseSpawner : public rclcpp::Node {
public:
    CheeseSpawner() : Node("cheese_spawner"), gen_(std::random_device{}()) {
        // ---- parameters (overridable from the launch file) ----
        cheese_urdf_ = declare_parameter<std::string>("cheese_urdf", "");
        belt_speed_ = declare_parameter<double>("belt_speed", 0.10); // m/s, +Y
        spawn_min_ = declare_parameter<double>("spawn_interval_min", 4.0); // s
        spawn_max_ = declare_parameter<double>("spawn_interval_max", 6.0); // s
        x_center_ = declare_parameter<double>("spawn_x_center", 0.40); // belt centre
        x_jitter_ = declare_parameter<double>("spawn_x_jitter", 0.10); // +/- across belt
        spawn_y_ = declare_parameter<double>("spawn_y", -1.5); // upstream of camera
        spawn_z_ = declare_parameter<double>("spawn_z", 0.02); // rest height on belt
        despawn_y_ = declare_parameter<double>("despawn_y", 2.0); // past the workspace
        cheese_length_ = declare_parameter<double>("cheese_length", 0.100);
        cheese_width_ = declare_parameter<double>("cheese_width", 0.050);
        cheese_height_ = declare_parameter<double>("cheese_height", 0.020);
        cheese_color_r_ = declare_parameter<double>("cheese_color_r", 0.95);
        cheese_color_g_ = declare_parameter<double>("cheese_color_g", 0.80);
        cheese_color_b_ = declare_parameter<double>("cheese_color_b", 0.20);
        cheese_color_a_ = declare_parameter<double>("cheese_color_a", 1.0);

        if (cheese_urdf_.empty())
            RCLCPP_WARN(get_logger(), "cheese_urdf param is empty — nothing will spawn.");

        markers_ = std::make_unique<CheeseMarkerPublisher>(this);

        spawn_cli_ = create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        delete_cli_ = create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
        get_cli_ = create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
        state_cli_ = create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");

        next_spawn_in_ = randRange(spawn_min_, spawn_max_);
        timer_ = create_wall_timer(20ms, [this] { tick(); }); // 50 Hz
        RCLCPP_INFO(get_logger(), "cheese_spawner running (belt speed %.2f m/s)", belt_speed_);
    }

private:
    static constexpr double kDt = 0.02; // matches the 20 ms timer

    double randRange(double lo, double hi) {
        return std::uniform_real_distribution<double>(lo, hi)(gen_);
    }

    // The single 50 Hz heartbeat: schedule spawns, advance every cheese, cull.
    void tick() {
        next_spawn_in_ -= kDt;
        if (next_spawn_in_ <= 0.0) {
            // Only consume this spawn slot once it's actually been acted on.
            // If /spawn_entity isn't ready yet (e.g. still starting up),
            // next_spawn_in_ stays <= 0 so this retries every tick instead
            // of silently losing the spawn until the next random interval.
            if (trySpawnCheese()) {
                next_spawn_in_ = randRange(spawn_min_, spawn_max_);
            }
        }

        std::vector<Cheese> survivors;
        for (auto &c: active_) {
            if (c.y <= despawn_y_) {
                c.y += belt_speed_ * kDt;
            }
            if (c.y > despawn_y_) {
                // Keep retrying delete every tick until it's actually been
                // submitted — otherwise a not-yet-ready /delete_entity
                // service permanently orphans this entity in Gazebo (it
                // drops out of active_ here but is never actually removed
                // from the world).
                if (!tryDeleteCheese(c.name)) {
                    survivors.push_back(c);
                }
            } else {
                setPose(c);
                survivors.push_back(c);
            }
        }
        active_ = std::move(survivors);

        markers_->publish(
            now(), active_, cheese_length_, cheese_width_, cheese_height_,
            cheese_color_r_, cheese_color_g_, cheese_color_b_, cheese_color_a_);
    }

    geometry_msgs::msg::Pose poseFrom(double x, double y, double z, double yaw) {
        geometry_msgs::msg::Pose p;
        p.position.x = x;
        p.position.y = y;
        p.position.z = z;
        p.orientation.z = std::sin(yaw / 2.0); // yaw-only quaternion
        p.orientation.w = std::cos(yaw / 2.0);
        return p;
    }

    // Returns true once a spawn has actually been attempted (submitted to
    // Gazebo, or permanently skipped because cheese_urdf_ is unset — that's
    // a config error already warned about at startup, not worth retrying
    // 50 times a second). Returns false only for the transient case — the
    // service isn't ready yet — so the caller knows to retry next tick
    // instead of burning this spawn slot.
    bool trySpawnCheese() {
        if (cheese_urdf_.empty()) return true;
        if (!spawn_cli_->service_is_ready()) return false;

        Cheese c;
        c.id = static_cast<int>(spawn_count_++);
        c.name = "cheese_" + std::to_string(c.id);
        c.x = x_center_ + randRange(-x_jitter_, x_jitter_);
        c.y = spawn_y_;
        c.z = spawn_z_;
        c.yaw = randRange(-M_PI_2, M_PI_2);

        auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        req->name = c.name;
        req->xml = cheese_urdf_;
        req->initial_pose = poseFrom(c.x, c.y, c.z, c.yaw);
        req->reference_frame = "world";

        spawn_cli_->async_send_request(
            req, [this, c](rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture f) {
                if (f.get()->success) {
                    active_.push_back(c);
                    return;
                }

                RCLCPP_WARN(
                    get_logger(), "spawn %s failed: %s",
                    c.name.c_str(), f.get()->status_message.c_str());
            });
        return true;
    }

    // Drives x/y along the belt every tick. z and orientation are read back
    // from Gazebo via GetEntityState first and passed straight through to
    // SetEntityState unchanged — i.e. wherever physics actually settled the
    // cheese (gravity, tipping, contact with the belt) — instead of being
    // forced from the cached spawn-time z/yaw on every single tick, which
    // would silently fight the physics engine 50 times a second. Falls back
    // to the spawn pose only for the few ticks before the first reply for a
    // newly spawned entity arrives.
    void setPose(const Cheese &c) {
        if (!get_cli_->service_is_ready() || !state_cli_->service_is_ready()) return;

        auto get_req = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        get_req->name = c.name;
        get_req->reference_frame = "world";

        get_cli_->async_send_request(
            get_req, [this, c](rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture f) {
                auto resp = f.get();

                // This is a two-hop async round trip: by the time this reply
                // arrives, a later tick may already have pushed this same
                // cheese past despawn_y_ and had it deleted from Gazebo.
                // Firing SetEntityState for a name Gazebo no longer has can
                // crash gzserver outright (gazebo_ros_state dereferences the
                // looked-up entity without a null check), so bail out if
                // this cheese isn't tracked as active anymore instead of
                // blindly sending the follow-up request.
                if (!isActive(c.name)) return;

                geometry_msgs::msg::Pose pose = poseFrom(c.x, c.y, c.z, c.yaw);
                if (resp->success) {
                    pose.position.z = resp->state.pose.position.z;
                    pose.orientation = resp->state.pose.orientation;
                }

                auto set_req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
                set_req->state.name = c.name;
                set_req->state.pose = pose;
                set_req->state.reference_frame = "world";
                state_cli_->async_send_request(
                    set_req, [](rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedFuture) {
                    });
            });
    }

    bool isActive(const std::string &name) const {
        for (const auto &c : active_) {
            if (c.name == name) return true;
        }
        return false;
    }

    // Returns true once the delete has actually been submitted, so the
    // caller knows whether it's safe to stop tracking this entity or
    // whether it needs to be retried next tick (see tick()).
    bool tryDeleteCheese(const std::string &name) {
        if (!delete_cli_->service_is_ready()) return false;
        auto req = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        req->name = name;
        delete_cli_->async_send_request(
            req, [](rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedFuture) {
            });
        return true;
    }

    // params
    std::string cheese_urdf_;
    double belt_speed_, spawn_min_, spawn_max_, x_center_, x_jitter_, spawn_y_, spawn_z_, despawn_y_;
    double cheese_length_, cheese_width_, cheese_height_;
    double cheese_color_r_, cheese_color_g_, cheese_color_b_, cheese_color_a_;
    // state
    std::mt19937 gen_;
    double next_spawn_in_{0.0};
    size_t spawn_count_{0};
    std::vector<Cheese> active_;
    // ros
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_cli_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_cli_;
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr get_cli_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr state_cli_;
    std::unique_ptr<CheeseMarkerPublisher> markers_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CheeseSpawner>());
    rclcpp::shutdown();
    return 0;
}
