#include <math.h>
#include <memory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
#include <nav2_map_server/map_io.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/lifecycle_utils.hpp>
#include <nav2_util/node_utils.hpp>

using namespace std::chrono_literals;
namespace fs = boost::filesystem;
using nav2_util::declare_parameter_if_not_declared;

namespace deadlock_test {
class Test : public nav2_util::LifecycleNode {
 public:
  Test(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~Test() {}

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
  void run_test();
  void run_check();

 private:
  bool alive_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  nav_msgs::msg::OccupancyGrid map_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  std::unique_ptr<std::thread> thread_;
  std::unique_ptr<std::thread> thread2_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
};
}  // namespace deadlock_test

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<deadlock_test::Test>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}

namespace deadlock_test {

Test::Test(const rclcpp::NodeOptions &options) : nav2_util::LifecycleNode("deadlock_test", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  alive_ = true;

  // Setup the global costmap
  costmap_ros_ =
      std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap", std::string{get_namespace()}, "global_costmap", false);
  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
}

nav2_util::CallbackReturn Test::on_configure(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "on_configure");
  map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

  costmap_ros_->on_configure(state);


  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Test::on_activate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "on_activate");
  costmap_ros_->on_activate(state);
  map_publisher_->on_activate();

  thread_ = std::make_unique<std::thread>([&]() {
    rclcpp::Rate r(1);
    r.sleep();
    RCLCPP_INFO(get_logger(), "run_test");
    run_test();
  });

  thread2_ = std::make_unique<std::thread>([&]() {
    rclcpp::Rate r(1);
    r.sleep();
    RCLCPP_INFO(get_logger(), "run_check");
    run_check();
  });
  
  return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn Test::on_deactivate(const rclcpp_lifecycle::State &/*state*/) {
  RCLCPP_INFO(get_logger(), "on_deactivate");

  return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn Test::on_cleanup(const rclcpp_lifecycle::State &/*state*/) {
  RCLCPP_INFO(get_logger(), "on_cleanup");

  return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn Test::on_shutdown(const rclcpp_lifecycle::State &/*state*/) {
  RCLCPP_INFO(get_logger(), "on_shutdown");

  return nav2_util::CallbackReturn::SUCCESS;
}

void Test::run_test() {
  fs::path base_path = ament_index_cpp::get_package_share_directory("deadlock_test");
  base_path /= "test";
  fs::path map_path = base_path / "TenByTen.yaml";
  nav2_map_server::LoadParameters yaml;
  if (boost::filesystem::exists(map_path)) {
    yaml = nav2_map_server::loadMapYaml(map_path.string());
    nav2_map_server::loadMapFromFile(yaml, map_);
  } else {
    RCLCPP_INFO(get_logger(), "map file not found\n");
  }

  rclcpp::Rate r(1000);
  int x = 0;
  int y = 0;
  while(alive_) {
    x = (x+3)%19;
    y = (y+5)%19;
    map_.info.origin.position.x = x/10.0 - 5.0;
    map_.info.origin.position.y = y/10.0 - 5.0;
    map_publisher_->publish(map_);
    r.sleep();
  }
}

void Test::run_check() {
  rclcpp::Rate r(10);
  int count = 0;
  while(alive_) {
    RCLCPP_ERROR(get_logger(), "check deadlock");
    std::unique_lock<std::recursive_mutex> lock(*(costmap_ros_->getCostmap()->getMutex()), std::defer_lock);
    if (lock.try_lock()) {
      lock.unlock();
      count = 0;
    } else {
      count++;
    }

    if (count > 10) {
      RCLCPP_ERROR(get_logger(), "costmap is deadlocked");
      alive_ = false;
    }
    r.sleep();
  }
}
}  // namespace deadlock_test
