#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"

#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>

using LaserScan = sensor_msgs::msg::LaserScan;
using GetDirection = robot_patrol::srv::GetDirection;

class DistanceTest : public rclcpp::Node {

public:
  DistanceTest();
  bool is_done() const;

private:
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
  bool done_ = false;
  bool called_ = false;
  std::string direction_;

  void scan_callback_(LaserScan::SharedPtr msg);
  void response_callback_(rclcpp::Client<GetDirection>::SharedFuture future);
  void send_async_request_(LaserScan::SharedPtr msg);
};

DistanceTest::DistanceTest() : Node("distance_test"), direction_("None") {
  using namespace std::placeholders;
  client_ = this->create_client<GetDirection>("direction_service");
  scan_sub_ = this->create_subscription<LaserScan>(
      "scan", 10, std::bind(&DistanceTest::scan_callback_, this, _1));
}

bool DistanceTest::is_done() const { return done_; }

void DistanceTest::scan_callback_(LaserScan::SharedPtr msg) {
  if (called_) {
    return;
  }
  send_async_request_(msg);
  called_ = true;
}

void DistanceTest::send_async_request_(LaserScan::SharedPtr msg) {
  using namespace std::chrono_literals;
  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Client interrupted while waiting for service. Terminating...");
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Service Unavailable. Waiting for Service...");
  }

  auto request = std::make_shared<GetDirection::Request>();
  request->laser_data = *msg;

  auto result = client_->async_send_request(
      request, std::bind(&DistanceTest::response_callback_, this,
                         std::placeholders::_1));
  called_ = true;

  // Now check for the response after a timeout of 1 second
  auto status = result.wait_for(1s);

  if (status != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
  }
}

void DistanceTest::response_callback_(
    rclcpp::Client<GetDirection>::SharedFuture future) {
  auto response = future.get();
  RCLCPP_INFO(this->get_logger(), "Response: %s", response->direction.c_str());
  done_ = true;
}

//------------------------------------------------------------------
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto distance_test = std::make_shared<DistanceTest>();
  while (!distance_test->is_done()) {
    rclcpp::spin_some(distance_test);
  }

  rclcpp::shutdown();
  return 0;
}