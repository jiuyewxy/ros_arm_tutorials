#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "advance_demo/action/pickup_place.hpp"

class PickupPlaceClient : public rclcpp::Node
{
public:
  using PickupPlace = advance_demo::action::PickupPlace;
  using GoalHandlePickupPlace = rclcpp_action::ClientGoalHandle<PickupPlace>;

  PickupPlaceClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("PickupPlace_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<PickupPlace>(
      this,
      "pickup_place");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PickupPlaceClient::send_goal, this));
  }
  ~PickupPlaceClient(){}


  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = PickupPlace::Goal();
    goal_msg.target_name = "box";
    goal_msg.target_pose.position.x = 0.35;
    goal_msg.target_pose.position.y = -0.35;
    goal_msg.target_pose.position.z = 0.1;
    goal_msg.target_pose.orientation.w = 1;

    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options = rclcpp_action::Client<PickupPlace>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&PickupPlaceClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&PickupPlaceClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&PickupPlaceClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<PickupPlace>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandlePickupPlace::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandlePickupPlace::SharedPtr,const std::shared_ptr<const PickupPlace::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Task completed %i%%", feedback->percent_complete);
  }

  void result_callback(const GoalHandlePickupPlace::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Pickup and place the object successfully!");
    rclcpp::shutdown();
  }
}; 


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PickupPlaceClient>());
  rclcpp::shutdown();
  return 0;
}