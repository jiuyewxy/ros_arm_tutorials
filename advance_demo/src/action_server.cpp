#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "advance_demo/action/pickup_place.hpp"

class PickupPlaceServer:public rclcpp::Node{
public:
  using PickupPlace = advance_demo::action::PickupPlace;
  using GoalHandlePickupPlace = rclcpp_action::ServerGoalHandle<PickupPlace>;

  PickupPlaceServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("PickupPlace_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<PickupPlace>(
      this,
      "pickup_place",
      std::bind(&PickupPlaceServer::handle_goal, this, _1, _2),
      std::bind(&PickupPlaceServer::handle_cancel, this, _1),
      std::bind(&PickupPlaceServer::handle_accepted, this, _1));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"PickupPlace_action_server is Ready!");

  }
  ~PickupPlaceServer(){}


private:
  rclcpp_action::Server<PickupPlace>::SharedPtr action_server_;
  // 服务端接收到action的goal时的回调执行函数
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickupPlace::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request. The target object is %s.", goal->target_name.c_str());
    //RCLCPP_INFO_STREAM(this->get_logger(), goal->target_pose);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePickupPlace> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(const std::shared_ptr<GoalHandlePickupPlace> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&PickupPlaceServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePickupPlace> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Start to pickup and place...");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PickupPlace::Feedback>();
    auto result = std::make_shared<PickupPlace::Result>();

    // 通过循环,模拟任务完成的进度百分比
    bool success = true;
    for(int i = 0; (i<10)&& rclcpp::ok(); i++){
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        success = false;
        break;
      }
      feedback->percent_complete += 10;
      // 发布目标的执行过程反馈feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback:%d",feedback->percent_complete);
      loop_rate.sleep();
    }

    // 若目标任务执行成功,发送目标执行结果
    if (success){
      RCLCPP_INFO(this->get_logger(),"/pickup_place: Succeeded");
      result->success = true;
      goal_handle->succeed(result);
    }
  }

};

//RCLCPP_COMPONENTS_REGISTER_NODE(PickupPlaceServer)
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PickupPlaceServer>());
  rclcpp::shutdown();
  return 0;
}
