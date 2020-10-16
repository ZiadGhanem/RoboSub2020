#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using std::placeholders::_1;

class MotionPlanner : public rclcpp::Node
{
  public:
    MotionPlanner()
    : Node("motion_planner")
    {
      // Get Frame shape parameter

      this->declare_parameter("frame_shape");
      auto frame_shape = this->get_parameter("frame_shape").as_integer_array();
      frame_width = frame_shape[0];
      frame_height = frame_shape[1];
      // Create publisher
      publisher_ = this->create_publisher<std_msgs::msg::String>("motion_plan", 1);
      // Create subscriber
      subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "gate_location", 1, std::bind(&MotionPlanner::plan_motion, this, _1));
    }

  private:
    void plan_motion(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const
    {
      // Get the center of the gate
      int center_x = msg->data[0];
      int center_y = msg->data[1];
      // Create motion plan message
      auto motion_plan = std_msgs::msg::String();

      if(center_y < frame_height / 3)
          motion_plan.data += "Up     ";
      else if(center_y > 2 * frame_height / 3)
          motion_plan.data += "Down   ";

      if(center_x < frame_width / 3)
          motion_plan.data += "Left   ";
      else if(center_x > 2 * frame_width / 3)
          motion_plan.data += "Right  ";

      if(motion_plan.data.empty())
        motion_plan.data += "Forwards";

      // Publish motion plan
      publisher_->publish(motion_plan);
      RCLCPP_INFO(this->get_logger(), "Published motion plan");
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    int frame_width, frame_height;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionPlanner>());
  rclcpp::shutdown();
  return 0;
}
