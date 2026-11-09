#include "waypoints.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;

class Control : public rclcpp::Node
{
public:
  Control() : Node("waypoints")
  {
    set_parameter(rclcpp::Parameter("use_sim_time", true));

    // load waypoints and thresholds
    Waypoint::load(waypoints, position_thr, orientation_thr);

    pose_cmd.header.frame_id = "world";
    pub = create_publisher<PoseStamped>("/bluerov2/cmd_pose", 1);

    static auto timer{create_wall_timer(10ms, [&]()
      {
        if(!buffer.canTransform("world", "bluerov2/base_link", tf2::TimePointZero))
          return;
        const auto transform{buffer.lookupTransform("world", "bluerov2/base_link", tf2::TimePointZero)};
        trackWaypoint(transform.transform);
      })};
  }

private:

  std::vector<Waypoint> waypoints;
  double position_thr, orientation_thr;

  tf2_ros::Buffer buffer{get_clock()};
  tf2_ros::TransformListener tl{buffer};

  PoseStamped pose_cmd;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub;


  void trackWaypoint(const Transform &pose)
  {
    static auto idx{0};
    const auto &cur_wp{waypoints[idx]};

    // TODO update 'idx' to cycle through the waypoints when the current one 'cur_wp' is reached
    // first we need to transform the waypoint to a position and a quaternion

    geometry_msgs::msg::Vector3 cur_wp_position;
    cur_wp_position.x = cur_wp.x;
    cur_wp_position.y = cur_wp.y;
    cur_wp_position.z = cur_wp.z;

    geometry_msgs::msg::Quaternion cur_wp_quaternion;
    cur_wp_quaternion.w = cos((cur_wp.theta)/2.0);
    cur_wp_quaternion.x = 0;
    cur_wp_quaternion.y = 0;
    cur_wp_quaternion.z = sin((cur_wp.theta/2.0));
    /*
    geometry_msgs::msg::Vector3 pos_threshold_p;
    pos_threshold_p.x = cur_wp.x + 1;
    pos_threshold_p.y = cur_wp.y + 1;
    pos_threshold_p.z = cur_wp.z + 1;
    */

    geometry_msgs::msg::Vector3 position_diff;
    position_diff.x = cur_wp.x - pose.translation.x;
    position_diff.y = cur_wp.y - pose.translation.y;
    position_diff.z = cur_wp.z - pose.translation.z;

    geometry_msgs::msg::Quaternion rotation_diff;
    rotation_diff.w = cur_wp_quaternion.w - pose.rotation.w;
    rotation_diff.x = cur_wp_quaternion.x - pose.rotation.x;
    rotation_diff.y = cur_wp_quaternion.y - pose.rotation.y;
    rotation_diff.z = cur_wp_quaternion.z - pose.rotation.z;

    float norm_diff_pos = sqrt((position_diff.x)*(position_diff.x) + (position_diff.y)*(position_diff.y) + (position_diff.z)*(position_diff.z));
    float norm_diff_rot = sqrt((rotation_diff.w)*(rotation_diff.w) + (rotation_diff.x)*(rotation_diff.x) + (rotation_diff.y)*(rotation_diff.y) + (rotation_diff.z)*(rotation_diff.z));

    //for(int i=0; i <= waypoints.size(); i++){}

    if(norm_diff_pos < position_thr){
        if(norm_diff_rot < orientation_thr){
            idx = idx + 1;
            if(idx > (int)waypoints.size()){
                idx = 0;
            }
        }
    }

    // write new waypoint as msg and publish as reference pose
    waypoints[idx].write(pose_cmd);
    pose_cmd.header.stamp = get_clock()->now();
    pub->publish(pose_cmd);
  }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());
  rclcpp::shutdown();
  return 0;
}
