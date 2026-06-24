///opt/ros/humble/include/**
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class Odom2Tf : public rclcpp::Node //表示公有继承
{
public:
    Odom2Tf(const std::string &name) : Node(name)//构造函数,先构造父类
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&Odom2Tf::odom_callback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);//独占智能指针，
    }
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = msg->header;//使用消息的头部信息(时间和header.frame.id)
        transform.child_frame_id = msg->child_frame_id;
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;
        tf_broadcaster_->sendTransform(transform);//数据传进去发送
    };


};

int main(int argc,char **argv){

    rclcpp::init(argc,argv);//参数传进去初始化
    auto node=std::make_shared<Odom2Tf>("odom2tf");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}