#include <ros/ros.h>
#include <std_msgs/Float64.h>
// #include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
# define M_PI           3.14159265358979323846

class VisualizerNode
{
public:
    VisualizerNode()
    {
        initial_config.setIdentity();
        current_config.setIdentity();
        // setup pose sub
        pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/cf1/pose",10, &VisualizerNode::pose_callback, this);
        ROS_INFO("Subscriber created");

        // setup marker
        marker_pub = nh_.advertise<visualization_msgs::Marker>("/cf1/rviz_marker", 1);
        marker.header.frame_id = "Quad";
        marker.ns = "Quad";
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.03;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.a = 0.3;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    };
    void broadcast()
    {
        br.sendTransform(tf::StampedTransform(current_config, ros::Time::now(), "world", "Quad"));
        // ROS_INFO("Pose:%f",current_config.getRotation().getW());
        marker_pub.publish(marker);
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_config.setOrigin(tf::Vector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z));
        tf::Quaternion tempq;
        tempq = tf::Quaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
        current_config.setRotation(tempq);
    }

private:
    ros::NodeHandle nh_;
    // ros::Subscriber imu_sub;
    ros::Subscriber pose_sub;
    tf::Transform initial_config;
    tf::Transform current_config;
    tf::TransformBroadcaster br;
    ros::Publisher marker_pub;
    visualization_msgs::Marker marker;
};

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "VisualizerNode");
    //Create an object of class Estimator that will take care of everything
    VisualizerNode VisualizerNode;
    // EstObject.initialize_config();
    ROS_INFO("Visualizer Node ready");
    while(ros::ok())
    {
        VisualizerNode.broadcast();
        ros::spinOnce();
    }
    

    return 0;
}