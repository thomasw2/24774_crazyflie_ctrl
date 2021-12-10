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
        pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/cf1/pose",1, &VisualizerNode::pose_callback, this);
        ROS_INFO("Subscriber created");

        // setup marker
        marker_pub = nh_.advertise<visualization_msgs::Marker>("/cf1/rviz_marker", 1);
        make_markers();
    };
    void broadcast()
    {
        br.sendTransform(tf::StampedTransform(current_config, ros::Time::now(), "world", "Quad"));
        // ROS_INFO("Pose:%f",current_config.getRotation().getW());
        marker_pub.publish(fbk_marker);
        marker_pub.publish(tar_marker);
        for(int i=0;i<4;i++)
        {
            marker_pub.publish(wall_marker[i]);
        }
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
    visualization_msgs::Marker fbk_marker;
    visualization_msgs::Marker tar_marker;
    visualization_msgs::Marker wall_marker[4];

    void make_markers()
    {
        fbk_marker.header.frame_id = "Quad";
        fbk_marker.ns = "Quad";
        fbk_marker.action = visualization_msgs::Marker::ADD;
        fbk_marker.type = visualization_msgs::Marker::CUBE;
        fbk_marker.pose.position.x = 0;
        fbk_marker.pose.position.y = 0;
        fbk_marker.pose.position.z = 0;
        fbk_marker.scale.x = 0.08;
        fbk_marker.scale.y = 0.08;
        fbk_marker.scale.z = 0.02;
        fbk_marker.pose.orientation.x = 0.0;
        fbk_marker.pose.orientation.y = 0.0;
        fbk_marker.pose.orientation.z = 0.0;
        fbk_marker.pose.orientation.w = 1.0;
        fbk_marker.color.a = 0.3;
        fbk_marker.color.r = 1.0;
        fbk_marker.color.g = 1.0;
        fbk_marker.color.b = 0.0;

        tar_marker.header.frame_id = "world";
        tar_marker.ns = "Target";
        tar_marker.action = visualization_msgs::Marker::ADD;
        tar_marker.type = visualization_msgs::Marker::CUBE;
        tar_marker.pose.position.x = 0;
        tar_marker.pose.position.y = 0;
        tar_marker.pose.position.z = 0.4;
        tar_marker.scale.x = 0.08;
        tar_marker.scale.y = 0.08;
        tar_marker.scale.z = 0.02;
        tar_marker.pose.orientation.x = 0.0;
        tar_marker.pose.orientation.y = 0.0;
        tar_marker.pose.orientation.z = 0.0;
        tar_marker.pose.orientation.w = 1.0;
        tar_marker.color.a = 0.3;
        tar_marker.color.r = 0.0;
        tar_marker.color.g = 0.5;
        tar_marker.color.b = 0.5;

        wall_marker[0].header.frame_id = "world";
        wall_marker[0].ns = "wall1";
        wall_marker[0].action = visualization_msgs::Marker::ADD;
        wall_marker[0].type = visualization_msgs::Marker::CUBE;
        wall_marker[0].pose.position.x = 0.3;
        wall_marker[0].pose.position.y = 0;
        wall_marker[0].pose.position.z = 0.3;
        wall_marker[0].scale.x = 0.03;
        wall_marker[0].scale.y = 0.6;
        wall_marker[0].scale.z = 0.6;
        wall_marker[0].pose.orientation.x = 0.0;
        wall_marker[0].pose.orientation.y = 0.0;
        wall_marker[0].pose.orientation.z = 0.0;
        wall_marker[0].pose.orientation.w = 1.0;
        wall_marker[0].color.a = 0.2;
        wall_marker[0].color.r = 0.5;
        wall_marker[0].color.g = 0.5;
        wall_marker[0].color.b = 0.5;

        wall_marker[1].header.frame_id = "world";
        wall_marker[1].ns = "wall2";
        wall_marker[1].action = visualization_msgs::Marker::ADD;
        wall_marker[1].type = visualization_msgs::Marker::CUBE;
        wall_marker[1].pose.position.x = 0;
        wall_marker[1].pose.position.y = 0.3;
        wall_marker[1].pose.position.z = 0.3;
        wall_marker[1].scale.x = 0.6;
        wall_marker[1].scale.y = 0.03;
        wall_marker[1].scale.z = 0.6;
        wall_marker[1].pose.orientation.x = 0.0;
        wall_marker[1].pose.orientation.y = 0.0;
        wall_marker[1].pose.orientation.z = 0.0;
        wall_marker[1].pose.orientation.w = 1.0;
        wall_marker[1].color.a = 0.2;
        wall_marker[1].color.r = 0.5;
        wall_marker[1].color.g = 0.5;
        wall_marker[1].color.b = 0.5;

        wall_marker[2].header.frame_id = "world";
        wall_marker[2].ns = "wall3";
        wall_marker[2].action = visualization_msgs::Marker::ADD;
        wall_marker[2].type = visualization_msgs::Marker::CUBE;
        wall_marker[2].pose.position.x = -0.3;
        wall_marker[2].pose.position.y = 0;
        wall_marker[2].pose.position.z = 0.3;
        wall_marker[2].scale.x = 0.03;
        wall_marker[2].scale.y = 0.6;
        wall_marker[2].scale.z = 0.6;
        wall_marker[2].pose.orientation.x = 0.0;
        wall_marker[2].pose.orientation.y = 0.0;
        wall_marker[2].pose.orientation.z = 0.0;
        wall_marker[2].pose.orientation.w = 1.0;
        wall_marker[2].color.a = 0.2;
        wall_marker[2].color.r = 0.5;
        wall_marker[2].color.g = 0.5;
        wall_marker[2].color.b = 0.5;

        wall_marker[3].header.frame_id = "world";
        wall_marker[3].ns = "wall4";
        wall_marker[3].action = visualization_msgs::Marker::ADD;
        wall_marker[3].type = visualization_msgs::Marker::CUBE;
        wall_marker[3].pose.position.x = 0;
        wall_marker[3].pose.position.y = -0.3;
        wall_marker[3].pose.position.z = 0.3;
        wall_marker[3].scale.x = 0.6;
        wall_marker[3].scale.y = 0.03;
        wall_marker[3].scale.z = 0.6;
        wall_marker[3].pose.orientation.x = 0.0;
        wall_marker[3].pose.orientation.y = 0.0;
        wall_marker[3].pose.orientation.z = 0.0;
        wall_marker[3].pose.orientation.w = 1.0;
        wall_marker[3].color.a = 0.2;
        wall_marker[3].color.r = 0.5;
        wall_marker[3].color.g = 0.5;
        wall_marker[3].color.b = 0.5;
    }
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