#include <map>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

class GazeboMocapNode {

    public:
        GazeboMocapNode(ros::NodeHandle, ros::NodeHandle);
        void run();

    private:
        ros::NodeHandle nh, pnh;
        tf2_ros::TransformBroadcaster broadcaster;
        void model_states_callback(const gazebo_msgs::ModelStates::ConstPtr&);
        std::map<std::string, ros::Publisher> publishers;
        ros::Subscriber model_states_sub;

};

GazeboMocapNode::GazeboMocapNode(ros::NodeHandle nh, ros::NodeHandle pnh) {
    this->nh = nh;
    this->pnh = pnh;
    this->model_states_sub = this->nh.subscribe("/gazebo/model_states", 1, &GazeboMocapNode::model_states_callback, this);

}

void GazeboMocapNode::model_states_callback(const gazebo_msgs::ModelStates::ConstPtr& model_states_msg) {

    for (std::size_t i = 0; i < model_states_msg->name.size(); i++) {
        std::string name = model_states_msg->name[i];
        // Check if "drone" is contained in the model state name
        if (name.find("drone") != std::string::npos) {
            // If there is no publisher yet, create it
            if (publishers.find(name.c_str()) == publishers.end()) {
                std::stringstream ss;
                ss << "/" << name << "/mavros/vision_pose/pose";
                std::string topic = ss.str();
                publishers[name] = nh.advertise<geometry_msgs::PoseStamped>(topic, 1);
            } else {

                // Get Gazebo model pose
                geometry_msgs::Pose model_pose = model_states_msg->pose[i];

                // Publish pose
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header.frame_id = "world";
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.pose = model_pose;

                publishers[name].publish(pose_msg);

                std::stringstream ss;
                ss << name << "/base_link";

                // Broadcast transform
                geometry_msgs::TransformStamped transform_msg;
                transform_msg.header.frame_id = "world";
                transform_msg.header.stamp = ros::Time::now();
                transform_msg.child_frame_id = ss.str();

                transform_msg.transform.translation.x = model_pose.position.x;
                transform_msg.transform.translation.y = model_pose.position.y;
                transform_msg.transform.translation.z = model_pose.position.z;

                transform_msg.transform.rotation = model_pose.orientation;

                broadcaster.sendTransform(transform_msg);

            }

        }
        // ROS_INFO_STREAM("Name: " << i);
    }

}

void GazeboMocapNode::run() {

    ros::Rate rate(100.0);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_mocap_node");
    ros::NodeHandle nh(""), pnh("~");
    GazeboMocapNode gazebo_mocap_node(nh, pnh);
    gazebo_mocap_node.run();
    return 0;
}