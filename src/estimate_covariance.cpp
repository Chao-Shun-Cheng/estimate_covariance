#include <math.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

using namespace std;

tf::StampedTransform rsu2map;
geometry_msgs::Pose ground_truth;
double distance_threshold_;
ofstream logfile;
string logfile_name_;
string save_path_;
/*
This function can transform pose from local frame to map frame.
*/
geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose &in_pose, const tf::StampedTransform &tf_stamp)
{
    tf::Transform transform;
    geometry_msgs::PoseStamped out_pose;
    transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
    transform.setRotation(tf::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
    geometry_msgs::PoseStamped pose_out;
    tf::poseTFToMsg(tf_stamp * transform, out_pose.pose);
    return out_pose.pose;
}

/*
This function can find the transformation matrix from local frame to map frame.
local2global_ will save the matrix including translation and rotation.
*/
bool updateNecessaryTransform(const autoware_msgs::DetectedObjectArray &input, tf::StampedTransform &local2global_)
{
    bool success = true;
    tf::TransformListener tf_listener_;
    try {
        tf_listener_.waitForTransform(input.header.frame_id, "map", ros::Time(0), ros::Duration(1.0));
        tf_listener_.lookupTransform("map", input.header.frame_id, ros::Time(0), local2global_);
    } catch (tf::TransformException ex)  // check TF
    {
        ROS_ERROR("%s", ex.what());
        success = false;
    }
    return success;
}

/*
This function create the log file.
The path will be ~/RSU/lilee_s0/Experiment/2022/
*/
void get_logfilename()
{
    time_t now = time(0);
    tm *ltm = localtime(&now);
    logfile_name_ = (1 + ltm->tm_mon) / 10 ? to_string(1 + ltm->tm_mon) : ("0" + to_string(1 + ltm->tm_mon));      // month
    logfile_name_ += (1 + ltm->tm_mday) / 10 ? to_string(1 + ltm->tm_mday) : ("0" + to_string(1 + ltm->tm_mday));  // day
    logfile_name_ += (1 + ltm->tm_hour) / 10 ? to_string(1 + ltm->tm_hour) : ("0" + to_string(1 + ltm->tm_hour));  // hour
    logfile_name_ += (1 + ltm->tm_min) / 10 ? to_string(1 + ltm->tm_min) : ("0" + to_string(1 + ltm->tm_min));     // minute
    logfile_name_ += ".csv";
    logfile.open(save_path_ + logfile_name_, std::ofstream::out | std::ios_base::out);
    if (!logfile.is_open()) {
        cerr << "failed to open " << logfile_name_ << '\n';
    } else {
        logfile << "time,ndt_x,ndt_y,ndt_yaw,rsu_x,rsu_y,rsu_yaw,dim_x,dim_y,dim_z,label\n";  // format
        logfile.close();
    }
    return;
}

void ndt_callback(const geometry_msgs::PoseStamped &input)
{
    cout << "Receive ground truth pose." << endl;
    ground_truth = input.pose;
    return;
}

void RSU_callback(const autoware_msgs::DetectedObjectArray &input)
{
    cout << "Receive RSU data." << endl;
    bool success = updateNecessaryTransform(input, rsu2map);
    if (!success) {
        ROS_INFO("Could not find coordiante transformation from RSU to map");
        return;
    }
    for (int i = 0; i < input.objects.size(); i++) {
        geometry_msgs::Pose out_pose = getTransformedPose(input.objects[i].pose, rsu2map);
        double distance = pow((ground_truth.position.x - out_pose.position.x), 2) + pow((ground_truth.position.y - out_pose.position.y), 2);
        distance = pow(distance, 0.5);
        if (distance < distance_threshold_) {  // write data into file
            logfile.open(save_path_ + logfile_name_, std::ofstream::out | std::ios_base::out);
            if (!logfile.is_open()) {
                cerr << "failed to open " << logfile_name_ << '\n';
            } else {
                logfile << to_string(input.header.stamp.toSec()) << ",";
                logfile << to_string(ground_truth.position.x) << "," << to_string(ground_truth.position.y) << ","
                        << to_string(tf::getYaw(ground_truth.orientation)) << ",";
                logfile << to_string(out_pose.position.x) << "," << to_string(out_pose.position.y) << ","
                        << to_string(tf::getYaw(out_pose.orientation)) << ",";
                logfile << to_string(input.objects[i].dimensions.x) << "," << to_string(input.objects[i].dimensions.y) << ","
                        << to_string(input.objects[i].dimensions.z) << ",";
                logfile << input.objects[i].label << endl;
                logfile.close();
            }
        }
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "estimate_covariance");
    ros::NodeHandle n;
    ros::NodeHandle private_nh_("~");
    private_nh_.param<double>("distance_threshold", distance_threshold_, 3);
    private_nh_.param<std::string>("save_path", save_path_, "/");
    cout << "Receive param, distance_threshold : " << distance_threshold_ << endl;
    cout << "save path : " << save_path_ << endl;

    ros::Subscriber sub1 = n.subscribe("/ndt_pose", 1, ndt_callback);
    ros::Subscriber sub2 = n.subscribe("RSU_topic_name_", 1, RSU_callback);
    get_logfilename();
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
