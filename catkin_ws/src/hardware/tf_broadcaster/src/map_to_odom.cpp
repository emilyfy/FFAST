#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::PoseWithCovarianceStamped map_base_link;
geometry_msgs::TransformStamped odom_trans;
ros::Time last_map_cb_time;

void mapCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    map_base_link = msg;
    last_map_cb_time = ros::Time::now();
}

void odomCallback(const nav_msgs::Odometry& odom)
{
    if (ros::Time::now() - last_map_cb_time > ros::Duration(1)) return;
    
    odom_trans.transform.translation.x = map_base_link.pose.pose.position.x - odom.pose.pose.position.x;
    odom_trans.transform.translation.y = map_base_link.pose.pose.position.y - odom.pose.pose.position.y;
    odom_trans.transform.translation.z = map_base_link.pose.pose.position.z - odom.pose.pose.position.z;
    
    tf::Quaternion odom_quat, map_quat;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, odom_quat);
    tf::quaternionMsgToTF(map_base_link.pose.pose.orientation, map_quat);
    tf::quaternionTFToMsg(map_quat*odom_quat.inverse(),odom_trans.transform.rotation);

    odom_trans.header.stamp = ros::Time::now();
    static tf::TransformBroadcaster map_odom_br;
    map_odom_br.sendTransform(odom_trans);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "map_to_odom");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
  
    std::string odom_topic;
    if (!pnh.getParam("odom_topic", odom_topic)) {
        odom_topic = "odom";
    }
    
    ros::Subscriber map_sub = nh.subscribe("/scan_matcher/pose",10,mapCallback);
    ros::Subscriber odom_sub = nh.subscribe(odom_topic.c_str(),10,odomCallback);
  
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "odom";
  
    ros::spin();
}

