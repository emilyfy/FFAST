#include <ros/ros.h>
#include <optic_flow/OpticFlowMsg.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#define PX_SCALE -3.0


class OpticFlowToTwist
{
    public:
        OpticFlowToTwist(ros::NodeHandle nh, ros::NodeHandle pnh);
    
    private:
        geometry_msgs::TwistWithCovarianceStamped twist_msg_;
        ros::Subscriber optic_flow_sub_;
        ros::Publisher twist_pub_;

        void OpticFlowCallback(const optic_flow::OpticFlowMsg::ConstPtr& optic_flow_msg);
};


OpticFlowToTwist::OpticFlowToTwist(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    twist_pub_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("optic_flow_twist", 10);
    optic_flow_sub_ = nh.subscribe("optic_flow", 10, &OpticFlowToTwist::OpticFlowCallback, this);

    twist_msg_.header.frame_id = "base_link";
    twist_msg_.twist.twist.linear.y = 0.0;
    twist_msg_.twist.twist.linear.z = 0.0;
    twist_msg_.twist.twist.angular.x = 0.0;
    twist_msg_.twist.twist.angular.y = 0.0;
    twist_msg_.twist.twist.angular.z = 0.0;
}

void OpticFlowToTwist::OpticFlowCallback(const optic_flow::OpticFlowMsg::ConstPtr& optic_flow_msg)
{
    twist_msg_.header.stamp = optic_flow_msg->header.stamp;

    unsigned int n = optic_flow_msg->x.size();
    
    double sum = 0.0;
    for (unsigned int i = 0; i < n; i++)
        sum += optic_flow_msg->vx[i]/PX_SCALE;
    double ave = sum/n;

    double sum_diff_sq = 0.0;
    for (unsigned int i = 0; i < n; i++)
        sum_diff_sq += pow(optic_flow_msg->vx[i]/PX_SCALE-ave,2.0);
    double var = sum_diff_sq/n;

    twist_msg_.twist.twist.linear.x = ave;
    twist_msg_.twist.covariance[0] = var;

    twist_pub_.publish(twist_msg_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "optic_flow_to_twist");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    OpticFlowToTwist optic_flow_to_twist(nh, pnh);

    ros::spin();

    return 0;
}
