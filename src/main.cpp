#include "main.hpp"


void pcl2_receive(const sensor_msgs::PointCloud2::ConstPtr& pcl2_msg)
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Offline_Estimator");
	ros::NodeHandle n;
	//ros::Subscriber sub_tf = n.subscribe("/tf", 1000, tranform_receive);
	ros::Subscriber sub_pcl2 = n.subscribe("/head_camera/depth_registered/points", 1000, pcl2_receive);
	while(ros::ok())
	{
		ros::spinOnce();
	}
}