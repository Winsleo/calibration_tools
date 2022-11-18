#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/eigen.h>
#include <cmath>
#include <string>
#include <Eigen/Eigen>
#include "tools_colored_print.hpp"
//dynamic reconfigure depends
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <calibration_publisher/CalibConfig.h>
template<typename T>
T rad2deg(T radians)
{
  return radians * 180.0 / M_PI;
}

std::string target_node_name("/calibration_publisher");//动态参数服务端节点名
double x_init,y_init,z_init,roll_init,pitch_init,yaw_init;
ros::ServiceClient set_service;
void config_callback(const calibration_publisher::CalibConfig& config){
	if(config.reset){
		calibration_publisher::CalibConfig calib_config;
		dynamic_reconfigure::Reconfigure srv;
		calib_config.reset = false;
		calib_config.x = x_init;
		calib_config.y = y_init;
		calib_config.z = z_init;
		calib_config.roll = roll_init;
		calib_config.pitch = pitch_init;
		calib_config.yaw = yaw_init;
		calib_config.__toMessage__(srv.request.config);
		set_service.call(srv);
	}
}
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "set_default_param");
	
  ros::NodeHandle private_nh(target_node_name);
  std::string calibration_file;
  private_nh.param<std::string>("calibration_file", calibration_file, "/home/test/20220720_143417_autoware_lidar_camera_calibration.yaml");
	cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
  if (!fs.isOpened())
	{
		ROS_ERROR("[%s] Cannot open file calibration file '%s'", "set_default_param", calibration_file.c_str());
		ros::shutdown();
		return -1;
	}
  cv::Mat CameraExtrinsicMat;
	fs["CameraExtrinsicMat"] >> CameraExtrinsicMat;
  Eigen::Matrix4d M_cam2lidar;
	M_cam2lidar<<CameraExtrinsicMat.at<double>(0, 0), CameraExtrinsicMat.at<double>(0, 1), CameraExtrinsicMat.at<double>(0, 2),CameraExtrinsicMat.at<double>(0, 3),
	CameraExtrinsicMat.at<double>(1, 0), CameraExtrinsicMat.at<double>(1, 1), CameraExtrinsicMat.at<double>(1, 2), CameraExtrinsicMat.at<double>(1, 3),
	CameraExtrinsicMat.at<double>(2, 0), CameraExtrinsicMat.at<double>(2, 1), CameraExtrinsicMat.at<double>(2, 2), CameraExtrinsicMat.at<double>(2, 3),
	0.0, 0.0, 0.0, 1.0;
	
 	pcl::getTranslationAndEulerAngles(Eigen::Affine3d(M_cam2lidar),x_init,y_init,z_init,roll_init,pitch_init,yaw_init);//获取仿射变换等效的x,y,z平移和欧拉角旋转(3-2-1姿态序列)
	//弧度转化为角度
	roll_init=rad2deg(roll_init);
	pitch_init=rad2deg(pitch_init);
	yaw_init=rad2deg(yaw_init);

  //设置初始参数 方法一：dynamic_reconfigure::client
	dynamic_reconfigure::Client<calibration_publisher::CalibConfig> client(target_node_name,boost::bind(&config_callback, _1));  //订阅服务，并设置回调函数用于回读改变后的最新参数
  calibration_publisher::CalibConfig default_config;
	default_config.x = x_init;
	default_config.y = y_init;
	default_config.z = z_init;
	default_config.roll = roll_init;
	default_config.pitch = pitch_init;
	default_config.yaw = yaw_init;
	client.setConfiguration(default_config);//用于更新参数

	// 设置初始参数 方法二：服务 ServiceClient
	set_service = private_nh.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters");
	//等待服务端上线
	set_service.waitForExistence(ros::Duration(-1));//等价于ros::service::waitForService("/calibration_publisher/set_parameters");
	
	ros::spin();
  ros::shutdown();
  return 0;
}