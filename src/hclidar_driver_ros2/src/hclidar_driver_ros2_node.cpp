

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif


#include <math.h>
#include <cmath>
#include <limits>
#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>


#include <stdio.h>

#include <string> 
#include <sstream>
#include <algorithm>
#include <fstream>

#include "base/HcData.h"
#include "base/HcSDK.h"




std::string  g_strLidarID = "";


void sdkCallBackFunErrorCode(int iErrorCode)
{
	char buff[128] = { 0 };
	sprintf(buff, "Error code=%d\n",iErrorCode);
	printf(buff);
}

void sdkCallBackFunSecondInfo(tsSDKStatistic sInfo)
{
	std::string strFile = "";
	strFile = "FPS_" + g_strLidarID + ".csv";
	std::ofstream outFile;
	outFile.open(strFile, std::ios::app);

	char buff[128] = { 0 };
	sprintf(buff, "%lld,%d,%d,%lld,%0.2f,%d,%d,%d,%lld\n",
		sInfo.u64TimeStampS, sInfo.iNumPerPacket, sInfo.iGrayBytes, sInfo.u64FPS
		, sInfo.dRMS,  sInfo.iValid, sInfo.iInvalid, sInfo.iPacketPerSecond
		, sInfo.u64ErrorPacketCount);

	outFile.write(buff, strlen(buff));
	outFile.close();

	printf(buff);
	
}

void sdkCallBackFunPointCloud(LstPointCloud lstG)
{
 
	std::string strFile = "";
	strFile = "Raw_" + g_strLidarID + ".csv";

	std::ofstream outFile;
	outFile.open(strFile, std::ios::app);

	char buff[128] = { 0 };
	sprintf(buff, "---------------Size=%d\n", lstG.size());
	printf(buff);

    for(auto sInfo : lstG)
    {
		//if (sInfo.dAngle > 310 || sInfo.dAngle < 50)
		//{

			memset(buff, 0, 128);
			sprintf(buff, "%lld,%0.3f,%0.3f,%d,%d,%d,%d\n",
				sInfo.u64TimeStampNs, sInfo.dAngle, sInfo.dAngleRaw, sInfo.u16Dist, sInfo.bValid, sInfo.u16Speed, sInfo.u16Gray);

			outFile.write(buff, strlen(buff));

			//printf(buff);
		//}

		
    }
		
	outFile.close();
	
}


int getPort()
{
	printf("Please select COM:\n");
	int iPort = 3;
    std::cin >> iPort;
	return iPort;
}

int getBaud()
{
	printf("Please select COM baud:\n");
	int iBaud = 115200;
	std::cin >> iBaud;
	return iBaud;
}

std::string getLidarModel()
{
	printf("Please select Lidar model:\n");
	std::string str = "X2M";
	std::cin >> str;
	std::transform(str.begin(), str.end(), str.begin(), ::toupper);
	return str;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("hclidar_driver_ros2_node");

  RCLCPP_INFO(node->get_logger(), "Start  ROS2 Driver\n");

int rtn = 0;

    bool bPollMode = true;
    bool bDistQ2 = false;
    bool bLoop = false;

	std::string strVer = getSDKVersion();
    std::cout << "Main: SDK verion=" << strVer.c_str()<< std::endl;

    auto funErrorCode = std::bind(sdkCallBackFunErrorCode, std::placeholders::_1);
	setSDKCallBackFunErrorCode(funErrorCode);

    auto funSecondInfo = std::bind(sdkCallBackFunSecondInfo, std::placeholders::_1);
    setSDKCallBackFunSecondInfo(funSecondInfo);

    if(!bPollMode)//call back
    {
        auto funPointCloud = std::bind(sdkCallBackFunPointCloud, std::placeholders::_1);
        setSDKCallBackFunPointCloud(funPointCloud);

    }

    
    int iPort = 0;
    std::string strPort;
    strPort = "/dev/ttyUSB" + std::to_string(iPort);              // For Linux OS

    node->declare_parameter("port", "/dev/ttyUSB0");
    node->get_parameter("port", strPort);
    printf( "Port=%s\n" , strPort.c_str());

    int iBaud = 115200;
    node->declare_parameter("baudrate", 115200);
    node->get_parameter("baudrate", iBaud);
    printf( "baudrate=%d\n" , iBaud);

    //std::string strLidarModel = "X2M";
    std::string strLidarModel = "X1";
    node->declare_parameter("lidar_model", strLidarModel);
    node->get_parameter("lidar_model", strLidarModel);
    printf( "strLidarModel=%s\n" , strLidarModel.c_str());


    std::string frame_id = "laser_frame";
    node->declare_parameter("frame_id", "laser_frame");
    node->get_parameter("frame_id", frame_id);
    printf( "frame_id=%s\n" , frame_id.c_str());



    int iReadTimeoutms = 10;

    setSDKCircleDataMode();
    rtn = hcSDKInitialize(strPort.c_str(), strLidarModel.c_str(), iBaud, iReadTimeoutms, bDistQ2, bLoop, bPollMode);

    if (rtn != 1)
    {
      hcSDKUnInit();
      RCLCPP_ERROR(node->get_logger(), "SDK init failed\n");
      exit(0);
      return 0;
          
    }

    setSDKLidarPowerOn(true);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    g_strLidarID = getSDKLidarID();
    
    
    printf( "Lidar ID=%s\n" , getSDKLidarID());
    printf( "Factory Info:%s\n" , getSDKFactoryInfo());
    printf( "Main: Firmware ver:%s\n", getSDKFirmwareVersion() );
    printf( "Main: Hardware ver:%s\n", getSDKHardwareVersion());
    printf( "Main: Lidar model:%s\n" , getSDKLidarModel() );

  
  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  
  float angle_max = 3.141592;
  node->declare_parameter("angle_max", 3.1415926f);
  node->get_parameter("angle_max", angle_max);

  float angle_min = -3.141592;
  node->declare_parameter("angle_min", -3.1415926f);
  node->get_parameter("angle_min", angle_min);

  printf( "Main: angle_min=%f\n" , angle_min );

  float range_max = 10.f;
  node->declare_parameter("range_max", 8.0f);
  node->get_parameter("range_max", range_max);

  float range_min = 0.36f;
  node->declare_parameter("range_min", 0.36f);
  node->get_parameter("range_min", range_min);


  rclcpp::WallRate loop_rate(100);

  rclcpp::Time  start_time = node->get_clock()->now();
  while (rclcpp::ok()) {

    LstPointCloud lstG;
				
				if (getSDKRxPointClouds(lstG))
				{

					if (lstG.size() > 0)
          {
                        rclcpp::Time current_time = node->get_clock()->now();
                        
                        sensor_msgs::msg::LaserScan scan_msg;

                        scan_msg.header.stamp = current_time;
                        scan_msg.header.frame_id = frame_id;
                        scan_msg.angle_min = angle_min; 
                        scan_msg.angle_max = angle_max; 
                        
                        // Fix beam count to 500 points per revolution so SLAM Toolbox has a constant sensor model
                        const int FIXED_BEAM_COUNT = 500;
                        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / static_cast<double>(FIXED_BEAM_COUNT - 1);       
                        
                        // Nominal rotation period for 360 RPM (6 Hz) is ~0.1667s.
                        // Strictly clamp scan_time so pauses, packet drops, or SDK latency never blow up the scan duration.
                        double dt = (current_time.nanoseconds() - start_time.nanoseconds()) / 1e9;
                        if (dt >= 0.08 && dt <= 0.25) {
                            scan_msg.scan_time = dt;
                        } else {
                            scan_msg.scan_time = 0.1667;
                        }
                        
                        // time_increment MUST be 0.0f for 2D SLAM Toolbox / RViz.
                        // Setting time_increment to 0.0f treats all 500 beams as a single rigid snapshot at header.stamp,
                        // completely eliminating ray-by-ray TF warping, distortion across movement, and RViz scan_time warnings!
                        scan_msg.time_increment = 0.0f;
                        
                        scan_msg.range_min = range_min;
                        scan_msg.range_max = range_max;
                        scan_msg.ranges.assign(FIXED_BEAM_COUNT, std::numeric_limits<float>::infinity());
                        scan_msg.intensities.assign(FIXED_BEAM_COUNT, 0.0f);  

						start_time = current_time;
                        
                        double rad_angel;
          
                        for(size_t i = 0; i < lstG.size(); i++)
                        {
                            if(lstG[i].dAngle > 180.0)
                            {
                                rad_angel= (360.0 - lstG[i].dAngle) * 3.141592653589793 / 180.0f;
                            }
                            else
                            {
                                rad_angel= -1.0 * lstG[i].dAngle * 3.141592653589793 / 180.0f;
                            }
                            
                            int index = static_cast<int>(std::round((rad_angel - angle_min) / scan_msg.angle_increment));
                            
                            if(index >= 0 && index < FIXED_BEAM_COUNT) 
                            {
                                float dist_m = (lstG[i].u16Dist)/1000.0f;
                                // Ignore invalid points, or points within robot radius (< range_min) or beyond max
                                if(lstG[i].bValid && dist_m >= range_min && dist_m <= range_max)
                                {
                                    if (std::isinf(scan_msg.ranges[index]) || dist_m < scan_msg.ranges[index])
                                    {
                                        scan_msg.ranges[index] = dist_m;
                                        scan_msg.intensities[index] = lstG[i].u16Gray;
                                    }
                                }
                            }
                        }
                        laser_pub->publish(scan_msg);
                        
          }
				}
				else
				{
					start_time = node->get_clock()->now();
					int iError = getSDKLastErrCode();
					if (iError != LIDAR_SUCCESS)
					{
						printf( "Main: Poll Rx Points error code=%d\n", iError );
						switch (iError)
						{
						case ERR_SHARK_MOTOR_BLOCKED:
							break;
						case ERR_SHARK_INVALID_POINTS:
							break;
						case ERR_LIDAR_SPEED_LOW:
							break;
						case ERR_LIDAR_SPEED_HIGH:
							break;
						case ERR_DISCONNECTED:
							break;
						case ERR_LIDAR_FPS_INVALID:
							break;
						default:
							break;
						}
					}
				}
    if(!rclcpp::ok()) {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
				                
    }

   


  RCLCPP_INFO(node->get_logger(), "Now HCLIDAR is stopping .......");

  rclcpp::shutdown();

  return 0;

}
