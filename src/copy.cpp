#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "LinuxDef.h"
#include "IRNet.h"

long s_pushtest = -1;
long s_hJpeg = -1;
long s_hCu = -1;
LONG s_hSerial = 0;

IRNETHANDLE m_hchann = IRNETHANDLE(-1);
IRNETHANDLE m_serialconnect = IRNETHANDLE(-1);
char rvsDevInfo[MAX_PATH] = {0};

using namespace cv;

class IraytekDriverNode
{
public:
  IraytekDriverNode(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~IraytekDriverNode();
  void loop();
  void close();

private:
  static void messageCallback(IRNETHANDLE hHandle, WPARAM wParam, LPARAM lParam, void *context);
  static void tempCallback(IRNETHANDLE hdle, float *temp_table, unsigned int imgwidth,  unsigned int imgheight, DEV_TEMP_SPAN*tempspan,void*context);


private:
  static IraytekDriverNode *ptr_;
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher image_pub_;
  int32_t port_, channelno_, streamno_;
  std::string devicename_, ip_, username_, password_;

  sensor_msgs::Image image_;
};

IraytekDriverNode::IraytekDriverNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh)
{
  channelno_ = 0;
  streamno_ = 0;
  private_nh_.param<int>("port", port_, 3000);
  private_nh_.param<std::string>("ip", ip_, "192.168.1.29");
  private_nh_.param<std::string>("devicename", devicename_, "ATF31");
  private_nh_.param<std::string>("username", username_, "888888");
  private_nh_.param<std::string>("password", password_, "888888");
  nh_.advertise<sensor_msgs::Image>("ir_image", 1);

  //
  IRNET_ClientStartup(0, NULL);
  IRNET_ClientWaitTime(30, 4);

  CHANNEL_CLIENTINFO m_chinfo;
  m_chinfo.m_sername = &devicename_[0];
  m_chinfo.m_username = &username_[0];
  m_chinfo.m_password = &password_[0];
  m_chinfo.m_buffnum = 50;
  m_chinfo.m_ch = channelno_;
  m_chinfo.m_hChMsgWnd = NULL;
  m_chinfo.m_hVideohWnd = NULL;
  m_chinfo.m_nChmsgid = 0;
  m_chinfo.m_playstart = FALSE;
  m_chinfo.m_tranType = 3;
  m_chinfo.m_useoverlay = FALSE;
  m_chinfo.nColorKey = 0x00FF00FF;
  m_chinfo.context = (void *)2;
  m_chinfo.m_messagecallback = messageCallback;

  //test connect
  m_hchann = IRNET_ClientStart(&ip_[0], &m_chinfo, port_, streamno_);
  if (m_hchann == IRNETHANDLE(-1))
    ROS_ERROR_STREAM("client start failed!");
  
  //sed callback
  if(!IRNET_ClientRegTempCallBack(m_hchann,tempCallback,NULL,NULL))
	{
		ROS_ERROR_STREAM("reg TempCallback failed!");
	}
	else
	{
    ROS_INFO_STREAM("reg TempCallback successful!");
	}
}

IraytekDriverNode::~IraytekDriverNode()
{
}

void IraytekDriverNode::loop()
{

}

void IraytekDriverNode::close()
{
}

void IraytekDriverNode::messageCallback(IRNETHANDLE hHandle, WPARAM wParam, LPARAM lParam, void *context)
{
}

void IraytekDriverNode::tempCallback(IRNETHANDLE hdle, float *temp_table, unsigned int imgwidth,  unsigned int imgheight, DEV_TEMP_SPAN*tempspan,void*context)
{
  image_pub_.publish(image_);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "iraytek_driver");
  ros::NodeHandle nh, private_nh("~");

  IraytekDriverNode nd(nh, private_nh);

  ros::Rate r(30);

  while (ros::ok())
  {
    ros::spinOnce();
    nd.loop();
    r.sleep();
  }

  nd.close();

  ROS_INFO("All finish");

  return 0;
}