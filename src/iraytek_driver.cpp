#include <iostream>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <std_msgs/Bool.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "LinuxDef.h"
#include "IRNet.h"

using namespace cv;

long s_pushtest = -1;
long s_hJpeg = -1;
long s_hCu = -1;
LONG s_hSerial = 0;

IRNETHANDLE m_hchann = IRNETHANDLE(-1);
IRNETHANDLE m_serialconnect = IRNETHANDLE(-1);

//ros::Publisher temp_pub_;
image_transport::Publisher temp_pub_, false_color_pub_;
cv::Mat temp_c_;
cv::Point max_temp_point_, min_temp_point_;
bool pub_false_color_;
float render_max_temp_, render_min_temp_;
int32_t port_, channelno_, streamno_;
std::string devicename_, ip_, username_, password_;

static void rawCallback(char* data, int width, int height, void* context)
{
  
}

static void tempCallback(IRNETHANDLE hdle, float *temp_table, unsigned int imgwidth, unsigned int imgheight, DEV_TEMP_SPAN *tempspan, void *context)
{
  // unsigned int *gmt = (unsigned int *)&temp_table[imgwidth * imgheight + 2];
  // ROS_INFO("first:%.1f w:%d, h:%d GMT:%u", temp_table[10], imgwidth, imgheight, *gmt);

  //cv::Mat m(int rows, int cols, int type);
  cv::Mat temp_c_matrix(imgheight, imgwidth, CV_32FC1);
  cv::Mat temp_k_matrix(imgheight, imgwidth, CV_16UC1);

  float max_temp = 0.0;
  float min_temp = 400.0;

  for (uint32_t r = 0; r < temp_c_matrix.rows; r++)
  {
    for (uint32_t c = 0; c < temp_c_matrix.cols; c++)
    {
      float temp = temp_table[r * temp_c_matrix.cols + c];
      //图像中r行c列的点
			temp_c_matrix.at<float>(r, c) = temp;
      temp_k_matrix.at<uint16_t>(r, c) = uint16_t(temp + 273.15);
      //search max and min temp
      if (temp > max_temp)
      {
        max_temp = temp;
        max_temp_point_ = cv::Point(r, c);
      }
      if (temp < min_temp)
      {
        min_temp = temp;
        min_temp_point_ = cv::Point(r, c);
      }
    }
  }

  temp_c_matrix.copyTo(temp_c_);

  cv_bridge::CvImage out_cv_image;
  temp_k_matrix.copyTo(out_cv_image.image);
  out_cv_image.encoding = "mono16";

  temp_pub_.publish(out_cv_image.toImageMsg());
}

static void nomalvideo(char *pbuff, int headsize, int datasize, int timetick, int biskeyframe, void *context)
{
  //.h264 nalu
  //ROS_INFO_STREAM("recv video data headsize:%d datasize:%d timetick:%d iskeframe:%d!", headsize, datasize, timetick, biskeyframe);
}

static void jpegdatacallback(IRNETHANDLE hHandle, int m_ch, char *pBuffer, int size, void *extraData, void *userdata /*IRNETHANDLE hHandle,int m_ch,char *pBuffer,int size,void *userdata*/)
{
}

static void messageCallback(IRNETHANDLE hHandle, WPARAM wParam, LPARAM lParam, void *context)
{
  switch (wParam)
  {
  case LAUMSG_LINKMSG:
  {
    if (lParam == 0)
    {
      //sMsgData = "连接成功";
      ROS_INFO_STREAM("connect successfull!");
      //test h.264 data
      if (IRNETHANDLE(-1) != m_hchann)
      {
        if (!IRNET_ClientStartNomalCap(m_hchann, nomalvideo, NULL, NULL, NULL))
        {
          ROS_INFO_STREAM("StartNomalCap start failed!!");
        }
        else
        {
          ROS_INFO_STREAM("StartNomalCap start success!!");
        }
      }
      else
      {
        ROS_INFO_STREAM("connect invalid!");
      }  
      //IRNET_ClientStartMp4Capture(hHandle, "/root/Desktop/rec/linuxrec.mp4");
    }
    else if (lParam == 1)
    {
      //sMsgData = "用户停止了连接";
      ROS_INFO_STREAM("user stop connect!");
    }
    else if (lParam == 2)
    {
      //sMsgData = "连接失败";
      ROS_INFO_STREAM("connect failed!");
    }
    else if (lParam == 3)
    {
      //sMsgData = "连接断开";
      ROS_INFO_STREAM("connect dislinked!");
    }
    else if (lParam == 4)
    {
      //sMsgData = "端口冲突";
      ROS_INFO_STREAM("port assert");
    }
    else if (lParam == 5)
    {
      //sMsgData = "分配内存失败";
      ROS_INFO_STREAM("memory assign failed!");
    }
    else if (lParam == 6)
    {
      //sMsgData = "连接域名服务器失败";
      ROS_INFO_STREAM("connect dns failed!");
    }
    else if (lParam == -102)
    {
      //sMsgData = "用户名密码错误";
      ROS_INFO_STREAM("username or password wrong!");
    }
    else if (lParam == -103)
    {
      //sMsgData = "系统用户满员";
      ROS_INFO_STREAM("system user full!");
    }
    else if (lParam == -105)
    {
      //sMsgData = "通道用户满员";
      ROS_INFO_STREAM("channel user full!");
    }
    else if (lParam == -106)
    {
      //sMsgData = "没有指定的通道";
      ROS_INFO_STREAM("no such channel!");
    }
    else if (lParam == -112)
    {
      //sMsgData = "没有找到服务";
      ROS_INFO_STREAM("not find server!");
    }
    else
    {
      //sMsgData = "未知";
    }
    break;
  }
  default:
    //sMsgType = "未知";
    break;
  }
  return;
}

static void pub_false_color_image()
{
  if(pub_false_color_ && !temp_c_.empty())
  {
    cv::Mat filter_temp_c_(temp_c_.rows, temp_c_.cols, CV_8UC1);
    float k = 255.0 / (abs(render_max_temp_) + abs(render_min_temp_));
    for (uint32_t r = 0; r < temp_c_.rows; r++)
    {
      for (uint32_t c = 0; c < temp_c_.cols; c++)
      {
        float temp =  temp_c_.at<float>(r, c);
        if (temp > render_max_temp_)
        {
          filter_temp_c_.at<uint8_t>(r, c) = 255;
        }
        else if (temp < render_min_temp_)
        {
          filter_temp_c_.at<uint8_t>(r, c) = 0;
        }
        else
        {
          filter_temp_c_.at<uint8_t>(r, c) = uint8_t((temp + abs(render_min_temp_)) * k);
        }
      }
    }

    cv::Mat im;
    cv::applyColorMap(filter_temp_c_, im, cv::COLORMAP_JET);
    cv_bridge::CvImage out_cv_image;
    im.copyTo(out_cv_image.image);
    out_cv_image.encoding = "rgb8";
    false_color_pub_.publish(out_cv_image.toImageMsg());
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "iraytek_driver");
  ros::NodeHandle nh_, private_nh_("~");

  channelno_ = 0;
  streamno_ = 2;
  private_nh_.param<int>("port", port_, 3000);
  private_nh_.param<std::string>("ip", ip_, "192.168.1.29");
  private_nh_.param<std::string>("devicename", devicename_, "AT31F");
  private_nh_.param<std::string>("username", username_, "888888");
  private_nh_.param<std::string>("password", password_, "888888");
  private_nh_.param<bool>("pub_false_color", pub_false_color_, false);
  private_nh_.param<float>("render_max_temp", render_max_temp_, 100.0);
  private_nh_.param<float>("render_min_temp", render_min_temp_, 0.000);
  //nh_.advertise<sensor_msgs::Image>("ir_image", 1);
  image_transport::ImageTransport it_(nh_);
  temp_pub_ = it_.advertise("ir/temp_raw", 1);
  false_color_pub_ = it_.advertise("ir/false_color", 1);

  //init irnet sdk
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

  //start connect
  m_hchann = IRNET_ClientStart(&ip_[0], &m_chinfo, port_, streamno_);
  if (m_hchann == IRNETHANDLE(-1))
  {
    ROS_ERROR_STREAM("client start failed!");
  }
  else
  {
    ROS_INFO_STREAM("client start successful!");
  }

  //set Temp callback
  if (!IRNET_ClientRegTempCallBack(m_hchann, tempCallback, NULL, NULL))
  {
    ROS_ERROR_STREAM("reg TempCallback failed!");
  }
  else
  {
    ROS_INFO_STREAM("reg TempCallback successful!");
  }

  // //set raw callback
  // if (!IRNET_ClientRegRawCallback(m_hchann, rawCallback, NULL))
  // {
  //   ROS_ERROR_STREAM("reg rawCallback failed!");
  // }
  // else
  // {
  //   ROS_INFO_STREAM("reg rawCallback successful!");
  // }

  //set messageopt
  IRNETHANDLE handle = IRNET_ClientMessageOpen(&devicename_[0], &ip_[0], &username_[0], &password_[0], port_);
  if (handle == IRNETHANDLE(-1))
  {
    ROS_ERROR_STREAM("MessageOpen failed!");
    return -1;
  }
  else
  {
    ROS_INFO_STREAM("MessageOpen successful!");
  }

  // IRNETHANDLE m_handleJpeg = IRNETHANDLE(-1);
  // if (IRNETHANDLE(-1) == (m_handleJpeg = IRNET_ClientJpegCapStart(&devicename_[0], &ip_[0], &username_[0], &password_[0], port_,
  //                                                                 jpegdatacallback, NULL)))
  // {
  //   ROS_ERROR_STREAM("jpeg start failed!");
  // }
  // if (IRNETHANDLE(-1) != m_handleJpeg && !IRNET_ClientJpegCapSingle(m_handleJpeg, 0, 70))
  // {
  //   ROS_ERROR_STREAM("jpeg single failed!");
  // }

  ros::Rate r(10);

  while (ros::ok())
  {
    ros::spinOnce();
    pub_false_color_image();
    r.sleep();
  }

  ROS_INFO("All finish");

  IRNET_ClientCleanup();

  return 0;
}
