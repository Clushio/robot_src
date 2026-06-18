/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Bhaskara Marthi
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/prepare.h>
#include <myglobal_planner/planner_core.h>
#include <navfn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/layered_costmap.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include <costmap_2d/costmap_math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
using namespace std;
using namespace cv;
using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using cm::Costmap2D;
using cm::Costmap2DROS;
using cm::LayeredCostmap;
using rm::PoseStamped;
using std::string;
using std::vector;

class SubAndPub
{
public:
    SubAndPub()
    {
         sub1 = nh.subscribe("/map", 1, &SubAndPub::sub_callback1, this);
        sub = nh.subscribe("/costmap", 1, &SubAndPub::sub_callback, this);
        string path="/home/znfs/Downloads/0.bmp";
        Mat depImg;
        Mat resImg1=depthimg_read22(path,depImg);
        cv::imshow("res",resImg1);
        waitKey(3000);
        
    }


Mat depthimg_read22(const std::string path, cv::Mat& depimg)
{
 cv::Mat saveimg = cv::imread(path);
   cv::imshow("saveimg",saveimg);
 depimg = cv::Mat(saveimg.size(), CV_16UC1);
 for (int i = 0; i < depimg.rows; ++i)
 {
  const uchar* save = saveimg.ptr<uchar>(i);
  
  unsigned short int* dep = depimg.ptr<unsigned short int>(i);
  for (int j = 0; j < depimg.cols; ++j)
  {
   //*dep++ = static_cast<unsigned short int>((*save << 8) | (*(save + 1)));
      cout<<(int)*save<<endl;
   *dep++ = static_cast<unsigned short int>(*save + *(save + 1) * 255);
   save += 3;
  cout<<*dep<<endl;
  }
 }
 return depimg;
}
    void chao_thinimage(Mat &srcimage) //单通道、二值化后的图像
    {
        vector<Point> deletelist1;
        int Zhangmude[9];
        int nl = srcimage.rows;
        int nc = srcimage.cols;
        while (true)
        {
            for (int j = 1; j < (nl - 1); j++)
            {
                uchar *data_last = srcimage.ptr<uchar>(j - 1);
                uchar *data = srcimage.ptr<uchar>(j);
                uchar *data_next = srcimage.ptr<uchar>(j + 1);
                for (int i = 1; i < (nc - 1); i++)
                {
                    if (data[i] == 255)
                    {
                        Zhangmude[0] = 1;
                        if (data_last[i] == 255)
                            Zhangmude[1] = 1;
                        else
                            Zhangmude[1] = 0;
                        if (data_last[i + 1] == 255)
                            Zhangmude[2] = 1;
                        else
                            Zhangmude[2] = 0;
                        if (data[i + 1] == 255)
                            Zhangmude[3] = 1;
                        else
                            Zhangmude[3] = 0;
                        if (data_next[i + 1] == 255)
                            Zhangmude[4] = 1;
                        else
                            Zhangmude[4] = 0;
                        if (data_next[i] == 255)
                            Zhangmude[5] = 1;
                        else
                            Zhangmude[5] = 0;
                        if (data_next[i - 1] == 255)
                            Zhangmude[6] = 1;
                        else
                            Zhangmude[6] = 0;
                        if (data[i - 1] == 255)
                            Zhangmude[7] = 1;
                        else
                            Zhangmude[7] = 0;
                        if (data_last[i - 1] == 255)
                            Zhangmude[8] = 1;
                        else
                            Zhangmude[8] = 0;
                        int whitepointtotal = 0;
                        for (int k = 1; k < 9; k++)
                        {
                            whitepointtotal = whitepointtotal + Zhangmude[k];
                        }
                        if ((whitepointtotal >= 2) && (whitepointtotal <= 6))
                        {
                            int ap = 0;
                            if ((Zhangmude[1] == 0) && (Zhangmude[2] == 1))
                                ap++;
                            if ((Zhangmude[2] == 0) && (Zhangmude[3] == 1))
                                ap++;
                            if ((Zhangmude[3] == 0) && (Zhangmude[4] == 1))
                                ap++;
                            if ((Zhangmude[4] == 0) && (Zhangmude[5] == 1))
                                ap++;
                            if ((Zhangmude[5] == 0) && (Zhangmude[6] == 1))
                                ap++;
                            if ((Zhangmude[6] == 0) && (Zhangmude[7] == 1))
                                ap++;
                            if ((Zhangmude[7] == 0) && (Zhangmude[8] == 1))
                                ap++;
                            if ((Zhangmude[8] == 0) && (Zhangmude[1] == 1))
                                ap++;
                            if (ap == 1)
                            {
                                if ((Zhangmude[1] * Zhangmude[7] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[5] * Zhangmude[7] == 0))
                                {
                                    deletelist1.push_back(Point(i, j));
                                }
                            }
                        }
                    }
                }
            }
            if (deletelist1.size() == 0)
                break;
            for (size_t i = 0; i < deletelist1.size(); i++)
            {
                Point tem;
                tem = deletelist1[i];
                uchar *data = srcimage.ptr<uchar>(tem.y);
                data[tem.x] = 0;
            }
            deletelist1.clear();

            for (int j = 1; j < (nl - 1); j++)
            {
                uchar *data_last = srcimage.ptr<uchar>(j - 1);
                uchar *data = srcimage.ptr<uchar>(j);
                uchar *data_next = srcimage.ptr<uchar>(j + 1);
                for (int i = 1; i < (nc - 1); i++)
                {
                    if (data[i] == 255)
                    {
                        Zhangmude[0] = 1;
                        if (data_last[i] == 255)
                            Zhangmude[1] = 1;
                        else
                            Zhangmude[1] = 0;
                        if (data_last[i + 1] == 255)
                            Zhangmude[2] = 1;
                        else
                            Zhangmude[2] = 0;
                        if (data[i + 1] == 255)
                            Zhangmude[3] = 1;
                        else
                            Zhangmude[3] = 0;
                        if (data_next[i + 1] == 255)
                            Zhangmude[4] = 1;
                        else
                            Zhangmude[4] = 0;
                        if (data_next[i] == 255)
                            Zhangmude[5] = 1;
                        else
                            Zhangmude[5] = 0;
                        if (data_next[i - 1] == 255)
                            Zhangmude[6] = 1;
                        else
                            Zhangmude[6] = 0;
                        if (data[i - 1] == 255)
                            Zhangmude[7] = 1;
                        else
                            Zhangmude[7] = 0;
                        if (data_last[i - 1] == 255)
                            Zhangmude[8] = 1;
                        else
                            Zhangmude[8] = 0;
                        int whitepointtotal = 0;
                        for (int k = 1; k < 9; k++)
                        {
                            whitepointtotal = whitepointtotal + Zhangmude[k];
                        }
                        if ((whitepointtotal >= 2) && (whitepointtotal <= 6))
                        {
                            int ap = 0;
                            if ((Zhangmude[1] == 0) && (Zhangmude[2] == 1))
                                ap++;
                            if ((Zhangmude[2] == 0) && (Zhangmude[3] == 1))
                                ap++;
                            if ((Zhangmude[3] == 0) && (Zhangmude[4] == 1))
                                ap++;
                            if ((Zhangmude[4] == 0) && (Zhangmude[5] == 1))
                                ap++;
                            if ((Zhangmude[5] == 0) && (Zhangmude[6] == 1))
                                ap++;
                            if ((Zhangmude[6] == 0) && (Zhangmude[7] == 1))
                                ap++;
                            if ((Zhangmude[7] == 0) && (Zhangmude[8] == 1))
                                ap++;
                            if ((Zhangmude[8] == 0) && (Zhangmude[1] == 1))
                                ap++;
                            if (ap == 1)
                            {
                                if ((Zhangmude[1] * Zhangmude[3] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[1] * Zhangmude[7] == 0))
                                {
                                    deletelist1.push_back(Point(i, j));
                                }
                            }
                        }
                    }
                }
            }
            if (deletelist1.size() == 0)
                break;
            for (size_t i = 0; i < deletelist1.size(); i++)
            {
                Point tem;
                tem = deletelist1[i];
                uchar *data = srcimage.ptr<uchar>(tem.y);
                data[tem.x] = 0;
            }
            deletelist1.clear();
        }
    }
    void zhangSkeleton(Mat &srcimage)
    {
        int kernel[9];
        int nl = srcimage.rows;
        int nc = srcimage.cols;
        vector<Point> delete_list;
        int A, B;
        while (true)
        {
            for (int j = 1; j < nl - 1; j++)
            {
                uchar *data_pre = srcimage.ptr<uchar>(j - 1);
                uchar *data = srcimage.ptr<uchar>(j);
                uchar *data_next = srcimage.ptr<uchar>(j + 1);
                for (int i = 1; i < (nc - 1); i++)
                {
                    if (data[i] == 255)
                    {
                        kernel[0] = 1;
                        if (data_pre[i] == 255)
                            kernel[1] = 1;
                        else
                            kernel[1] = 0;
                        if (data_pre[i + 1] == 255)
                            kernel[2] = 1;
                        else
                            kernel[2] = 0;
                        if (data[i + 1] == 255)
                            kernel[3] = 1;
                        else
                            kernel[3] = 0;
                        if (data_next[i + 1] == 255)
                            kernel[4] = 1;
                        else
                            kernel[4] = 0;
                        if (data_next[i] == 255)
                            kernel[5] = 1;
                        else
                            kernel[5] = 0;
                        if (data_next[i - 1] == 255)
                            kernel[6] = 1;
                        else
                            kernel[6] = 0;
                        if (data[i - 1] == 255)
                            kernel[7] = 1;
                        else
                            kernel[7] = 0;
                        if (data_pre[i - 1] == 255)
                            kernel[8] = 1;
                        else
                            kernel[8] = 0;

                        B = 0;
                        for (int k = 1; k < 9; k++)
                        {
                            B = B + kernel[k];
                        }
                        if ((B >= 2) && (B <= 6))
                        {
                            A = 0;
                            if (!kernel[1] && kernel[2])
                                A++;
                            if (!kernel[2] && kernel[3])
                                A++;
                            if (!kernel[3] && kernel[4])
                                A++;
                            if (!kernel[4] && kernel[5])
                                A++;
                            if (!kernel[5] && kernel[6])
                                A++;
                            if (!kernel[6] && kernel[7])
                                A++;
                            if (!kernel[7] && kernel[8])
                                A++;
                            if (!kernel[8] && kernel[1])
                                A++;
                            //
                            if (A == 1)
                            {
                                if ((kernel[1] * kernel[3] * kernel[5] == 0) && (kernel[3] * kernel[5] * kernel[7] == 0))
                                {
                                    delete_list.push_back(Point(i, j));
                                }
                            }
                        }
                    }
                }
            }
            int size = delete_list.size();
            if (size == 0)
            {
                break;
            }
            for (int n = 0; n < size; n++)
            {
                Point tem;
                tem = delete_list[n];
                uchar *data = srcimage.ptr<uchar>(tem.y);
                data[tem.x] = 0;
            }
            delete_list.clear();
            for (int j = 1; j < nl - 1; j++)
            {
                uchar *data_pre = srcimage.ptr<uchar>(j - 1);
                uchar *data = srcimage.ptr<uchar>(j);
                uchar *data_next = srcimage.ptr<uchar>(j + 1);
                for (int i = 1; i < (nc - 1); i++)
                {
                    if (data[i] == 255)
                    {
                        kernel[0] = 1;
                        if (data_pre[i] == 255)
                            kernel[1] = 1;
                        else
                            kernel[1] = 0;
                        if (data_pre[i + 1] == 255)
                            kernel[2] = 1;
                        else
                            kernel[2] = 0;
                        if (data[i + 1] == 255)
                            kernel[3] = 1;
                        else
                            kernel[3] = 0;
                        if (data_next[i + 1] == 255)
                            kernel[4] = 1;
                        else
                            kernel[4] = 0;
                        if (data_next[i] == 255)
                            kernel[5] = 1;
                        else
                            kernel[5] = 0;
                        if (data_next[i - 1] == 255)
                            kernel[6] = 1;
                        else
                            kernel[6] = 0;
                        if (data[i - 1] == 255)
                            kernel[7] = 1;
                        else
                            kernel[7] = 0;
                        if (data_pre[i - 1] == 255)
                            kernel[8] = 1;
                        else
                            kernel[8] = 0;

                        B = 0;
                        for (int k = 1; k < 9; k++)
                        {
                            B = B + kernel[k];
                        }
                        if ((B >= 2) && (B <= 6))
                        {
                            A = 0;
                            if (!kernel[1] && kernel[2])
                                A++;
                            if (!kernel[2] && kernel[3])
                                A++;
                            if (!kernel[3] && kernel[4])
                                A++;
                            if (!kernel[4] && kernel[5])
                                A++;
                            if (!kernel[5] && kernel[6])
                                A++;
                            if (!kernel[6] && kernel[7])
                                A++;
                            if (!kernel[7] && kernel[8])
                                A++;
                            if (!kernel[8] && kernel[1])
                                A++;
                            //
                            if (A == 1)
                            {
                                if ((kernel[1] * kernel[3] * kernel[7] == 0) && (kernel[1] * kernel[5] * kernel[7] == 0))
                                {
                                    delete_list.push_back(Point(i, j));
                                }
                            }
                        }
                    }
                }
            }
            if (size == 0)
            {
                break;
            }
            for (int n = 0; n < size; n++)
            {
                Point tem;
                tem = delete_list[n];
                uchar *data = srcimage.ptr<uchar>(tem.y);
                data[tem.x] = 0;
            }
            delete_list.clear();
        }
    }
    void sub_callback1(const nav_msgs::OccupancyGridConstPtr &new_map)
    {
        cout << "开始订阅" << endl;
        unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
        result = Mat::zeros((int)size_y, (int)size_x, CV_8UC1);
        dstImage = Mat::zeros((int)size_y, (int)size_x, CV_8UC1);
        cv::Mat srcImg((int)size_y, (int)size_x, CV_8UC1);
        cv::Mat binaryImage((int)size_y, (int)size_x, CV_8UC1);
        Mat fit((int)size_y, (int)size_x, CV_8UC3);
        unsigned int index = 0;
        for (unsigned int i = 0; i < size_y; ++i)
        {
            for (unsigned int j = 0; j < size_x; ++j)
            {
                unsigned char value = new_map->data[index];
                int row = (int)index / (int)size_x;
                int col = (int)index % (int)size_x;
                if (value <= 100)
                {
                    srcImg.at<uchar>(row, col) = 255;
                }
                else
                {
                    srcImg.at<uchar>(row, col) = 0;
                }
                index++;
            }
        }
        cv::imshow("input", srcImg);

        GaussianBlur(srcImg, binaryImage, Size(5, 5), 0);
        cv::imshow("gaussion", binaryImage);
        Mat element = getStructuringElement(MORPH_RECT, Size(7, 7));
        morphologyEx(binaryImage, binaryImage, MORPH_OPEN, element);
        cv::imshow("open", binaryImage);

        chao_thinimage(binaryImage);
        cv::imshow("skeleton", binaryImage);

        for (int p = 0; p < binaryImage.rows; p++)
        {
            for (int q = 0; q < binaryImage.cols; q++)
            {
                // cout<<(int)binaryImage.at<uchar>(p, q)<<" ";
                if (binaryImage.at<uchar>(p, q) == 255 && srcImg.at<uchar>(p, q) == 255)
                {
                    points.push_back(Point(p, q));
                    result.at<uchar>(p, q) = 255;
                }
            }
            // cout<<endl;
        }
        cv::imshow("output", result);
        vector<vector<Point>> contours;
        findContours(result, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        vector<vector<Point>> contours_ploy(contours.size());
        for (int i = 0; i < contours.size(); i++)
        {
            //epsilon==3
            approxPolyDP(Mat(contours[i]), contours_ploy[i], 3, false);
            drawContours(dstImage, contours_ploy, i, cv::Scalar(255), 1, 8);
        }
        imshow("poly", dstImage);

        //cv::waitKey(2000);
    }
    void sub_callback(const nav_msgs::OccupancyGridConstPtr &new_map)
    {
        
        unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
        LayeredCostmap *layered_costmap_;
        bool rolling_window = false;
        bool track_unknown_space = false;
        string global_frame_ = new_map->header.frame_id;
  
        layered_costmap_ = new LayeredCostmap(global_frame_, rolling_window, track_unknown_space);
        layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                    new_map->info.origin.position.y, true);
        unsigned int index1 = 0;
        Costmap2D *master = layered_costmap_->getCostmap();
        unsigned char *master_array = master->getCharMap();
        for (unsigned int i = 0; i < size_y; ++i)
        {
            for (unsigned int j = 0; j < size_x; ++j)
            {
                unsigned char value = new_map->data[index1];
                //将订阅拿到的map数据拷贝到costmap_2d数据成员`costmap_`
                master_array[index1] = value;
                //  cout << (int)master_array[index1] << " ";

                ++index1;
            }
            //out << endl;
        }
        //CoveragePlanner cp("planner",layered_costmap_);

        // Inflation inflat;
        // inflat.updateCosts(master_array, dstImage);
        // unsigned int index2 = 0;
        // for (unsigned int i = 0; i < size_y; ++i)
        // {
        //     for (unsigned int j = 0; j < size_x; ++j)
        //     {
        //         if (dstImage.at<uchar>(i, j) == 255)
        //         {
        //             master_array[index2] = 0;
         
        //         }
    
        //         ++index2;
        //     }
        // }
        //PlannerWithCostmap pwc("planner", layered_costmap_);
    }

private:
    ros::Subscriber sub;
    ros::Subscriber sub1;
    ros::NodeHandle nh;
    cv::Mat result;
    cv::Mat dstImage;
    vector<Point> points;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner");
    SubAndPub sap;
    ros::Rate r(10);
   while(ros::ok()){
       ros::spinOnce();
       r.sleep();

   }
    
    return 0;
}
