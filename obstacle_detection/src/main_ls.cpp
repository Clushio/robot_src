#include "GroundDetector.hpp"
#include "Cluster/DBSCAN.h"
#include "utility.h"
#include "tic_toc.h"

using namespace dbscan;
using namespace std;
typedef double ProcessType;

#define PUB 1

class ObjectDetector
{
private:
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;
    ros::Publisher PubGround;
    ros::Publisher PubUnGround;
    ros::Publisher PubObjectCluster;
    ros::Publisher PubState;

    std_msgs::Header cloudHeader;
    pcl::PointCloud<PointType>::Ptr laserCloudIn; // 存储原始ls数据
    pcl::PointCloud<PointType>::Ptr laserCloudIn2;

    pcl::PointCloud<PointType>::Ptr laserCloudOut_ground;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudOut_clusters; // rgb
    
    // 参数列表

    // 话题名
    std::string stopic;

    // 危险停止区域
    float fXLowerBoundary1;
    float fXUpperBoundary1;
    float fYLowerBoundary1;
    float fYUpperBoundary1;
    pcl::PassThrough<PointType> Xpassthrough1;
    pcl::PassThrough<PointType> Ypassthrough1;

    // 不可靠减速区域
    float fXLowerBoundary2;
    float fXUpperBoundary2;
    float fYLowerBoundary2;
    float fYUpperBoundary2;
    pcl::PassThrough<PointType> Xpassthrough2;
    pcl::PassThrough<PointType> Ypassthrough2;

    // Z方向点检测
    bool bzLimits_;
    float z_lower_boundary_; 
    pcl::PassThrough<PointType> Zpassthrough;

    float z_upper_boundary_; 
 //   pcl::PassThrough<PointType> Zpassthrough_up;

    // 地面检测器
    GroundDetector* mGroundDetector;
    bool bground_detector_;
    int num_segments_;
    double sensor_height_;
    int num_iter_;
    int num_lpr_;
    double th_seeds_;
    double th_dist_;

    // DBSCAN聚类
    DBSCAN<ProcessType>* mDBSCAN;
    bool bcluster_;
    float neighbourhood_;
    int min_pts_;
    int thread_count_;
    

public:
    ObjectDetector()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudIn2.reset(new pcl::PointCloud<PointType>());
        laserCloudOut_ground.reset(new pcl::PointCloud<PointType>());
        laserCloudOut_clusters.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

        // 外部传入参数
        nh.param<std::string>("object_detector/topic", stopic, "/points_raw");

        nh.param<float>("object_detector/XLowerBoundary1", fXLowerBoundary1, 0);
        nh.param<float>("object_detector/XUpperBoundary1", fXUpperBoundary1, 1.5);
        nh.param<float>("object_detector/YLowerBoundary1", fYLowerBoundary1, -0.5);
        nh.param<float>("object_detector/YUpperBoundary1", fYUpperBoundary1, 0.5);

        nh.param<float>("object_detector/XLowerBoundary2", fXLowerBoundary2, 0);
        nh.param<float>("object_detector/XUpperBoundary2", fXUpperBoundary2, 3);
        nh.param<float>("object_detector/YLowerBoundary2", fYLowerBoundary2, -1);
        nh.param<float>("object_detector/YUpperBoundary2", fYUpperBoundary2, 1);

        nh.param<bool>("object_detector/use_z_limit", bzLimits_, false);
        nh.param<float>("object_detector/ZLowerBoundary", z_lower_boundary_, -5);
        nh.param<float>("object_detector/ZUpperBoundary", z_upper_boundary_, 1.5);

        nh.param<bool>("object_detector/use_ground_detector", bground_detector_, true);
        nh.param<int>("object_detector/NumSegments", num_segments_, 3);
        nh.param<double>("object_detector/SensorHeight", sensor_height_, 0.3);
        nh.param<int>("object_detector/NumIter", num_iter_, 3);
        nh.param<int>("object_detector/NumLpr", num_lpr_, 250);
        nh.param<double>("object_detector/SeedTH", th_seeds_, 0.2);
        nh.param<double>("object_detector/DistanceTH", th_dist_, 0.1);

        nh.param<bool>("object_detector/use_dbscan_cluster", bcluster_, true);
        nh.param<float>("object_detector/NeighbourhoodTH", neighbourhood_, 0.2);
        nh.param<int>("object_detector/MinPts", min_pts_, 5);
        nh.param<int>("object_detector/ThreadCount", thread_count_, 4);

        // 初始化地面检测器和聚类模块
        mGroundDetector = new GroundDetector(3, sensor_height_, num_iter_, num_lpr_, th_seeds_, th_dist_);
        mDBSCAN = new DBSCAN<ProcessType>(neighbourhood_, min_pts_, thread_count_);

        // 初始化直通滤波器
        std::string filterFieldx = "x";
        std::string filterFieldy = "y";
        Xpassthrough1.setFilterFieldName(filterFieldx);
        Xpassthrough1.setFilterLimits(fXLowerBoundary1, fXUpperBoundary1);

        Ypassthrough1.setFilterFieldName(filterFieldy);
        Ypassthrough1.setFilterLimits(fYLowerBoundary1, fYUpperBoundary1);

        Xpassthrough2.setFilterFieldName(filterFieldx);
        Xpassthrough2.setFilterLimits(fXLowerBoundary2, fXUpperBoundary2);

        Ypassthrough2.setFilterFieldName(filterFieldy);
        Ypassthrough2.setFilterLimits(fYLowerBoundary2, fYUpperBoundary2);

        Zpassthrough.setFilterFieldName("z");
        Zpassthrough.setFilterLimits(z_lower_boundary_, z_upper_boundary_);

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(stopic, 1, &ObjectDetector::cloudHandler, this);  

        PubGround = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_detection/ground_pc", 1); 
        PubUnGround = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_detection/unground_pc", 1);   
        PubObjectCluster = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_detection/obstacle_pc", 1);   
        PubState = nh.advertise<std_msgs::Int32>("/obstacle_detection/state", 1);  
    }

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        // 将ROS中的sensor_msgs::PointCloud2ConstPtr类型转换到pcl点云库指针
        laserCloudIn->clear();
        cloudHeader = laserCloudMsg->header;
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	    for (size_t i = 0; i < laserCloudIn->points.size(); ++i)
	    {
		    if (laserCloudIn->points[i].x == 0 && laserCloudIn->points[i].y == 0 && laserCloudIn->points[i].z == 0)
		    {
			    indices->indices.push_back(i); // 将点的索引添加到indices中
		    }
	    }

        laserCloudIn2->clear();
	    pcl::ExtractIndices<PointType> extract;
	    extract.setInputCloud(laserCloudIn);
	    extract.setIndices(indices);
	    extract.setNegative(true); // 保留指定索引的点，删除其他点
	    extract.filter(*laserCloudIn2); // 应用提取方法，并将结果存储在 laserCloudIn2 中

    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        int cur_state = -1;

        copyPointCloud(laserCloudMsg);

        std::cout << "laserCloudIn->points.size(): " << laserCloudIn->points.size() << std::endl;

        PointType point;
        if(!bground_detector_ && !bzLimits_)
        {
            ROS_INFO("\033[1;31m----> param use_ground_detector or use_z_limit use at least one! \033[0m");
            return;
        }

        // 分离地面，获取非地面点云
        pcl::PointCloud<PointType>::Ptr unground_pc(new pcl::PointCloud<PointType>);
        if(bground_detector_)
        {
            // TicToc t_ground;
            mGroundDetector->DetectGround(laserCloudIn2);
            std::vector<pcl::PointCloud<PointType>::Ptr> vRPCs = mGroundDetector->getRawPointCloudWithLabel();
            for(int i=0; i<vRPCs.size(); i++)
            {
                for(int j=0; j<vRPCs[i]->points.size(); j++)
                {
                    if((int)vRPCs[i]->points[j].intensity==0)
                    unground_pc->push_back(vRPCs[i]->points[j]);
                }
            }

            // std::cout << "ground detection time: " << t_ground.toc() << " ms" << std::endl;

            if(1)
            {
                laserCloudOut_ground->clear();
                std::vector<pcl::PointCloud<PointType>::Ptr> vGroundMeas = mGroundDetector->getDetect3dGround();
                for(int i=0; i<vGroundMeas.size(); i++)
                {
                    *laserCloudOut_ground += *vGroundMeas[i];
                }

                sensor_msgs::PointCloud2 laserCloudground;
		        pcl::toROSMsg(*laserCloudOut_ground, laserCloudground);
		        laserCloudground.header.stamp = cloudHeader.stamp;
		        laserCloudground.header.frame_id = cloudHeader.frame_id;
	    	    PubGround.publish(laserCloudground);
            }
        }
        else
            unground_pc = laserCloudIn2;

        pcl::PointCloud<PointType>::Ptr unground_pc2(new pcl::PointCloud<PointType>);
        if(bzLimits_)
        {
            Zpassthrough.setInputCloud(unground_pc);
            Zpassthrough.filter(*unground_pc2);
        }
        else
            unground_pc2 = unground_pc;

        // 大的xy边界点云筛选
        pcl::PointCloud<PointType>::Ptr large_range_pc_x(new pcl::PointCloud<PointType>);
        Xpassthrough1.setInputCloud(unground_pc2);
        Xpassthrough1.filter(*large_range_pc_x);

        pcl::PointCloud<PointType>::Ptr large_range_pc_xy(new pcl::PointCloud<PointType>);        
        Ypassthrough1.setInputCloud(large_range_pc_x);
        Ypassthrough1.filter(*large_range_pc_xy);

        // 发布非地面不可靠区域点云
        if(1)
        {
            sensor_msgs::PointCloud2 laserCloudUnground;
		    pcl::toROSMsg(*unground_pc2, laserCloudUnground);
		    laserCloudUnground.header.stamp = cloudHeader.stamp;
		    laserCloudUnground.header.frame_id = cloudHeader.frame_id;
	    	PubUnGround.publish(laserCloudUnground);
        }

        if(bcluster_)
        {
            if(large_range_pc_xy->size())
            {
                // 进行点云聚类，保留尺寸足够大的聚类结果
                std::vector<Point3<ProcessType>> pointCloud;
                for (size_t i = 0; i < large_range_pc_xy->size(); i++)
	            {
		            pointCloud.emplace_back((*large_range_pc_xy)[i].x, (*large_range_pc_xy)[i].y, (*large_range_pc_xy)[i].z);
	            }
                std::vector<std::vector<size_t>> cluster = mDBSCAN->GetClusterPointSet(pointCloud);

                int real_obstacle_num = 0;
                for(int i = 0; i < cluster.size(); i++)
                {
                    if(cluster[i].size()>0)
                    {
                        real_obstacle_num++;
                    }
                }
                
                std::cout << "聚类数目为：" << real_obstacle_num << std::endl;

                if(!real_obstacle_num)
                {
                    std::cout << "state1-大区域内无障碍！ 正常工作" << std::endl << std::endl;
                    cur_state = 1;
                }
                else
                {
                    pcl::PointXYZRGB rgb_point;
                    laserCloudOut_clusters->clear();
                    pcl::PointCloud<PointType>::Ptr cluster_pc(new pcl::PointCloud<PointType>);  
                    for(int i = 0; i < cluster.size(); i++)
                    {
                        int r,g,b;
                        r = (rand() % (255-0+1)) + 0;
                        g = (rand() % (255-0+1)) + 0;
                        b = (rand() % (255-0+1)) + 0;
                        for (size_t j = 0; j < cluster[i].size(); j++)
                        {
                            point.x = pointCloud[cluster[i][j]].x;
                            point.y = pointCloud[cluster[i][j]].y;
                            point.z = pointCloud[cluster[i][j]].z;
                            cluster_pc->push_back(point);

                            if(PUB)
                            {
                                rgb_point.x = point.x;
                                rgb_point.y = point.y;
                                rgb_point.z = point.z;
                                rgb_point.r = r;
                                rgb_point.g = g;
                                rgb_point.b = b;
                                laserCloudOut_clusters->push_back(rgb_point);
                            }
                        }
                        std::cout << "laserCloudOut_clusters->size():" << laserCloudOut_clusters->size() << std::endl;
                    }

                    if(PUB)
                    {
                        sensor_msgs::PointCloud2 laserCloudCluster;
                        pcl::toROSMsg(*laserCloudOut_clusters, laserCloudCluster);
                        laserCloudCluster.header.stamp = cloudHeader.stamp;
                        laserCloudCluster.header.frame_id = cloudHeader.frame_id;
                        PubObjectCluster.publish(laserCloudCluster);
                    }

                    pcl::PointCloud<PointType>::Ptr small_range_pc_x(new pcl::PointCloud<PointType>);
                    Xpassthrough2.setInputCloud(cluster_pc);
                    Xpassthrough2.filter(*small_range_pc_x);

                    pcl::PointCloud<PointType>::Ptr small_range_pc_xy(new pcl::PointCloud<PointType>);        
                    Ypassthrough2.setInputCloud(small_range_pc_x);
                    Ypassthrough2.filter(*small_range_pc_xy);

                    if(small_range_pc_xy->size())
                    {
                        std::cout << "state3-小区域内有障碍！ 停止" << std::endl << std::endl;
                        cur_state = 3;
                    }
                    else
                    {
                        std::cout << "state2-大区域内有障碍! 减速！" << std::endl << std::endl;
                        cur_state = 2;
                    }
                }
            }
            else
            {
                std::cout << "state1-大区域内无障碍！ 正常工作" << std::endl << std::endl;
                cur_state = 1;
            }
        }
        else
        {
            if(large_range_pc_xy->size())
            {
                pcl::PointCloud<PointType>::Ptr small_range_pc_x(new pcl::PointCloud<PointType>);
                Xpassthrough2.setInputCloud(large_range_pc_xy);
                Xpassthrough2.filter(*small_range_pc_x);

                pcl::PointCloud<PointType>::Ptr small_range_pc_xy(new pcl::PointCloud<PointType>);        
                Ypassthrough2.setInputCloud(small_range_pc_x);
                Ypassthrough2.filter(*small_range_pc_xy);

                if(small_range_pc_xy->size())
                {
                    std::cout << "state3-小区域内有障碍！ 停止" << std::endl << std::endl;
                    cur_state = 3;
                }
                else
                {
                    std::cout << "state2-大区域内有障碍! 减速！" << std::endl << std::endl;
                    cur_state = 2;
                }
            }
            else
            {
                std::cout << "state1-大区域内无障碍！ 正常工作" << std::endl << std::endl;
                cur_state = 1;
            }
        }
        if(PUB)
        {
            std_msgs::Int32 RobotState;
            RobotState.data = cur_state;
            PubState.publish(RobotState);
        }
    }
};



int main(int argc, char** argv) 
{

    ros::init(argc, argv, "obstacle_detection");
    
    ObjectDetector OD;

    ROS_INFO("\033[1;32m---->\033[0m obstacle detection Started.");

    ros::spin();

    return 0;
}