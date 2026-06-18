#ifndef Dbscan
#define Dbscan

#include "Point3.h"
#include "PointDataSource.h"
#include <thread>

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace dbscan {

	template<typename T>
	class  DBSCAN
	{
	public:

		DBSCAN() = default;

		DBSCAN(float Neighbourhood, int MinPts,int threadCount=4)
			:Neighbourhood(Neighbourhood), MinPts(MinPts),threadCount(threadCount) {

		}

		~DBSCAN() {

		}

		std::vector<std::vector<size_t>> GetClusterPointSet(std::vector<Point3<T>>& pointCloud_) 
		{
			
			pointCloud = pointCloud_;
			std::vector<std::vector<int>> temp1;
			std::vector<std::vector<float>> temp2;
			temp1.swap(neighbourPoints);
			temp2.swap(neighbourDistance);

			std::vector<std::vector<size_t>> cluster;
			std::vector<size_t> kernelObj;
			neighbourPoints.resize(pointCloud.size());
			neighbourDistance.resize(pointCloud.size());
			SelectKernelAndNeighbour(kernelObj);

			//迭代标记同一聚类点
			int k = -1;	//初始化聚类簇数
			std::cout << "根据核心对象进行聚类..." << std::endl;
			for (int i = 0; i < kernelObj.size(); i++)
			{
				if (pointCloud[kernelObj[i]].cluster!= NOT_CLASSIFIED)
				{
					continue;
				}
				std::vector<T> queue;
				queue.push_back(kernelObj[i]);
				pointCloud[kernelObj[i]].cluster = ++k;
				while (!queue.empty()) 
				{
					size_t index = queue.back();	//弹出最后一个核心对象
					queue.pop_back();

					if (neighbourPoints[index].size()>MinPts)
					{
						for (int j = 0; j < neighbourPoints[index].size(); j++)
						{
							if (k==pointCloud[neighbourPoints[index][j]].cluster)
							{
								continue;
							}
							queue.push_back(neighbourPoints[index][j]);
							pointCloud[neighbourPoints[index][j]].cluster = k;
						}
					}
				}
			}
			std::cout << "聚类结束" << std::endl;

			cluster.resize(k+1);
			for (size_t i = 0; i < pointCloud.size(); i++)
			{
				if (pointCloud[i].cluster!=NOISE)
				{
					cluster[pointCloud[i].cluster].push_back(i);
				}
			}

			return cluster;
		}

	private:
		//建立核心对象
		void SelectKernelAndNeighbour(std::vector<size_t>& kernelObj) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			cloud->points.resize(pointCloud.size());
			for (size_t i = 0; i < pointCloud.size(); i++)
			{
				cloud->points[i].x = pointCloud[i].x;
				cloud->points[i].y = pointCloud[i].y;
				cloud->points[i].z = pointCloud[i].z;
			}

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud(cloud);
			// std::cout << "开始建立核心对象..." << std::endl;

			//多线程进行数据查询
			size_t span = pointCloud.size() / threadCount;
			std::vector<std::thread*> threads;
			std::vector<std::vector<size_t>> kernelObjTmps(threadCount);		//储存每个线程中所获得的核心
			for (int i = 0; i < threadCount; i++) {
				std::thread* myThread = nullptr;
				if (i < threadCount - 1) {
					myThread = new std::thread(&dbscan::DBSCAN<T>::ConcurrentQuery, this, cloud, span*i,
						span*(i + 1), std::ref(kdtree), std::ref(kernelObjTmps[i]));
				}
				else {
					myThread = new std::thread(&dbscan::DBSCAN<T>::ConcurrentQuery, this, cloud, span*i,
						pointCloud.size(), std::ref(kdtree), std::ref(kernelObjTmps[i]));
				}
				
				// std::cout << "线程" << myThread->get_id() << "创建成功！" << std::endl;
				threads.push_back(myThread);
			}

			for (int i = 0; i < threads.size(); i++) {
				threads[i]->join();
				delete threads[i];
				kernelObj.insert(kernelObj.end(), kernelObjTmps[i].begin(), kernelObjTmps[i].end());
			}

			// std::cout << "核心对象建立结束!" << std::endl;
		}

		//并行运算的函数
		void ConcurrentQuery(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, size_t start, size_t end,
			pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, std::vector<size_t>& kernelObjTmp) {
			for (size_t i = start; i < end; i++)
			{
				kdtree.radiusSearch(cloud->points[i], Neighbourhood, neighbourPoints[i], neighbourDistance[i]);
				if (neighbourPoints[i].size() >= MinPts)
				{
					kernelObjTmp.push_back(i);
				}
				else
				{
					pointCloud[i].cluster = NOISE;
				}
			}
			// std::cout << "线程执行结束！" << std::endl;
		}

	private:
		float Neighbourhood;
		int MinPts;
		int threadCount;
		PointDataSource<T> pointCloud;
		std::vector<std::vector<int>> neighbourPoints;
		std::vector<std::vector<float>> neighbourDistance;		//pcl中的默认距离是float
		
	};
}

#endif // !Dbscan
