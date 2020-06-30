// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_


#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};


struct KdTree3D
{
	Node* root;

	KdTree3D()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root,point,0,id);
	}

	void insertHelper(Node** node, std::vector<float> point, uint depth, int id) {

		//Case where current tree node is empty
		if(*node == NULL) {
			*node = new Node(point, id);
		}
		//If not empty, we can insert new elements to the left or the right of the last node of the tree
		else {
			//Find the current depth to know if we insert regarding the x (0) or y (1) axis
			uint currentDepth = depth % 3;

			if(point[currentDepth] < ((*node)->point[currentDepth])) {
				insertHelper(&((*node)->left), point, depth+1, id);
			}
			else {
				insertHelper(&((*node)->right), point, depth+1, id);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0 , distanceTol, ids);

		return ids;
	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids) {

		if(node != NULL) {

			//Check if the current node is inside the bounding box of our target
			if(node->point[0] >= (target[0] - distanceTol) && node->point[0]<= (target[0] + distanceTol)
				&& node->point[1] >= (target[1] - distanceTol) && node->point[1]<= (target[1] + distanceTol)
				&& node->point[2] >= (target[2] - distanceTol) && node->point[2]<= (target[2] + distanceTol)) {
					
				float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]) + (node->point[2] - target[2]) * (node->point[2] - target[2]));
				if(distance <= distanceTol) {
					ids.push_back(node->id);
				}
			}

			//check if we need to process the right or the left branch of the current node
			if((target[depth%3] - distanceTol) < node->point[depth%3]) {
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
			if((target[depth%3] + distanceTol) > node->point[depth%3]) {
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}

	}
	

};


template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> CustomSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceToPlane); 

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> CustomClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);


    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
    BoxQ BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */