// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    //Create the filtering object
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes );
    vg.filter(*cloudFiltered);


    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);


    //Remove points corresponding to the car's roof
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //Extract the inliers
    for(int point : indices) {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr roadPointCloudPtr(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstaclesPointCloudPtr(new pcl::PointCloud<PointT>);

    //Extract the inliers
    for(int i : inliers->indices) {
        roadPointCloudPtr->points.push_back(cloud->points[i]);
    }

    pcl::ExtractIndices<PointT> extract;
    //Extract the obstacles
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    //If false, the filter would be used to get the plane cloud
    extract.setNegative(true);
    extract.filter(* obstaclesPointCloudPtr);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclesPointCloudPtr, roadPointCloudPtr);
    return segResult;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    //Optional
    seg.setOptimizeCoefficients(true);
    //Mandatory(Hyperparameters)
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;


    if(inliers->indices.size() == 0) {
        std::cout << "Could not estimate a planar model for the given dataset" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices; //Each element of this vector contains all the indices of the points of a specific cluster

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices); //Clusters exctraction filled in cluster_indices

    //Separate each cluster
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


/* Custom RANSAC implementation with plane model */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceToPlane) {

    pcl::PointIndices::Ptr indices {new pcl::PointIndices};
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	while(maxIterations--) {

		//Select 3 Random points from the point cloud in order to compute the plane model
		std::unordered_set<int> inliers;
		while(inliers.size() < 3) {
			inliers.insert(rand()%(cloud->points.size()));
		}

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto it = inliers.begin();
		x1 = cloud->points[*it].x;
		y1 = cloud->points[*it].y;
		z1 = cloud->points[*it].z;
		it++;
		x2 = cloud->points[*it].x;
		y2 = cloud->points[*it].y;
		z2 = cloud->points[*it].z;
		it++;
		x3 = cloud->points[*it].x;
		y3 = cloud->points[*it].y;
		z3 = cloud->points[*it].z;


		// A plane is defined by 2 vectors and a point.
		std::vector<float> v1{x2 - x1, y2 - y1, z2 - z1};
		std::vector<float> v2{x3 - x1, y3 - y1, z3 - z1};

		//Find the normal vector to the plane and then the coefficient A,B,C and D of the plane equation
		float a = v1[1] * v2[2] - v1[2] * v2[1]; 
		float b = v1[2] * v2[0] - v1[0] * v2[2]; 
		float c = v1[0] * v2[1] - v1[1] * v2[0];
		float d = -1 *(a * x1 + b * y1 + c * z1);

		for(int i = 0; i < cloud->points.size(); i++) {

			//if inliers.count() == 0 it means that we already have this index in our inliers so we don't have to take it into account a second time, so we pass
			if(inliers.count(i)>0) {
				continue; //The continue statement breaks one iteration (in the loop), if a specified condition occurs, and continues with the next iteration in the loop.
			}

			PointT point = cloud->points[i];

			float distance = fabs(a*point.x + b*point.y + c*point.z + d)/sqrt(a*a + b*b + c*c);

			//std::cout << "Distance to plane  :" << distance << std::endl;

			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceToPlane) {
				inliers.insert(i);
			}
		}


		if(inliers.size()>inliersResult.size()) {
			inliersResult = inliers;
		}

	}

    indices->indices.insert(indices->indices.end(),inliersResult.begin(), inliersResult.end());
    if(indices->indices.size() == 0){
        std::cout << "Could not estimate a planar model for the dataset." << std::endl;
    } 

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Custom Ransac Plane took " << elapsedTime.count() << " milliseconds" << std::endl;

   
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(indices,cloud);
  
  return segResult;
}




template<typename KdTreeT>
void clusterHelper(int index, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTreeT* tree, float distanceTol) {
	
	processed[index] = true;

	cluster.push_back(index);
	std::vector<int> nearby_points = tree->search(points[index], distanceTol);

	for(int id : nearby_points) {
		if(processed[id]) {
			continue;
		}

		clusterHelper(id, points, cluster, processed, tree, distanceTol);
	}
}

template<typename KdTreeT>
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTreeT* tree, float distanceTol)
{


	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false); // List of processed point indices

	for(int i = 0; i < points.size(); i++) {
		if(processed[i]) {
			continue;
		}

		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);

	}
 
	return clusters;
}

/* Custom Euclidean clustering */
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    KdTree3D* tree = new KdTree3D;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<float>> points;

    for (int i=0; i<cloud->points.size(); i++) {
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, cloud->points[i].intensity};
        points.push_back(point);
    	tree->insert(point,i); 
    }

    auto startTime = std::chrono::steady_clock::now();
  	
  	std::vector<std::vector<int>> clustersIndices = euclideanCluster(points, tree, clusterTolerance);
    for(std::vector<int> clusterIndices : clustersIndices)
  	{
        if(clusterIndices.size() < minSize || clusterIndices.size() > maxSize)
        {
            continue; 
        }

  		typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
  		for(int indice: clusterIndices) {
            cluster->points.push_back(cloud->points[indice]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


//Attempt to use PCA for generating optimized oriented bounding box following this link :
//http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
//We want to rotate around the Z axis but to still be oriented flat with the X and Y axis
template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster) {

    //Possible solution for OBB only around Z axis => project all the points on x and y, find PCA from this temporary pointcloud
    //and then apply the transform on the original pointcloud
    //I guess another solution would be to compute evrything omitting the Z axis
    typename pcl::PointCloud<PointT>::Ptr tmpXYcloud(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cluster, *tmpXYcloud);
    for(int i = 0; i < tmpXYcloud->points.size(); i++) {
        tmpXYcloud->points[i].z = 0;
    }

    BoxQ boxq;

    //From my understanding, we first start by finding a reference frame which leads the direction
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*tmpXYcloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*tmpXYcloud, pcaCentroid, covariance);
    //Compute Eigen values and vector
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    //This is our new reference frame
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));


    //Then we move all the points of our point cloud to that reference frame
    //I think we use an homogeneous transformation matrix
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity()); // We initalize the matrix as an identity matrix => null transform
    //block() allows us to manipulate parts of the matrix
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    // Apparently, the transformation given by the rotation matrix and the centroids must be inverted
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    //Final transform
    boxq.bboxQuaternion = eigenVectorsPCA;
    boxq.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    boxq.cube_height = maxPoint.z - minPoint.z;
    boxq.cube_length = maxPoint.x - minPoint.x;
    boxq.cube_width = maxPoint.y - minPoint.y;

    return boxq;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}