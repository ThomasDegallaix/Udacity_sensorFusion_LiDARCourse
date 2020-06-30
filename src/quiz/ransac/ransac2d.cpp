/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}



std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	
	// For max iterations 
	while(maxIterations--) {
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers; // unordered set because contains only unique elements -> we avoid to pick the same inlier twice in a row bcs of rand (same index will not be added)
		while(inliers.size() < 2) {
			inliers.insert(rand()%(cloud->points.size()));
		}

		float x1, y1, x2, y2;

		auto it = inliers.begin();
		x1 = cloud->points[*it].x;
		y1 = cloud->points[*it].y;
		it++;
		x2 = cloud->points[*it].x;
		y2 = cloud->points[*it].y;

		float a = (y1-y2);
		float b = (x2-x1);
		float c = (x1*y2-x2*y1);

		for(int i = 0; i < cloud->points.size(); i++) {

			//if inliers.count() == 0 it means that we already have this index in our inliers so we don't have to take it into account a second time, so we pass
			if(inliers.count(i)>0) {
				continue; //The continue statement breaks one iteration (in the loop), if a specified condition occurs, and continues with the next iteration in the loop.
			}

			pcl::PointXYZ point = cloud->points[i];
			float x3 = point.x;
			float y3 = point.y;

			float distanceToLine = fabs(a*x3 + b*y3 + c)/sqrt(a*a + b*b);

			// If distance is smaller than threshold count it as inlier
			if(distanceToLine <= distanceTol) {
				inliers.insert(i);
			}

		}

		if(inliers.size()>inliersResult.size()) {
			inliersResult = inliers;
		}
		
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceToPlane) {

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

			pcl::PointXYZ point = cloud->points[i];

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

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Custom Ransac Plane took " << elapsedTime.count() << " milliseconds" << std::endl;


	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
