#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/point_cloud.h>
#include <map>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int
main(int argc, char** argv)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	pcl::console::print_highlight("Loading point cloud...\n");
	if (reader.read(argv[1], *cloud))
	{
		pcl::console::print_error("Error loading cloud file!\n");
		return (1);
	}

	std::cerr << "PointCloud size: " << cloud->width * cloud->height << endl;

	pcl::VoxelGridCovariance<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.04f, 0.04f, 0.04f);
	sor.setMinPointPerVoxel(3);
	sor.filter();
	std::cerr << "Generated leaves: " << sor.getLeaves().size() << endl; 
	std::cerr << "Generated voxels: " << sor.getCentroids()->width * sor.getCentroids()->height << endl;
	/*
	When we generate a voxel grid in PCL, "leaf" is every voxel-space made by the grid. 
	Using getLeaves() will get us all voxels, including non-filtered ones with less than 3 points inside.
	getCentroids() gives us a point-cloud of centroids created by the filter.
	*/

	float linearity;
	pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin();
	ofstream outfile;
	outfile.open("text.txt");

	/*
	Below is naive way of calculating features. We calculate voxel's feature for each point, so it's slow.
	*/
	for (it; it != cloud->end(); it++)
	{
		if (sor.getLeaf(*it)->getPointCount() > 2)
		{
			linearity = (sor.getLeaf(*it)->getEvals()[0] - sor.getLeaf(*it)->getEvals()[1]) / sor.getLeaf(*it)->getEvals()[0];
			outfile << it->x << ", " << it->y << ", " << it->z << ", " << linearity << endl;
		}
	}

	outfile.close();

	return (0);
}