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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	pcl::console::print_highlight("Loading point cloud...\n");
	if (reader.read(argv[1], *cloud))
	{
		pcl::console::print_error("Error loading cloud file!\n");
		return (1);
	}

	std::cerr << "PointCloud size: " << cloud->width * cloud->height << " data points." << endl;

	pcl::VoxelGridCovariance<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.04f, 0.04f, 0.04f);
	sor.filter();
	std::cerr << "VoxelGrid size: " << sor.getLeaves().size() << " leafs." << endl;

	map<size_t, pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf> allVoxels = sor.getLeaves();
	map<size_t, pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf>::iterator it = allVoxels.begin();
	float helper;

	for (it; it != allVoxels.end(); it++)
	{
		
		//linearity
		helper = (it->second.getEvals()[0] - it->second.getEvals()[1]) / it->second.getEvals()[0];

		cout << it->second.getEvals() << endl;
		cout << "Linearity: " << helper << endl;
		cout << "" << endl;

		/*
		ofstream outfile;
		outfile.open("text.txt");
		for (int i = 0; i < 3; i++)
		{
			outfile << it->second.getEvals()[i] << ", " << endl;
		}
		*/
		
		

	}
	return (0);
}