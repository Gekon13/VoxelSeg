#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <iterator>
#include <unordered_map>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/point_cloud.h>

using namespace std;
using namespace Eigen;

typedef float (*featureCalculation) (const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf);

inline float linearity(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return (leaf->getEvals()[0] - leaf->getEvals()[1])/ leaf->getEvals()[0];
}

inline float planarity(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return (leaf->getEvals()[1] - leaf->getEvals()[2]) / leaf->getEvals()[0];
}

inline float sphericity(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return (leaf->getEvals()[2]) / leaf->getEvals()[0];
}

inline float omnivariance(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return pow(leaf->getEvals()[0] * leaf->getEvals()[1] * leaf->getEvals()[2], 3);
}

inline float anisotropy(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return (leaf->getEvals()[0] - leaf->getEvals()[2]) / leaf->getEvals()[0];
}

inline float eigenentropy(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return -1 * (leaf->getEvals()[0] * log(leaf->getEvals()[0]) + leaf->getEvals()[1] * log(leaf->getEvals()[1]) + leaf->getEvals()[2] * log(leaf->getEvals()[2]));
}

inline float sumOfEigenvalues(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return leaf->getEvals()[0] + leaf->getEvals()[1] + leaf->getEvals()[2];
}

int main(int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

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

	pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin();
	ofstream linearityFile, planarityFile, sphericityFile, omnivarianceFile, anisotropyFile, eigenentropyFile, sumOfEigenvaluesFile;
	ofstream *textFiles[] = { &linearityFile, &planarityFile, &sphericityFile, &omnivarianceFile, &anisotropyFile, &eigenentropyFile, &sumOfEigenvaluesFile };
	featureCalculation features[] = { linearity, planarity, sphericity, omnivariance, anisotropy, eigenentropy, sumOfEigenvalues };
	string fileSuffix[] = { "linearity", "planarity", "sphericity", "omnivariance", "anisotropy", "eigenentropy", "sumOfEigenvalues" };

	//open all ofstreams and create text files
	for (int i = 0; i < 7; i++)
	{
		textFiles[i]->open(string(argv[1]) + "_" + fileSuffix[i] + ".txt");
	}

	/*
	Below is naive way of calculating features. We calculate each feature for each point.
	*/
	for (it; it != cloud->end(); it++)
	{
		if (sor.getLeaf(*it)->getPointCount() > 2)
		{
			for (int i = 0; i < 7; i++)
			{
				*textFiles[i] << it->x << ", " << it->y << ", " << it->z << ", " << features[i](sor.getLeaf(*it)) << endl;
			}
		}
	}

	//close all ofstreams
	for (int i = 0; i < 7; i++)
	{
		textFiles[i]->close();
	}

	return (0);
}