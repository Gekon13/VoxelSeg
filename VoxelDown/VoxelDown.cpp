#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <string>
#include <iterator>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/point_cloud.h>
#include <time.h>

using namespace std;
using namespace Eigen;

typedef float (*featureCalculation) (const pcl::VoxelGridCovariance<pcl::PointXYZRGBA>::Leaf * leaf);

inline float linearity(const pcl::VoxelGridCovariance<pcl::PointXYZRGBA>::Leaf * leaf)
{
	return (leaf->getEvals()[2] - leaf->getEvals()[1])/ leaf->getEvals()[2];
}

inline float planarity(const pcl::VoxelGridCovariance<pcl::PointXYZRGBA>::Leaf * leaf)
{
	return (leaf->getEvals()[1] - leaf->getEvals()[0]) / leaf->getEvals()[2];
}

inline float sphericity(const pcl::VoxelGridCovariance<pcl::PointXYZRGBA>::Leaf * leaf)
{
	return (leaf->getEvals()[0]) / leaf->getEvals()[2];
}

inline float omnivariance(const pcl::VoxelGridCovariance<pcl::PointXYZRGBA>::Leaf * leaf)
{
	return pow(leaf->getEvals()[2] * leaf->getEvals()[1] * leaf->getEvals()[0], 1/3);
}

inline float anisotropy(const pcl::VoxelGridCovariance<pcl::PointXYZRGBA>::Leaf * leaf)
{
	return (leaf->getEvals()[2] - leaf->getEvals()[0]) / leaf->getEvals()[2];
}

inline float eigenentropy(const pcl::VoxelGridCovariance<pcl::PointXYZRGBA>::Leaf * leaf)
{
	return -1 * (leaf->getEvals()[2] * log(leaf->getEvals()[2]) + leaf->getEvals()[1] * log(leaf->getEvals()[1]) + leaf->getEvals()[0] * log(leaf->getEvals()[0]));
}

inline float sumOfEigenvalues(const pcl::VoxelGridCovariance<pcl::PointXYZRGBA>::Leaf * leaf)
{
	return leaf->getEvals()[2] + leaf->getEvals()[1] + leaf->getEvals()[0];
}

vector<string> makeColor(vector<float> featuresVector)
{
	float minimum = *min_element(featuresVector.begin(), featuresVector.end());
	float maximum = *max_element(featuresVector.begin(), featuresVector.end());

	float range = maximum - minimum;
	float min = minimum;
	vector<string> colors;
	int red, grn, blu;

	for (int i = 0; i < int(featuresVector.size()); i++)
	{
		red = 255 * ((featuresVector[i] - minimum) / range);
		blu = 255 - (255 * (featuresVector[i] - minimum) / range);
		grn = 0;
		colors.push_back(to_string(red) + ", " + to_string(grn) + ", " + to_string(blu));
	}

	return colors;
}

int main(int argc, char** argv)
{
	/*if (argc < 2)
	{
		pcl::console::print_error("Syntax is: %s <pcd-file> \n "
			"-all Calculates all features. \n"
			"-pla Calculates planarity. \n"
			"-sph Calculates sphericity. \n"
			"-omn Calculates omnivariance. \n"
			"-ani Calculates anisotropy. \n"
			"-eig Calculates eigenentropy. \n"
			"-sum Calculates sum of Eigenvalues. \n", argv[0]);
		return (1);
	}*/

	clock_t t1, t2;

	t1 = clock();

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	pcl::console::print_highlight("Loading point cloud...\n");
	if (reader.read(argv[1], *cloud))
	{
		pcl::console::print_error("Error loading cloud file!\n");
		return (1);
	}
	pcl::console::print_highlight("Point cloud loaded. \n");

	std::cerr << "PointCloud size: " << cloud->width * cloud->height << " points." << endl;

	pcl::VoxelGridCovariance<pcl::PointXYZRGBA> sor;
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

	//initialize all ofstreams
	ofstream linearityFile, planarityFile, sphericityFile, omnivarianceFile, anisotropyFile, eigenentropyFile, sumOfEigenvaluesFile;
	ofstream *textFiles[7] = { &linearityFile, &planarityFile, &sphericityFile, &omnivarianceFile, &anisotropyFile, &eigenentropyFile, &sumOfEigenvaluesFile };
	featureCalculation features[7] = { linearity, planarity, sphericity, omnivariance, anisotropy, eigenentropy, sumOfEigenvalues };
	string fileSuffix[7] = { "linearity", "planarity", "sphericity", "omnivariance", "anisotropy", "eigenentropy", "sumOfEigenvalues" };

	vector<const pcl::VoxelGridCovariance<pcl::PointXYZRGBA>::Leaf *> calculatedVoxels;
	vector<float> calculatedFeatures[7];

	//open all ofstreams and create text files
	for (int i = 0; i < 7; i++)
	{
		textFiles[i]->open(string(argv[1]) + "_" + fileSuffix[i] + ".txt");
	}

	vector<const pcl::VoxelGridCovariance<pcl::PointXYZRGBA>::Leaf *>::iterator helper;
	vector<pcl::VoxelGridCovariance<pcl::PointXYZRGBA>::Leaf *>::iterator::difference_type position;
	float feature;

	//calculate features for every voxel with 3 or more points inside
	pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloud->begin();

	for(it; it < cloud->end(); it++)
	{
		std::cerr << "Generated voxels: " << it->x << " " << it->y << " " << it->z << endl;
		if (sor.getLeaf(*it)->getPointCount() > 2)
		{
			if (find(calculatedVoxels.begin(), calculatedVoxels.end(), sor.getLeaf(*it)) == calculatedVoxels.end())
			{
				calculatedVoxels.push_back(sor.getLeaf(*it));

				for (int i = 0; i < 7; i++)
				{
					feature = features[i](sor.getLeaf(*it));
					calculatedFeatures[i].push_back(feature);
				}
			}
		}
	}

	//Transform calculated features into colours
	/*
	vector<string> calculatedColors[7];
	for (int i = 0; i < 7; i++)
	{
		calculatedColors[i] = makeColor(calculatedFeatures[i]);
	}
	*/

	//save points and their features to text files
	pcl::PointCloud<pcl::PointXYZRGBA>::iterator pit = cloud->begin();
	for (pit; pit != cloud->end(); pit++)
	{
		if (sor.getLeaf(*pit)->getPointCount() > 2)
		{
			helper = find(calculatedVoxels.begin(), calculatedVoxels.end(), sor.getLeaf(*pit));
			position = distance(calculatedVoxels.begin(), helper);
			for (int i = 0; i < 7; i++)
			{
				*textFiles[i] << pit->x << ", " << pit->y << ", " << pit->z << ", " << calculatedFeatures[i].at(position) << endl;
			}
			calculatedVoxels.push_back(sor.getLeaf(*pit));
		}
	}


	//close all ofstreams
	for (int i = 0; i < 7; i++)
	{
		textFiles[i]->close();
	}

	t2 = clock();
	float diff((float)t2 - (float)t1);
	cout << "Execution time: " << diff/1000 << "s." << endl;
	system("pause");
	return (0);
}