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

typedef double (*featureCalculation) (const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf);
const int numberOfFeatures = 8; // TODO: can just use vectors and their lenghts

/*
IMPORTANT:
In all publications eigenvalues are listed in DECREASING order.
Eigenvalues in PCL and Eigen are listed in INCREASING order.
Basically: 
lambda1 = getEvals()[2]
lambda2 = getEvals()[1]
lambda3 = getEvals()[0]
*/

inline double linearity(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return (leaf->getEvals()[2] - leaf->getEvals()[1])/ leaf->getEvals()[2];
}

inline double planarity(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return (leaf->getEvals()[1] - leaf->getEvals()[0]) / leaf->getEvals()[2];
}

inline double sphericity(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return (leaf->getEvals()[0]) / leaf->getEvals()[2];
}

inline double omnivariance(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return pow(leaf->getEvals()[2] * leaf->getEvals()[1] * leaf->getEvals()[0], 1/3);
}

inline double anisotropy(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return (leaf->getEvals()[2] - leaf->getEvals()[0]) / leaf->getEvals()[2];
}

inline double eigenentropy(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return -1 * (leaf->getEvals()[2] * log(leaf->getEvals()[2]) + leaf->getEvals()[1] * log(leaf->getEvals()[1]) + leaf->getEvals()[0] * log(leaf->getEvals()[0]));
}

inline double sumOfEigenvalues(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return leaf->getEvals()[2] + leaf->getEvals()[1] + leaf->getEvals()[0];
}

inline double localSurfVar(const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf * leaf)
{
	return leaf->getEvals()[0] / (leaf->getEvals()[2] + leaf->getEvals()[1] + leaf->getEvals()[0]);
}

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		pcl::console::print_error("Too few arguments. Please provide location of the input point cloud.", argv[0]);
		return (1);
	}

	clock_t t1, t2;
	t1 = clock();

	// Create cloud object to store input cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	pcl::console::print_highlight("Loading point cloud...\n");
	if (reader.read(argv[1], *cloud))
	{
		pcl::console::print_error("Error loading cloud file!\n");
		return (1);
	}
	pcl::console::print_highlight("Point cloud loaded successfully. \n");

	pcl::console::print_highlight("PointCloud size: "); std:cerr << cloud->width * cloud->height << " points." << endl;

	// Voxel segmentation
	pcl::VoxelGridCovariance<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.04f, 0.04f, 0.04f);
	sor.setMinPointPerVoxel(3);
	sor.filter();
	pcl::console::print_highlight("Generated leaves: "); std::cerr << sor.getLeaves().size() << endl;
	pcl::console::print_highlight("Generated voxels: "); std::cerr << sor.getCentroids()->width * sor.getCentroids()->height << endl;
	/*
	When we generate a voxel grid in PCL, "leaf" is every voxel-space made by the grid.
	Using getLeaves() will get us all voxels, including non-filtered ones with less than 3 points inside.
	getCentroids() gives us a point-cloud of centroids created by the filter.
	*/

	//initialize all ofstreams
	ofstream linearityFile, planarityFile, sphericityFile, omnivarianceFile, anisotropyFile, eigenentropyFile, sumOfEigenvaluesFile, localSurfVarFile;
	ofstream *textFiles[numberOfFeatures] = { &linearityFile, &planarityFile, &sphericityFile, &omnivarianceFile, &anisotropyFile, &eigenentropyFile, &sumOfEigenvaluesFile, &localSurfVarFile };
	featureCalculation features[numberOfFeatures] = { linearity, planarity, sphericity, omnivariance, anisotropy, eigenentropy, sumOfEigenvalues, localSurfVar };
	string fileSuffix[numberOfFeatures] = { "linearity", "planarity", "sphericity", "omnivariance", "anisotropy", "eigenentropy", "sumOfEigenvalues", "localSurfVar" };

	vector<const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf *> calculatedVoxels;
	vector<double> calculatedFeatures[numberOfFeatures];

	//open all ofstreams and create text files
	for (int i = 0; i < numberOfFeatures; i++)
	{
		textFiles[i]->open(string(argv[1]) + "_" + fileSuffix[i] + ".txt");
	}

	vector<const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf *>::iterator helper;
	vector<pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf *>::iterator::difference_type position;
	double feature;

	//calculate features for every voxel with 3 or more points inside
	pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin();

	for(it; it < cloud->end(); it++)
	{
		if (sor.getLeaf(*it) && sor.getLeaf(*it)->getPointCount() > 2)
		{
			if (find(calculatedVoxels.begin(), calculatedVoxels.end(), sor.getLeaf(*it)) == calculatedVoxels.end())
			{
				calculatedVoxels.push_back(sor.getLeaf(*it));

				for (int i = 0; i < numberOfFeatures; i++)
				{
					feature = features[i](sor.getLeaf(*it));
					calculatedFeatures[i].push_back(feature);
				}
			}
		}
	}

	//save points and their features to text files
	pcl::PointCloud<pcl::PointXYZ>::iterator pit = cloud->begin();
	for (pit; pit != cloud->end(); pit++)
	{
		if (sor.getLeaf(*pit) && sor.getLeaf(*pit)->getPointCount() > 2)
		{
			helper = find(calculatedVoxels.begin(), calculatedVoxels.end(), sor.getLeaf(*pit));
			position = distance(calculatedVoxels.begin(), helper);
			for (int i = 0; i < numberOfFeatures; i++)
			{
				*textFiles[i] << pit->x << ", " << pit->y << ", " << pit->z << ", " << calculatedFeatures[i].at(position) << endl;
			}
			calculatedVoxels.push_back(sor.getLeaf(*pit));
		}
	}


	//close all ofstreams
	for (int i = 0; i < numberOfFeatures; i++)
	{
		textFiles[i]->close();
	}

	t2 = clock();
	double diff((double)t2 - (double)t1);
	pcl::console::print_highlight("Execution time: "); std::cerr << diff / 1000 << "s." << endl;
	system("pause");
	return (0);
}