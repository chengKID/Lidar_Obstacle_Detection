// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // voxel grid filter
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new typename pcl::PointCloud<PointT>);
    typename pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloud_filtered);

    // crop region
    typename pcl::PointCloud<PointT>::Ptr cloud_region(new typename pcl::PointCloud<PointT>);
    typename pcl::CropBox<PointT> crop_box(true);
    crop_box.setInputCloud(cloud_filtered);
    crop_box.setMin(minPoint);
    crop_box.setMax(maxPoint);
    crop_box.filter(*cloud_region);

    // remove points around the roof of ego-car
    std::vector<int> root_indices;
    typename pcl::CropBox<PointT> crop_roof(true);
    crop_roof.setInputCloud(cloud_region);
    crop_roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    crop_roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    crop_roof.filter(root_indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int idx : root_indices)
        inliers->indices.push_back(idx);
    
    typename pcl::ExtractIndices<PointT> roof_extract;
    roof_extract.setInputCloud(cloud_region);
    roof_extract.setIndices(inliers);
    roof_extract.setNegative(true);
    roof_extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    for (int index : inliers->indices) {
        planeCloud->points.push_back(cloud->points[index]);
    }
    
    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // Creating the segmentation objecte
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largerst planar component from the cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cout<< "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


// TODO: Final-Project
// Segmentation using RANSAC
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacSegment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;

	// Return indicies of inliers from fitted line with most inliers
	auto startTime = std::chrono::steady_clock::now();
	while (maxIterations--)
	{
		std::unordered_set<int>* inliers = new std::unordered_set<int>();
		while (inliers->size() < 3)
        {
            int selected = random() % cloud->points.size();
            if (!inliers->count(selected))
                inliers->insert(selected);
        }

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto itr = inliers->begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float v1x = x2 - x1;
		float v1y = y2 - y1;
		float v1z = z2 - z1;

		float v2x = x3 - x1;
		float v2y = y3 - y1;
		float v2z = z3 - z1;

		float i = v1y*v2z - v1z*v2y;
		float j = v1z*v2x - v1x*v2z;
		float k = v1x*v2y - v1y*v2x;
		float D = -(i*x1 + j*y1 + k*z1);

		for (int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers->count(index))
				continue;
			
			float x4 = cloud->points[index].x;
			float y4 = cloud->points[index].y;
			float z4 = cloud->points[index].z;
			float d = fabs(i*x4 + j*y4 + k*z4 + D) / sqrt(i*i + j*j + k*k);

			if (d <= distanceTol)
				inliers->insert(index);
		}

		if (inliers->size() > inliersResult.size())
			inliersResult = *inliers;

        delete inliers;
	}
	
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout<< "RANSAC took " << elapsedTime.count() << " miliseconds!" << std::endl;
 	
	typename pcl::PointCloud<PointT>::Ptr  plane_cloud(new typename pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr obst_cloud(new typename pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			plane_cloud->points.push_back(point);
		else
			obst_cloud->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> seg_result(obst_cloud, plane_cloud);
    return seg_result;
}

// TODO: Final-Project
// Euclidean clustring
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(KdTree* tree, const typename std::vector<PointT, Eigen::aligned_allocator<PointT>>* points, std::vector<int>& cluster, float distanceTol, std::vector<bool> &points_mask, int id)
{
	points_mask[id] = true;
	cluster.push_back(id);
	std::vector<int> near_points = tree->search((*points)[id], distanceTol);

	for (int near_point : near_points)
	{
		if (!points_mask[near_point])
			clusterHelper(tree, points, cluster, distanceTol, points_mask, near_point);
	}
}

// TODO: Final-Project
// Euclidean clustring
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const typename std::vector<PointT, Eigen::aligned_allocator<PointT>>& points, KdTree* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> points_mask(points.size(), false);
	for (int i = 0; i < points.size(); i++)
	{
		if (!points_mask[i])
		{
			std::vector<int> cluster;
			clusterHelper(tree, &points, cluster, distanceTol, points_mask, i);
			clusters.push_back(cluster);
		}
	}
	return clusters;
}

// TODO: Final-Project
// Euclidean clustring
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::KDTreeClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree* tree = new KdTree;
    for (int i=0; i<cloud->points.size(); i++) 
    	tree->insert(cloud->points[i],i);

    std::vector<std::vector<int>> clusters_indices = euclideanCluster(cloud->points, tree, clusterTolerance);
    for (auto cluster_itr = clusters_indices.begin(); cluster_itr != clusters_indices.end(); cluster_itr++)
    {
        if (cluster_itr->size() < minSize || cluster_itr->size() > maxSize)
            continue;

        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new typename pcl::PointCloud<PointT>);
        for (auto pit = cluster_itr->begin(); pit != cluster_itr->end(); pit++)
            cloud_cluster->points.push_back(cloud->points[*pit]);
        
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout<< "pointCloud representing the cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        clusters.push_back(cloud_cluster);
    }    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator itr = cluster_indices.begin(); itr != cluster_indices.end(); itr++)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new typename pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = itr->indices.begin(); pit != itr->indices.end(); pit++)
            cloud_cluster->points.push_back(cloud->points[*pit]);
        
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout<< "pointCloud representing the cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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