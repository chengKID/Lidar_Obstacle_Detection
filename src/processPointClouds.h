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

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    // TODO: Final-Project
    // RANSAC
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RansacSegment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
    // KD-Tree
    struct Node
    {
	    PointT point;
	    int id;
	    Node* left;
	    Node* right;

	    Node(PointT arr, int setId)
	    :	point(arr), id(setId), left(NULL), right(NULL)
	    {}
    };
    struct KdTree
    {
        Node *root;

        KdTree()
        : root(NULL)
        {}

        void insertHelper(Node*& node, PointT point, int depth, int id)
        {
            // TODO: for 3D data
            if (node == NULL)
                node = new Node(point, id);
            else
            {
                // TODO: maybe use four values intensity
                if (node->point.data[depth%3] >= point.data[depth%3])
                    insertHelper(node->left, point, depth+1, id);
                else
                    insertHelper(node->right, point, depth+1, id);
            }
        }
	    void insert(PointT point, int id)
	    { 
		    insertHelper(root, point, 0, id);
	    }

	    void searchHelper(Node* node_, PointT target_, std::vector<int>* ids_, int depth, float distanceTol_)
	    {
		    // TODO: for 3D data
		    if (node_ == NULL)
			    return;

		    uint cd = depth % 3;
		    float box_x_min = target_.data[0] - distanceTol_;
		    float box_x_max = target_.data[0] + distanceTol_;
		    float box_y_min = target_.data[1] - distanceTol_;
		    float box_y_max = target_.data[1] + distanceTol_;
		    float box_z_min = target_.data[2] - distanceTol_;
		    float box_z_max = target_.data[2] + distanceTol_;
		    if (node_->point.data[0]>box_x_min && node_->point.data[0]<box_x_max && 
		    	node_->point.data[1]>box_y_min && node_->point.data[1]<box_y_max && 
			    node_->point.data[2]>box_z_min && node_->point.data[2]<box_z_max)
		    {
			    float distance = std::sqrt((node_->point.data[0]-target_.data[0])*(node_->point.data[0]-target_.data[0])+
									       (node_->point.data[1]-target_.data[1])*(node_->point.data[1]-target_.data[1])+
									       (node_->point.data[2]-target_.data[2])*(node_->point.data[2]-target_.data[2]));
			    if (distance <= distanceTol_)
				    ids_->push_back(node_->id);
		    }

		    if ((target_.data[cd]-distanceTol_) < node_->point.data[cd])
			    searchHelper(node_->left, target_, ids_, depth+1, distanceTol_);
		    if ((target_.data[cd]+distanceTol_) > node_->point.data[cd])
			    searchHelper(node_->right, target_, ids_, depth+1, distanceTol_);
	    }

	    // return a list of point ids in the tree that are within distance of target
	    std::vector<int> search(PointT target, float distanceTol)
	    {
		    std::vector<int> ids;
		    searchHelper(root, target, &ids, 0, distanceTol);
		    return ids;
	    }
    };
    // TODO: Final-Project
    // Euclidean Clustring
    void clusterHelper(KdTree* tree, const typename std::vector<PointT, Eigen::aligned_allocator<PointT>>* points, std::vector<int>& cluster, float distanceTol, std::vector<bool> &points_mask, int id);
    std::vector<std::vector<int>> euclideanCluster(const typename std::vector<PointT, Eigen::aligned_allocator<PointT>>& points, KdTree* tree, float distanceTol);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> KDTreeClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

};
#endif /* PROCESSPOINTCLOUDS_H_ */