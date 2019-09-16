/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node* &node, std::vector<float> point, int depth, int id)
	{
		if (node == NULL)
		{
			node = new Node(point, id);
		}
		else if (depth%2 == 0)
		{
			if (node->point[0] >= point[0])
				insertHelper(node->left, point, depth+1, id);
			else
				insertHelper(node->right, point, depth+1, id);
		}
		else
		{
			if (node->point[1] >= point[1])
				insertHelper(node->left, point, depth+1, id);
			else
				insertHelper(node->right, point, depth+1, id);
		}

		/*// TODO: for 3D data
		if (node == NULL)
		{
			node = new Node(point, id);
		}
		else 
		{
			if (node->point[depth%3] >= point[depth%3])
				insertHelper(node->left, point, depth+1, id);
			else
				insertHelper(node->right, point, depth+1, id);			
		}*/
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, point, 0, id);

	}

	void searchHelper(Node* node_, std::vector<float> target_, std::vector<int>* ids_, int depth, float distanceTol_)
	{
		if (node_ == NULL)
			return;

		uint cd = depth % 2;
		float box_x_min = target_[0] - distanceTol_;
		float box_x_max = target_[0] + distanceTol_;
		float box_y_min = target_[1] - distanceTol_;
		float box_y_max = target_[1] + distanceTol_;
		if (node_->point[0]>box_x_min && node_->point[0]<box_x_max && node_->point[1]>box_y_min && node_->point[1]<box_y_max)
		{
			float distance = std::sqrt((node_->point[0]-target_[0])*(node_->point[0]-target_[0]) + (node_->point[1]-target_[1])*(node_->point[1]-target_[1]));
			if (distance <= distanceTol_)
				ids_->push_back(node_->id);
		}

		if ((target_[cd]-distanceTol_) < node_->point[cd])
			searchHelper(node_->left, target_, ids_, depth+1, distanceTol_);
		if ((target_[cd]+distanceTol_) > node_->point[cd])
			searchHelper(node_->right, target_, ids_, depth+1, distanceTol_);		

		/*// TODO: for 3D data
		if (node_ == NULL)
			return;

		uint cd = depth % 3;
		float box_x_min = target_[0] - distanceTol_;
		float box_x_max = target_[0] + distanceTol_;
		float box_y_min = target_[1] - distanceTol_;
		float box_y_max = target_[1] + distanceTol_;
		float box_z_min = target_[2] - distanceTol_;
		float box_z_max = target_[2] + distanceTol_;
		if (node_->point[0]>box_x_min && node_->point[0]<box_x_max && 
			node_->point[1]>box_y_min && node_->point[1]<box_y_max && 
			node_->point[2]>box_z_min && node_->point[2]<box_z_max)
		{
			float distance = std::sqrt((node_->point[0]-target_[0])*(node_->point[0]-target_[0])+
									   (node_->point[1]-target_[1])*(node_->point[1]-target_[1])+
									   (node_->point[2]-target_[2])*(node_->point[2]-target_[2]));
			if (distance <= distanceTol_)
				ids_->push_back(node_->id);
		}

		if ((target_[cd]-distanceTol_) < node_->point[cd])
			searchHelper(node_->left, target_, ids_, depth+1, distanceTol_);
		if ((target_[cd]+distanceTol_) > node_->point[cd])
			searchHelper(node_->right, target_, ids_, depth+1, distanceTol_);*/
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, &ids, 0, distanceTol);
		return ids;
	}
	

};




