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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		insertHelper(&root,point,0,id);

	}

	void insertHelper(Node** node, std::vector<float> point, uint depth, int id) {

		//Case where current tree node is empty
		if(*node == NULL) {
			*node = new Node(point, id);
		}
		//If not empty, we can insert new elements to the left or the right of the last node of the tree
		else {
			//Find the current depth to know if we insert regarding the x (0) or y (1) axis
			uint currentDepth = depth % 2;

			if(point[currentDepth] < ((*node)->point[currentDepth])) {
				insertHelper(&((*node)->left), point, depth+1, id);
			}
			else {
				insertHelper(&((*node)->right), point, depth+1, id);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0 , distanceTol, ids);

		return ids;
	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids) {

		if(node != NULL) {

			//Check if the current node is inside the bounding box of our target
			if(node->point[0] >= (target[0] - distanceTol) && node->point[0]<= (target[0] + distanceTol)
				&& node->point[1] >= (target[1] - distanceTol) && node->point[1]<= (target[1] + distanceTol)) {
					
				float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]));
				if(distance <= distanceTol) {
					ids.push_back(node->id);
				}
			}

			//check if we need to process the right or the left branch of the current node
			if((target[depth%2] - distanceTol) < node->point[depth%2]) {
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
			if((target[depth%2] + distanceTol) > node->point[depth%2]) {
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}

	}
	

};

struct KdTree3D
{
	Node* root;

	KdTree3D()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root,point,0,id);
	}

	void insertHelper(Node** node, std::vector<float> point, uint depth, int id) {

		//Case where current tree node is empty
		if(*node == NULL) {
			*node = new Node(point, id);
		}
		//If not empty, we can insert new elements to the left or the right of the last node of the tree
		else {
			//Find the current depth to know if we insert regarding the x (0) or y (1) axis
			uint currentDepth = depth % 3;

			if(point[currentDepth] < ((*node)->point[currentDepth])) {
				insertHelper(&((*node)->left), point, depth+1, id);
			}
			else {
				insertHelper(&((*node)->right), point, depth+1, id);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0 , distanceTol, ids);

		return ids;
	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids) {

		if(node != NULL) {

			//Check if the current node is inside the bounding box of our target
			if(node->point[0] >= (target[0] - distanceTol) && node->point[0]<= (target[0] + distanceTol)
				&& node->point[1] >= (target[1] - distanceTol) && node->point[1]<= (target[1] + distanceTol)
				&& node->point[2] >= (target[2] - distanceTol) && node->point[2]<= (target[2] + distanceTol)) {
					
				float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]) + (node->point[2] - target[2]) * (node->point[2] - target[2]));
				if(distance <= distanceTol) {
					ids.push_back(node->id);
				}
			}

			//check if we need to process the right or the left branch of the current node
			if((target[depth%3] - distanceTol) < node->point[depth%3]) {
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
			if((target[depth%3] + distanceTol) > node->point[depth%3]) {
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}

	}
	

};





