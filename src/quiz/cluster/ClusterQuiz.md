# Implementing KD-Tree and Euclidean Clustering

A KD-Tree is a binary tree that splits points between alternating axes. By seperating space by splitting regions, it can make it much faster to do nearest neighbor search when using an algorithm like euclidean clustering. In this quiz we will be looking at a 2D example, so the the tree will be a 2D-Tree. 

### Improving the Tree Structure

Having a balanced tree that evenly splits regions improves the search time for finding points later. To improve the tree insert points that alternate between splitting the x region and the y region. To do this pick the median of sorted x and y points. For instance if you are inserting the first four points that we used above (-6.3,8.4),(-6.2,7),(-5.2,7.1),(-5.7,6.3) we would first insert (-5.2,7.1) since its the median for the x points, if there is an even number of elements the lower median is chosen. The next point to be insorted would be (-6.2,7) the median of the three points for y. Followed by (-5.7,6.3) the lower median between the two for x, and then finally (-6.3,8.4). This ordering will allow the tree to more evenly split the region space and improving searching later.

## Searching Nearby Points in the Tree

Once points are able to be inserted into the tree, the next step is being able to search for nearby points (points within a distance of distanceTol) inside the tree compared to a given pivot point. The kd-tree is able to split regions and allows certain regions to be completly ruled out, speeding up the process of finding nearby neighbors. The naive approach of finding nearby neighbors is to go through every single point in the tree and compare its distance with the pivots, selecting point indices that fall with in the distance tolerance. Instead with the kd-tree we can compare distance within a boxed square that is 2 x distanceTol for length, centered around the pivot point. If the current node point is within this box then we can directly calculate the distance and see if we add it to the list of `ids`. Then we see if our box crosses over the node division region and if it does compare that next node. We do this recursively, with the advantage being that if the box region is not inside some division region we completly skip that branch.


## Euclidean Clustering

Once the kd-tree method for searching for nearby points is implemented, its not difficult to implement a euclidean clustering method that groups indidual cluster indicies based on their proximity. Inside `cluster.cpp` there is a function called `euclideanCluster` which returns a vector of vector ints, this is the list of each cluster's indices. To perform the clustering, iterate through each point in the cloud and keep track of which points have been processed already. For each point add it to a cluster group then get a list of all the points in proximity to that point. For each point in proximity if it hasn't already been processed add it to the cluster group and repeat the process of calling proximity points. Once the recursion stops for the first cluster group, create a new cluster and move through the point list. Once all the points have been processed there will be a certain number of cluster groups found.

### Exercise

```
list of clusters 

Proximity(point,cluster)
	If point has not been processed
		mark point as processed
		add point to cluster
		nearby points = tree(point)
		Iterate through each nearby point
			Proximity(cluster)


Iterate through each point
	Create cluster
	Proximity(point,cluster)
	cluster add cluster

return clusters

```

When running the euclidean clustering function on the data here is the results

```
euclideanCluster(points, tree, 3.0);

```

There are three clusters found, using a distance tolerance of 3.0. Each cluster group is colored a differently, red, green, and blue.


