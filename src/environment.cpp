/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
{
    // ----------------------------------------------------
    // ------ Open 3D viewer and display City Block -------
    // ----------------------------------------------------
    // TODO: Final-Project
    bool render_origin = false;
    bool render_sparse = false;
    bool render_segment = false;
    bool render_clusters = true;
    bool render_boxes = true;

    if (render_origin)
        renderPointCloud(viewer, input_cloud, "inputCloud");

    // Filter the cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud = pointProcessorI.FilterCloud(input_cloud, 0.3, Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f (30, 8, 1, 1));
    if (render_sparse)
        renderPointCloud(viewer, filter_cloud, "filterCloud");

    // Segment the filtered cloud into road & obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = pointProcessorI.RansacSegment(filter_cloud, 100, 0.15);
    renderPointCloud(viewer, segment_cloud.second, "planeCloud", Color(0, 1, 0));
    if (render_segment)
        renderPointCloud(viewer, segment_cloud.first, "obstCloud", Color(1, 0, 0));

    // Cluster the obstacle cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters = pointProcessorI.KDTreeClustering(segment_cloud.first, 0.35, 10, 1000);
    int cluster_ID = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for (auto cluster : cloud_clusters)
    {
        // TODO: render cluster cloud
        if (render_clusters)
        {
            std::cout<< "cluster size: ";
            pointProcessorI.numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(cluster_ID), colors[cluster_ID%3]);
        }

        // TODO: render box around cluster
        if (render_boxes)
        {
            Box box = pointProcessorI.BoundingBox(cluster);
            renderBox(viewer, box, cluster_ID);           
        }
        cluster_ID++; 
    }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar_sensor = new Lidar(cars, 0.0);
    bool render_lidarray = false;
    bool render_pointcloud = false;
    bool render_clusters = true;
    bool render_boxes = true;

    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud = lidar_sensor->scan();
    if (render_lidarray)
    {
        renderRays(viewer, lidar_sensor->position, lidar_cloud);
        renderPointCloud(viewer, lidar_cloud, "lidar_cloud"); //, Color(255, 255, 255));
    }
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(lidar_cloud, 100, 0.2);
    if (render_pointcloud)
    {
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
    int cluster_ID = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloud_clusters)
    {
        // TODO: render cluster cloud
        if (render_clusters)
        {
            std::cout<< "cluster size: ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(cluster_ID), colors[cluster_ID]);
        }

        // TODO: render box around cluster
        if (render_boxes)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, cluster_ID);           
        }

        ++cluster_ID; 
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    // Load data
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto stream_iterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd & run obstacle detection function
        input_cloud = pointProcessorI.loadPcd((*stream_iterator).string());
        cityBlock(viewer, pointProcessorI, input_cloud);

        stream_iterator++;
        if (stream_iterator == stream.end())
            stream_iterator = stream.begin();

        viewer->spinOnce ();
    } 
}