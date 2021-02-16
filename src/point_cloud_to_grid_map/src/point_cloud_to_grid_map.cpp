#include "point_cloud_to_grid_map.hpp"


pointCloud2GridMap::pointCloud2GridMap()
{
	m_nh.getParam("/point_cloud_to_grid_map/m_sub_point_cloud", m_sub_point_cloud);
    m_nh.getParam("/point_cloud_to_grid_map/map_xMax", map_xMax);
    m_nh.getParam("/point_cloud_to_grid_map/map_xMin", map_xMin);
    m_nh.getParam("/point_cloud_to_grid_map/map_yMax", map_yMax);
    m_nh.getParam("/point_cloud_to_grid_map/map_yMin", map_yMin);
    m_nh.getParam("/point_cloud_to_grid_map/map_resolution", map_resolution);
    m_nh.getParam("/point_cloud_to_grid_map/safe_radius", safe_radius);

	m_sub_filtered_point_cloud = m_nh.subscribe(m_sub_point_cloud, 10, &pointCloud2GridMap::VelodyneCallback, this);
	m_pub_occupancy_grid_map = m_nh.advertise<nav_msgs::OccupancyGrid>( "occupancy_map", 1, true );
}

pointCloud2GridMap::~pointCloud2GridMap()
{

}

void pointCloud2GridMap::VelodyneCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr in_no_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::fromROSMsg(*input, *in_no_ground_cloud);

	nav_msgs::OccupancyGrid occupancyGrid;

	occupancyGrid.header.frame_id= input->header.frame_id;
	occupancyGrid.info.resolution= map_resolution;
	occupancyGrid.info.height= int( (map_yMax - map_yMin)/map_resolution );
	occupancyGrid.info.width= int( (map_xMax - map_xMin)/map_resolution );

	occupancyGrid.info.origin.position.x= map_xMin;
	occupancyGrid.info.origin.position.y= map_yMin;
	occupancyGrid.info.origin.position.z= 0.0f;

	tf2::Quaternion q;
	q.setRPY( 0.0, 0.0, 0.0 );
	occupancyGrid.info.origin.orientation.x= q.x();
	occupancyGrid.info.origin.orientation.y= q.y();
	occupancyGrid.info.origin.orientation.z= q.z();
	occupancyGrid.info.origin.orientation.w= q.w();
	occupancyGrid.data.resize( occupancyGrid.info.height * occupancyGrid.info.width );

	for (const auto &point:in_no_ground_cloud->points)
	{
		geometry_msgs::Point sensor_point;
		sensor_point.x = point.x;
		sensor_point.y = point.y;
		sensor_point.z = point.z;

		int xmin_idx= 0;
		int ymin_idx= 0;
		int xmax_idx= occupancyGrid.info.width;
		int ymax_idx= occupancyGrid.info.height;
		int xmid_idx= int( occupancyGrid.info.width/2 );
		int ymid_idx= int( occupancyGrid.info.height/2 );
		int left_idx= (sensor_point.x - safe_radius)/map_resolution + xmid_idx;
		int right_idx= (sensor_point.x + safe_radius)/map_resolution + xmid_idx;
		int top_idx= (sensor_point.y - safe_radius)/map_resolution + ymid_idx;
		int bottom_idx= (sensor_point.y + safe_radius)/map_resolution + ymid_idx;

		// lock the map message
		// boost::mutex::scoped_lock lock( map_mutex_ );
		// loop over the grids occupied by the robot
		for( int y=top_idx; y<bottom_idx; y++ )
		{
			for( int x=left_idx; x<right_idx; x++ )
			{
				// boundary check
				if( x < xmin_idx || x >= xmax_idx || y < ymin_idx || y >= ymax_idx )
					continue;
				occupancyGrid.data[ y*occupancyGrid.info.width + x ]= GRID_OCCUPIED;
			}
		}
		// set stamp
		occupancyGrid.header.stamp= ros::Time::now();
	}
	m_pub_occupancy_grid_map.publish(occupancyGrid);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "point_cloud_to_grid_map");
	ros::NodeHandle m_nh;

	pointCloud2GridMap pointCloud2GridMap;

	ros::Rate loop_rate(10);

    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}