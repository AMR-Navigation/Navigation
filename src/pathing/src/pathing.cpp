#include "../include/pathing.h"
#include <pluginlib/class_list_macros.h>

#include <iostream>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::DynamicLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

coord goodtobad(coord c) {
	coord badc = coord();
	badc.x = c.y;
	badc.y = -1*c.x;
	return badc;
}

coord goodtobad(float x, float y) {
	coord badc = coord();
	badc.x = y;
	badc.y = -1*x;
	return badc;
}

DynamicLayer::DynamicLayer() {}

void DynamicLayer::onInitialize()
{
	std::cout << "Init." << std::endl;
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&DynamicLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  this->sub = nh.subscribe("/coordinates_topic", 1000, &DynamicLayer::callback, this);
  ROS_INFO("Subscibed to coords topic as ");
  std::cout << name_ << std::endl;
}


void DynamicLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void DynamicLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  mark_x_ = robot_x + cos(robot_yaw);
  mark_y_ = robot_y + sin(robot_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void DynamicLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  
  for (int i=0;i<idobjs.size();i++) 
  {
	coord badcoord = goodtobad(idobjs[i].x,idobjs[i].y);
	unsigned int mx;
	unsigned int my;
	if(master_grid.worldToMap(badcoord.x, badcoord.y, mx, my)){
		setCircleCost(master_grid, mx, my, 20);
  	}
  }
}

void DynamicLayer::setCircleCost(costmap_2d::Costmap2D& costmap, int center_x, int center_y, int radius)
{
	ROS_INFO("Setting circle");
    // Iterate over the cells in the circle's bounding box
    for (int x = center_x - radius; x <= center_x + radius; ++x) {
        for (int y = center_y - radius; y <= center_y + radius; ++y) {
            // Calculate the distance from the center to the current cell
            int dx = x - center_x;
            int dy = y - center_y;
            double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

            // Check if the cell (x, y) is within the circle's radius
            if (distance <= radius) {
				std::cout << "Setting " << x << ' ' << y << " to " << (int) ((1-distance/(float)radius)*((float)LETHAL_OBSTACLE)) << " while distance is " << distance << " and r is " << radius << std::endl;
                // Set the cost value for the cell
                costmap.setCost(x, y, (int) ((1-distance/(float)radius)*((float)LETHAL_OBSTACLE)));
            }
        }
    }
}

void DynamicLayer::callback(const messages::coordinates::ConstPtr& msg)
{
	ROS_INFO("Gooot coords");
	while (idobjs.size()>0) idobjs.pop_back();
	for (int i=0;i<msg->matched_objects.size();i++)
	{
		coord c = coord();
		c.x = msg->matched_objects[i].x;
		c.y = msg->matched_objects[i].y;
		idobjs.push_back(c);
	}
	for (int i=0;i<msg->unmatched_objects.size();i++)
	{
		coord c = coord();
		c.x = msg->unmatched_objects[i].x;
		c.y = msg->unmatched_objects[i].y;
		idobjs.push_back(c);
	}
}

} // end namespace