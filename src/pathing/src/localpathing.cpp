#include "../include/localpathing.h"
#include <pluginlib/class_list_macros.h>

#include <iostream>

PLUGINLIB_EXPORT_CLASS(simple_local_layer_namespace::DynamicLocalLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

int LOCALDIM = 60;
float RES = .05;

namespace simple_local_layer_namespace
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

bool worldtolocal(unsigned int &rx, unsigned int &ry, float x, float y, float robotx, float roboty) {
	coord loc;
	loc.x = (x-robotx)/RES + LOCALDIM/2;
	loc.y = (y-roboty)/RES + LOCALDIM/2;
	if (loc.x < 0 or loc.x > LOCALDIM or loc.y <0 or loc.y > LOCALDIM) {
		std::cout << "Bad coords " << 0 << '|' << y << ' ' << loc.x << ' ' << loc.y << std::endl;
		return false;
	}
	rx = (int)loc.x;
	ry = (int)loc.y;
	return true;
}
	
DynamicLocalLayer::DynamicLocalLayer() {}

void DynamicLocalLayer::onInitialize()
{
	std::cout << "Local init." << std::endl;
	ros::NodeHandle nh("~/" + name_);
	current_ = true;
	default_value_ = NO_INFORMATION;
	matchSize();

	dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
	dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&DynamicLocalLayer::reconfigureCB, this, _1, _2);
	dsrv_->setCallback(cb);

	this->sub = nh.subscribe("/coordinates_topic", 1000, &DynamicLocalLayer::callback, this);
	ROS_INFO("Subscibed to coords topic as ");
	std::cout << name_ << std::endl;
}

void DynamicLocalLayer::matchSize()
{
	Costmap2D* master = layered_costmap_->getCostmap();
	resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
}


void DynamicLocalLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
	enabled_ = config.enabled;
}

void DynamicLocalLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
	if (!enabled_)
		return;

	// MY PROCESSING HERE:

	for (int i=0;i<idobjs.size();i++) 
	{
	coord badcoord = goodtobad(idobjs[i].x,idobjs[i].y);
	unsigned int mx;
	unsigned int my;
	std::cout << i << ": Drawing obj at " << badcoord.x << ' ' << badcoord.y << ' ';
	if(worldtolocal(mx, my, badcoord.x, badcoord.y, robot_x, robot_y)){
			std::cout << " with map coords " << mx << ' ' << my << std::endl;
			setCircleCost(mx, my, 20);
		}
	}

	// ------------------------------------------------------------------------
	double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
	*min_x = std::min(*min_x, mark_x);
	*min_y = std::min(*min_y, mark_y);
	*max_x = std::max(*max_x, mark_x);
	*max_y = std::max(*max_y, mark_y);

}

void DynamicLocalLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
	if (!enabled_)
		return;
	
	for (int j = min_j; j < max_j; j++)
	{
		for (int i = min_i; i < max_i; i++)
		{
			int index = getIndex(i, j);
			if (costmap_[index] == NO_INFORMATION)
				continue;
			master_grid.setCost(i, j, costmap_[index]); 
			if (costmap_[index]>2) costmap_[index]-=2;
			else costmap_[index] = FREE_SPACE;
		}
	}
}

void DynamicLocalLayer::setCircleCost(int center_x, int center_y, int radius)
{
	//ROS_INFO("Setting circle");
		// Iterate over the cells in the circle's bounding box
		for (int x = center_x - radius; x <= center_x + radius; ++x) {
				for (int y = center_y - radius; y <= center_y + radius; ++y) {
					if (x<0 or x>getSizeInCellsX() or y<0 or y>getSizeInCellsY()) continue;
						// Calculate the distance from the center to the current cell
						int dx = x - center_x;
						int dy = y - center_y;
						double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

						// Check if the cell (x, y) is within the circle's radius
						if (distance <= radius) {
								// std::cout << "Setting " << x << ' ' << y << " to " << (int) ((1-distance/(float)radius)*((float)LETHAL_OBSTACLE)) << " while distance is " << distance << " and r is " << radius << std::endl;
								// Set the cost value for the cell
								setCost(x, y, (int) ((1-distance/(float)radius)*((float)LETHAL_OBSTACLE)));
						}
				}
		}
}

void DynamicLocalLayer::callback(const messages::objectsList::ConstPtr& msg)
{
	ROS_INFO("Local gooot coords");
	while (idobjs.size()>0) idobjs.pop_back();
	for (int i=0;i<msg->matched_objects.size();i++)
	{
		coord c;
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