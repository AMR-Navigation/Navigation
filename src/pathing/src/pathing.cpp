#include "../include/pathing.h"
#include <pluginlib/class_list_macros.h>

#include <iostream>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::DynamicLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

// Macros
#define PEOPLERADIUS 30
#define CLASS_CHAIR "Chair"
#define CLASS_PEOPLE "People"
#define CLASS_ROBOT "Robot"
#define CLASS_SOFA "Sofa"
#define CLASS_TABLE "Table"
#define CLASS_UNKNOWN "unknown"

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
	default_value_ = NO_INFORMATION;
	matchSize();

	dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
	dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&DynamicLayer::reconfigureCB, this, _1, _2);
	dsrv_->setCallback(cb);

	this->sub = nh.subscribe("/coordinates_topic", 1000, &DynamicLayer::callback, this);
	ROS_INFO("Subscibed to coords topic as ");
	std::cout << name_ << std::endl;
}

void DynamicLayer::matchSize()
{
	Costmap2D* master = layered_costmap_->getCostmap();
	resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
}


void DynamicLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
	enabled_ = config.enabled;
}

void DynamicLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
	if (!enabled_)
		return;

	// MY PROCESSING HERE:

	for (int i=0;i<idobjs.size();i++) 
	{
		int r;
		float rf;
		coord badcoord = goodtobad(idobjs[i].location.x,idobjs[i].location.y);
		if (idobjs[i].classification==CLASS_PEOPLE) {
			r=40;
			rf=.4;

			// Create circle hopefully one meter ahead of the object
			coord dircoord;
			unsigned int dx;
			unsigned int dy;
			dircoord.x = badcoord.x+3*cos(idobjs[i].direction);
			dircoord.y = badcoord.y+3*sin(idobjs[i].direction);
			if (worldToMap(dircoord.x,dircoord.y,dx,dy)) {
				setCircleCost(dx,dy,20,.01);
				std::cout << "CREATING DIRECTIONAL OBSTACLE IN DIRECTION " << idobjs[i].direction / 3.14159 << "pi" << std::endl;
			}
			/*dircoord.x = badcoord.x-3*cos(idobjs[i].direction);
			dircoord.y = badcoord.y-3*sin(idobjs[i].direction);
			if (worldToMap(dircoord.x,dircoord.y,dx,dy)) {
				setCircleCost(dx,dy,20,.01);
				std::cout << "CREATING DIRECTIONAL OBSTACLE IN DIRECTION " << idobjs[i].direction / 3.14159 << "pi" << std::endl;
			}*/
			
		} else if (idobjs[i].classification==CLASS_UNKNOWN) {
			r = 25;
			rf=.5;

			// Create circle hopefully one meter ahead of the object
			/*coord dircoord;
			unsigned int dx;
			unsigned int dy;
			dircoord.x = badcoord.x+2*cos(idobjs[i].direction);
			dircoord.y = badcoord.y+2*sin(idobjs[i].direction);
			if (worldToMap(dircoord.x,dircoord.y,dx,dy)) {
				setCircleCost(dx,dy,15,1);
				std::cout << "CREATING DIRECTIONAL OBSTACLE IN DIRECTION " << idobjs[i].direction / 3.14159 << "pi" << std::endl;
			}*/
		} else {
			r = 20;
			rf=1;
		}
		double mr = (double)r/.05;
		*min_x = std::min(*min_x, badcoord.x - mr);
		*min_y = std::min(*min_y, badcoord.y - mr);
		*max_x = std::max(*max_x, badcoord.x + mr);
		*max_y = std::max(*max_y, badcoord.y + mr);




		unsigned int mx;
		unsigned int my;
		worldToMap(badcoord.x, badcoord.y, mx, my);
		if(worldToMap(badcoord.x, badcoord.y, mx, my)){
			setCircleCost(mx,my,r,rf);
		}
	}

	// ------------------------------------------------------------------------
	double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
	*min_x = std::min(*min_x, mark_x);
	*min_y = std::min(*min_y, mark_y);
	*max_x = std::max(*max_x, mark_x);
	*max_y = std::max(*max_y, mark_y);

}

void DynamicLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
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
			if (master_grid.getCost(i,j)<costmap_[index]) master_grid.setCost(i, j, costmap_[index]); 
			if (costmap_[index]>20) costmap_[index]-=20;
			else costmap_[index] = FREE_SPACE;
		}
	}
}

void DynamicLayer::setCircleCost(int center_x, int center_y, int radius, float ratefactor)
{
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
								if ((int) getCost(x,y) < (int) (std::pow(1-distance/(float)radius, ratefactor)*((float)LETHAL_OBSTACLE)) or getCost(x,y)==255) setCost(x, y, (int) (std::pow(1-distance/(float)radius, ratefactor)*((float)LETHAL_OBSTACLE)));
						}
				}
		}
}

void DynamicLayer::callback(const messages::objectsList::ConstPtr& msg)
{
	while (idobjs.size()>0) idobjs.pop_back();
	for (int i=0;i<msg->matched_objects.size();i++)
	{
		object o;
		coord c;
		c.x = msg->matched_objects[i].x;
		c.y = msg->matched_objects[i].y;
		o.location = c;
		o.classification = msg->matched_objects[i].classification;
		o.direction = msg->matched_objects[i].direction;
		idobjs.push_back(o);
	}
	for (int i=0;i<msg->unmatched_objects.size();i++)
	{
		object o;
		coord c;
		c.x = msg->unmatched_objects[i].x;
		c.y = msg->unmatched_objects[i].y;
		o.location = c;
		o.classification = msg->unmatched_objects[i].classification;
		o.direction = msg->unmatched_objects[i].direction;
		idobjs.push_back(o);
	}
}

} // end namespace