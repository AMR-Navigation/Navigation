#include "../include/localpathing.h"
#include <pluginlib/class_list_macros.h>

#include <iostream>

PLUGINLIB_EXPORT_CLASS(simple_local_layer_namespace::DynamicLocalLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

int LOCALDIM = 60;
float RES = .05;

// Macros
#define PEOPLERADIUS 30
#define CLASS_CHAIR "Chair"
#define CLASS_PEOPLE "People"
#define CLASS_ROBOT "Robot"
#define CLASS_SOFA "Sofa"
#define CLASS_TABLE "Table"
#define CLASS_UNKNOWN "unknown"

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
	if (loc.x < 0 or loc.x > LOCALDIM or loc.y <0 or loc.y > LOCALDIM) 
		return false;
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
		setcostfor(idobjs[i],robot_x,robot_y);
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
			if (costmap_[index]>50) costmap_[index]-=50;
			else costmap_[index] = FREE_SPACE;
		}
	}
}

void DynamicLocalLayer::setcostfor(object o, double robot_x, double robot_y) {
	coord badcoord = goodtobad(o.location.x, o.location.y);
	unsigned int cx;
	unsigned int cy;
	if (!worldtolocal(cx,cy,badcoord.x,badcoord.y,robot_x,robot_y))
		return;
	
	if (o.classification==CLASS_UNKNOWN) {
		// Set circle with radius r and ratefactor 1
		int r = 20;
		setCircleCost(cx,cy,r,.2);

		// Create circle hopefully one meter ahead of the object
		/*coord dircoord;
		unsigned int dx;
		unsigned int dy;
		dircoord.x = badcoord.x+.5*cos(o.direction);
		dircoord.y = badcoord.y+.5*sin(o.direction);
		if (worldtolocal(dx,dy,dircoord.x,dircoord.y,robot_x,robot_y)) {
			setCircleCost(dx,dy,7,2);
			std::cout << "CREATING DIRECTIONAL OBSTACLE" << std::endl;
		}*/
	} else if (o.classification==CLASS_SOFA or o.classification==CLASS_CHAIR) {
		int r = 10;
		setCircleCost(cx,cy,r,.5);
	} else if (o.classification==CLASS_PEOPLE) {
		ROS_INFO("Drawing circle for a PERSON");
		int r=PEOPLERADIUS;
		setCircleCost(cx,cy,r, .5);

		// Create circle hopefully one meter ahead of the object
		coord dircoord;
		unsigned int dx;
		unsigned int dy;
		dircoord.x = badcoord.x+cos(o.direction);
		dircoord.y = badcoord.y+sin(o.direction);
		if (worldtolocal(dx,dy,dircoord.x,dircoord.y,robot_x,robot_y)) {
			setCircleCost(dx,dy,7,.2);
		}
	} else if (o.classification==CLASS_ROBOT) {
		int r=20;
		setCircleCost(cx,cy,r, .5);
	}
}

void DynamicLocalLayer::setCircleCost(unsigned int center_x, unsigned int center_y, int radius, float ratefactor)
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
								if ((int) getCost(x,y) < (int) (std::pow(1-distance/(float)radius, ratefactor)*((float)LETHAL_OBSTACLE)) or getCost(x,y)==255) setCost(x, y, (int) (std::pow(1-distance/(float)radius, ratefactor)*((float)LETHAL_OBSTACLE)));
						}
				}
		}
}

void DynamicLocalLayer::setEggCost(int center_x, int center_y, float direction)
{
	int major = 20;
	int minor = 10; // also the radius
	// Iterate over the cells in the egg's bounding box
	for (int x = center_x - major; x <= center_x + major; x++) {
			for (int y = center_y - minor; y <= center_y + minor; y++) {
				if (x<0 or x>getSizeInCellsX() or y<0 or y>getSizeInCellsY()) continue;

				// Calculate the distance from the center to the current cell
				int dx = x - center_x;
				int dy = y - center_y;
				double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

				// Check if the cell (x, y) is within the circle's radius
				if (isInEgg(x,y,major,minor,direction)) {
						setCost(x, y, (int) (std::pow(1-distance/(float)major, 1)*((float)LETHAL_OBSTACLE)));
				}
			}
	}
}

bool DynamicLocalLayer::isInEgg(int x, int y, int major, int minor, float direction) 
{
	return true;
}

void DynamicLocalLayer::callback(const messages::objectsList::ConstPtr& msg)
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