#ifndef SIMPLE_LAYER_H_
#define SIMPLE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include "messages/objectsList.h"



#include <vector>
#include <string>
#include <cmath>




namespace simple_local_layer_namespace
{
struct coord 
{
	float x;
	float y;
};

struct object
{
	coord location;
	std::string classification;
	float direction;
};

typedef std::vector<coord> COORDLIST;
typedef std::vector<object> OBJLIST;

class DynamicLocalLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  DynamicLocalLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

  void callback(const messages::objectsList::ConstPtr& msg);
  void setcostfor(object o, double robot_x, double robot_y);
  void setCircleCost(unsigned int center_x, unsigned int center_y, int radius, float ratefactor);
  bool isInEgg(int x, int y, int major, int minor, float direction);
  void setEggCost(int center_x, int center_y, float direction);
  ros::Subscriber sub;
  OBJLIST idobjs; // change later
  

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  
};
}
#endif