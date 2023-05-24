/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Eitan Marder-Eppstein, Sachin Chitta, 
* Edited by Daehyung Park
*********************************************************************/
#include <angles/angles.h>
#include <py_astar_planner/py_astar_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#undef NUMPY_IMPORT_ARRAY_RETVAL
#define NUMPY_IMPORT_ARRAY_RETVAL


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(py_astar_planner::PyAstarPlanner, nav_core::BaseGlobalPlanner)

namespace py_astar_planner {

  PyAstarPlanner::PyAstarPlanner()
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){}

  PyAstarPlanner::PyAstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  PyAstarPlanner::~PyAstarPlanner() {
    this->pyFinalize(pModule_);
    // deleting a nullptr is a noop
    delete world_model_;
  }
  
  void PyAstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      convert_offset_ = 0.5;

      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      world_model_ = new base_local_planner::CostmapModel(*costmap_); 
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
      
      // Python Embedding
      this->pyInitialize("py_astar_planner.planner", &pModule_);
      import_array();
      
      initialized_ = true;

#ifdef DEBUG
      int x_size = costmap_->getSizeInCellsX();
      int y_size = costmap_->getSizeInCellsY();
      unsigned char *ucharmap = costmap_->getCharMap();
      unsigned int start_x_i = 200;
      unsigned int start_y_i = 199;
      unsigned int goal_x_i = 215;
      unsigned int goal_y_i = 239;      
      this->pyCostmapPlot(pModule_, ucharmap, x_size, y_size, start_x_i, start_y_i, goal_x_i, goal_y_i);
#endif
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }
      
  bool PyAstarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

#ifdef DEBUG
    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
#endif

    //clear the plan, just in case
    plan.clear();
    costmap_ = costmap_ros_->getCostmap();
    std::string global_frame = costmap_ros_->getGlobalFrameID();

    if(goal.header.frame_id != global_frame){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          global_frame, goal.header.frame_id.c_str());
      return false;
    }

    if (start.header.frame_id != global_frame) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame, start.header.frame_id.c_str());
        return false;
    }
    
    // Goal
    double wgoal_x = goal.pose.position.x; // world pos
    double wgoal_y = goal.pose.position.y;
    unsigned int goal_x_i, goal_y_i; // map indices
    double goal_x, goal_y; // map pos
    if (!costmap_->worldToMap(wgoal_x, wgoal_y, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    goal_x = goal_x_i;
    goal_y = goal_y_i;

    // Start
    double wstart_x = start.pose.position.x;
    double wstart_y = start.pose.position.y;
    unsigned int start_x_i, start_y_i;
    double start_x, start_y;
    if (!costmap_->worldToMap(wstart_x, wstart_y, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    start_x = start_x_i;
    start_y = start_y_i;
    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, start_x, start_y);

    // Python
    int x_size = costmap_->getSizeInCellsX();
    int y_size = costmap_->getSizeInCellsY();
    unsigned char *ucharmap = costmap_->getCharMap();
      
    std::vector<std::pair<float, float> > path;
    this->pyMakePlan(pModule_, ucharmap, x_size, y_size, start_x_i, start_y_i, goal_x_i, goal_y_i, path);
    if( path.size()<1 ){
        ROS_ERROR("NO PATH!");
        return false;
    }
#ifdef DEBUG    
    std::cout << "path size = " << path.size() << std::endl;
#endif

    ros::Time plan_time = ros::Time::now();
    double world_x, world_y;
    for (int i = 0 ; i<=path.size() -1; i++) {
        std::pair<float, float> point = path[i];

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = point.first;
        pose.pose.position.y = point.second;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
   
    //publish the plan for visualization purposes
    publishPlan(plan);
#ifdef DEBUG    
    ROS_WARN("Published a plan!");
#endif
    
    return (true);
  }

    void PyAstarPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
        if (!initialized_) {
            ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //set the associated costs in the cost map to be free
        costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
    }

    void PyAstarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //create a message for the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }
    
  // Python embedding initialization
  void PyAstarPlanner::pyInitialize(char* module_name, PyObject **pModule){
    Py_Initialize();

    PyObject *pName = PyUnicode_DecodeFSDefault(module_name);
    *pModule = PyImport_Import(pName);
    
    Py_DECREF(pName);     
  }

  // Python embedding ends
  void PyAstarPlanner::pyFinalize(PyObject *pModule)
  {
      ROS_WARN("pyFinalize()!");
      Py_DECREF(pArgs_);
      Py_DECREF(pArray_);

      Py_DECREF(px_size_);
      Py_DECREF(py_size_);
      Py_DECREF(px_start_);
      Py_DECREF(py_start_);
      Py_DECREF(px_goal_);
      Py_DECREF(py_goal_);    
      Py_DECREF(pFunc_);
      Py_DECREF(pValue_);
      Py_DECREF(ptemp_);
     
      Py_DECREF(pModule);
      Py_FinalizeEx();
  }

    // Python Make Plan
    void PyAstarPlanner::pyMakePlan(PyObject *pModule, unsigned char *arg, int x_size, int y_size, unsigned int x_start, unsigned int y_start, unsigned int x_goal, unsigned int y_goal, std::vector<std::pair<float, float> >& path)
  {
      npy_intp m = x_size*y_size;    
      pFunc_ = PyObject_GetAttrString(pModule, "make_plan");
      pArgs_ = PyTuple_New(7);

      pArray_ = PyArray_SimpleNewFromData(
          1, &m, NPY_UBYTE, reinterpret_cast<void*>(arg));
      PyTuple_SetItem(pArgs_, 0, pArray_);

      px_size_ = PyLong_FromLong((long)x_size);
      py_size_ = PyLong_FromLong((long)y_size);
      PyTuple_SetItem(pArgs_, 1, px_size_);
      PyTuple_SetItem(pArgs_, 2, py_size_);
      px_start_ = PyLong_FromLong((long)x_start);
      py_start_ = PyLong_FromLong((long)y_start);
      PyTuple_SetItem(pArgs_, 3, px_start_);
      PyTuple_SetItem(pArgs_, 4, py_start_);
      px_goal_ = PyLong_FromLong((long)x_goal);
      py_goal_ = PyLong_FromLong((long)y_goal);
      PyTuple_SetItem(pArgs_, 5, px_goal_);
      PyTuple_SetItem(pArgs_, 6, py_goal_);
    
      pValue_ = PyObject_CallObject(pFunc_, pArgs_);
    
      if (pValue_ != NULL)
      {
#ifdef DEBUG    
          std::cout << "Result of call: " << PyList_Check(pValue_) << std::endl;
#endif
          int count = (int) PyList_Size(pValue_);
          std::pair<float, float> current;

          unsigned int x_i;
          unsigned int y_i;
          double x,y;
        
          for (int i = 0 ; i < count ; i++ )
          {
              ptemp_ = PyList_GetItem(pValue_,i);
              x_i = PyLong_AsLong( PyList_GetItem(ptemp_, 0));
              y_i = PyLong_AsLong( PyList_GetItem(ptemp_, 1));
            
              costmap_->mapToWorld(x_i, y_i, x, y);
              current.first = x;
              current.second = y;        
              path.push_back(current);            
          }
      }
      else
      {
          std::cout <<"Null pValue!!!" << std::endl;
          PyErr_Print();
      }
  }


    void PyAstarPlanner::pyCostmapPlot(PyObject *pModule, unsigned char *arg, int x_size, int y_size, unsigned int x_start, unsigned int y_start, unsigned int x_goal, unsigned int y_goal)
  {
    npy_intp m = x_size*y_size;    
    pFunc_ = PyObject_GetAttrString(pModule, "costmap_plot");
    pArgs_ = PyTuple_New(7);

    pArray_ = PyArray_SimpleNewFromData(
        1, &m, NPY_UBYTE, reinterpret_cast<void*>(arg));
    PyTuple_SetItem(pArgs_, 0, pArray_);

    px_size_ = PyLong_FromLong((long)x_size);
    py_size_ = PyLong_FromLong((long)y_size);
    PyTuple_SetItem(pArgs_, 1, px_size_);
    PyTuple_SetItem(pArgs_, 2, py_size_);
    px_start_ = PyLong_FromLong((long)x_start);
    py_start_ = PyLong_FromLong((long)y_start);
    PyTuple_SetItem(pArgs_, 3, px_start_);
    PyTuple_SetItem(pArgs_, 4, py_start_);
    px_goal_ = PyLong_FromLong((long)x_goal);
    py_goal_ = PyLong_FromLong((long)y_goal);
    PyTuple_SetItem(pArgs_, 5, px_goal_);
    PyTuple_SetItem(pArgs_, 6, py_goal_);
    
    pValue_ = PyObject_CallObject(pFunc_, pArgs_);    
  }
    
};


