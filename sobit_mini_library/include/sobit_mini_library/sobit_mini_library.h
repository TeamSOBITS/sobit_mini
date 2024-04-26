#ifndef _SOBIT_MINI_LIBRARY_SOBIT_MINI_LIBRARY_H_
#define _SOBIT_MINI_LIBRARY_SOBIT_MINI_LIBRARY_H_

#include <pybind11/pybind11.h>
#include <ros/ros.h>

class ROSCommonNode
{
  public:
    ROSCommonNode( const std::string &name ) {
        char* cstr = new char[name.size() + 1];
        std::strcpy(cstr, name.c_str()); 
        char **argv = &cstr;
        int argc = 0;
        delete[] cstr;
        ros::init( argc, argv, "sobit_turtlebot_controller_node");
    }
    ROSCommonNode( ) { }
};
#endif /* _SOBIT_MINI_LIBRARY_SOBIT_MINI_LIBRARY_H_ */