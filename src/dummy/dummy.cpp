#include <iostream>
#include <math.h>
// #include "stdafx.h"
// #include "interpolation.h"
#include "spline.h"
#include <algorithm>
#include <sstream>
namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

using namespace std;
// using namespace alglib;

struct points_for_spline{
      double x;
      double y;
      double heading;
    };

bool sort_function(points_for_spline a, points_for_spline b)
{
  return a.x<b.x;
}

bool makePlan(const points_for_spline& start, const points_for_spline& goal){

    points_for_spline temp_pt, start_pt, goal_pt;
    std::vector<points_for_spline> points;
    double path_length, extra_pt_dist;
//    start_pt.x = start.pose.position.x;
//    start_pt.y = start.pose.position.y;
//    start_pt.heading = getYawFromQuat(start.pose.orientation);
//    goal_pt.heading = getYawFromQuat(goal.pose.orientation);
//    goal_pt.x = goal.pose.position.x;
//    goal_pt.y = goal.pose.position.y;
	  start_pt = start;
	  goal_pt = goal;
    path_length = sqrt(pow(start_pt.x-goal_pt.x,2)+pow(start_pt.y-goal_pt.y,2));
    extra_pt_dist = path_length/50;
    temp_pt.x = start_pt.x-2*extra_pt_dist*cos(start_pt.heading);
    temp_pt.y = start_pt.y-2*extra_pt_dist*sin(start_pt.heading);
    points.push_back(temp_pt);
    temp_pt.x = start_pt.x-extra_pt_dist*cos(start_pt.heading);
    temp_pt.y = start_pt.y-extra_pt_dist*sin(start_pt.heading);
    points.push_back(temp_pt);
    points.push_back(start_pt);
    points.push_back(goal_pt);
    temp_pt.x = goal_pt.x+extra_pt_dist*cos(goal_pt.heading);
    temp_pt.y = goal_pt.y+extra_pt_dist*sin(goal_pt.heading);
    points.push_back(temp_pt);
    temp_pt.x = goal_pt.x+2*extra_pt_dist*cos(goal_pt.heading);
    temp_pt.y = goal_pt.y+2*extra_pt_dist*sin(goal_pt.heading);
    points.push_back(temp_pt);
    // std::string x_str, y_str;
    // x_str = "[";
    // y_str = "[";
    // for (unsigned int i =0 ; i< points.size()-1; i++)
    // {
    //     x_str = x_str + patch::to_string(points.at(i).x) + ",";
    //     y_str = y_str + patch::to_string(points.at(i).y) + ",";
    // }
    // x_str = x_str + patch::to_string(points.at(points.size()-1).x)+ "]";
    // y_str = y_str + patch::to_string(points.at(points.size()-1).y)+ "]";
    // real_1d_array x(x_str.c_str());
    // real_1d_array y(y_str.c_str());
    // spline1dinterpolant s;
    // const xparams _xparams = alglib::xdefault;
    // spline1dbuildcubic(x, y, s,_xparams);
    for (unsigned int i =0; i<points.size(); i++)
    {
      cout << points.at(i).x << endl;
    }
    std::vector<points_for_spline> path_points;

    std::vector<double> x(6),y(6);
      for (unsigned int i =0; i<points.size(); i++)
      {
        x.at(i) = points.at(i).x;
        y.at(i) = points.at(i).y;
      }
      std::sort(points.begin(), points.end(), sort_function);
      tk::spline s;
      s.set_points(x,y);
    if(goal_pt.x > start_pt.x)
    {
      

      for (double i = start_pt.x; i<goal_pt.x; i+=extra_pt_dist)
      {
          temp_pt.x = i;
          temp_pt.y = s(i);
          double next_y = s(i+extra_pt_dist);
          temp_pt.heading = atan2(next_y-temp_pt.y, extra_pt_dist);
          path_points.push_back(temp_pt);
      }
    }
    else // Do something for moving in just y 
    {
      for (double i = start_pt.x; i<goal_pt.x; i-=extra_pt_dist)
      {
          temp_pt.x = i;
          temp_pt.y = s(i);
          double next_y = s(i-extra_pt_dist);
          temp_pt.heading = atan2(next_y-temp_pt.y, -extra_pt_dist);
          path_points.push_back(temp_pt);
      }
    }
    
    for (int i=0; i<path_points.size(); i++){
     cout << path_points.at(i).x << endl;
   }
  return true;
 }

int main()
{
	points_for_spline start, goal;
	std::cout << "enter start point \n";
	std::cin >> start.x >> start.y >> start.heading;
	std:: cout << "enter the goal point \n";
	std::cin >> goal.x >> goal.y >> goal.heading;
	makePlan(start,goal);
}