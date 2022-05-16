#include <ros/ros.h>
#include <iostream>
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
#include <cstdlib> // for exit function
#include <vector>

// This program output values from an array to a file named example2.dat
int main(int argc, char** argv)
{
    ros::init(argc, argv, "WayPoint_Saver");

    ofstream outdata; // outdata is like cin
 
    double num[3] = {1.5, 1.5, 1.5}; // list of output values
    double num2[3] = {1.5, -1.0, 1.5};

    outdata.open("/home/chanzz/catkin_ws/src/cm_navigation/cm_waypoint.txt"); // opens the file
    if( !outdata )
    { // file couldn't be opened
        cerr << "Error: file could not be opened" << endl;
        exit(1);
    }

    for (int i=0; i<3; ++i)
    {
        outdata << num[i] << " " << num2[i] << endl;
    }         
    outdata.close();
 
    return 0;
}
