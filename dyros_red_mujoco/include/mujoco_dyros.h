#ifndef _MUJOCO_DYROS_H
#define _MUJOCO_DYROS_H


#include <ros/ros.h>
#include <Eigen/Dense>
#include <thread>
#include <mujoco.h>
#include <stdio.h>
#include <glfw3.h>
#include <string.h>


#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_publisher.h>


//-------------------------------- global variables -------------------------------------

// model
mjModel* m = 0;
mjData* d = 0;
char lastfile[1000] = "";

//
double ros_sim_start_time ;

// user state
bool paused = false;
bool showoption = false;
bool showinfo = true;
bool showfullscreen = false;
bool slowmotion = false;
bool showdepth = false;
bool showsensor = false;
bool showprofiler = true;
int showhelp = 1;                   // 0: none; 1: brief; 2: full
int fontscale = mjFONTSCALE_150;    // can be 100, 150, 200
int keyreset = -1;                  // non-negative: reset to keyframe

// abstract visualization
mjvScene scn;
mjvCamera cam;
mjvOption vopt;
mjvPerturb pert;
mjvFigure figconstraint;
mjvFigure figcost;
mjvFigure figtimer;
mjvFigure figsize;
mjvFigure figsensor;
char status[1000] = "";

// OpenGL rendering
int refreshrate;
mjrContext con;
float depth_buffer[5120*2880];        // big enough for 5K screen
unsigned char depth_rgb[1280*720*3];  // 1/4th of screen

// selection and perturbation
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
double window2buffer = 1;           // framebuffersize / windowsize (for scaled video modes)

// help strings
const char help_title[] =
"Help\n"
"Option\n"
"Info\n"
"Depth\n"
"Full screen\n"
"Stereo\n"
"Profiler\n"
"Slow motion\n"
"Key reset\n"
"Pause\n"
"Reset\n"
"Forward\n"
"Back\n"
"Forward 100\n"
"Back 100\n"
"Autoscale\n"
"Reload\n"
"Geoms\n"
"Sites\n"
"Select\n"
"Center\n"
"Track\n"
"Zoom\n"
"Translate\n"
"Rotate\n"
"Perturb\n"
"Free Camera\n"
"Camera\n"
"Frame\n"
"Label\n"
"Fontsize";


const char help_content[] =
"F1\n"
"F2\n"
"F3\n"
"F4\n"
"F5\n"
"F6\n"
"F7\n"
"Enter\n"
"Page Up/Down\n"
"Space\n"
"BackSpace\n"
"Right arrow\n"
"Left arrow\n"
"Down arrow\n"
"Up arrow\n"
"Ctrl A\n"
"Ctrl L\n"
"0 - 4\n"
"Shift 0 - 4\n"
"L dblclick\n"
"R dblclick\n"
"Ctrl R dblclick\n"
"Scroll or M drag\n"
"[Shift] R drag\n"
"L drag\n"
"Ctrl [Shift] L/R drag\n"
"Esc\n"
"[ ]\n"
"; '\n"
". /\n"
"- =";

char opt_title[1000] = "";
char opt_content[1000];

//---------------ROS ----------
double timer_ros;
ros::Publisher joint_state_pub_;
sensor_msgs::JointState joint_state_msg_;

//-------------------------------- profiler and sensor ----------------------------------

// init profiler
void profilerinit(void);

// show profiler
void profilerupdate(void);

// show profiler
void profilershow(mjrRect rect);

// init sensor figure
void sensorinit(void);

// update sensor figure
void sensorupdate(void);

// show sensor figure
void sensorshow(mjrRect rect);




//-------------------------------- utility functions ------------------------------------

// center and scale view
void autoscale(GLFWwindow* window);

// load mjb or xml model
void loadmodel(GLFWwindow* window, const char* filename);

// timer in milliseconds
mjtNum timer(void);

// clear all times
void cleartimers(mjData* d);




//--------------------------------- GLFW callbacks --------------------------------------

// keyboard
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);

// mouse button
void mouse_button(GLFWwindow* window, int button, int act, int mods);

// mouse move
void mouse_move(GLFWwindow* window, double xpos, double ypos);

// scroll
void scroll(GLFWwindow* window, double xoffset, double yoffset);

// drop
void drop(GLFWwindow* window, int count, const char** paths);






//-------------------------------- simulation and rendering -----------------------------

// make option string
void makeoptionstring(const char* name, char key, char* buf);

// advance simulation
void simulation(void);

// render
void render(GLFWwindow* window);





#endif
