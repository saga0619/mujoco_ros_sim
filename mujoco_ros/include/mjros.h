#ifndef _MJROS_H
#define _MJROS_H

//Mujoco include
#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include <thread>
#include <mutex>
#include <chrono>

#include <iomanip>

//Ros include
#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <mujoco_ros_msgs/SensorState.h>
#include <mujoco_ros_msgs/JointSet.h>
#include <mujoco_ros_msgs/SimStatus.h>
#include <mujoco_ros_msgs/applyforce.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>

#include <deque>

#ifdef COMPILE_SHAREDMEMORY
#include "shm_msgs.h"
SHMmsgs *mj_shm_;
int shm_msg_id;

#define USE_SHM true
#else
#define USE_SHM false
#endif

//-------------------------------- global -----------------------------------------------
//-----mujoco var-----
// constants
const int maxgeom = 5000;         // preallocated geom array in mjvScene
const double syncmisalign = 0.1;  // maximum time mis-alignment before re-sync
const double refreshfactor = 0.7; // fraction of refresh available for simulation

// model and data
mjModel *m = NULL;
mjData *d = NULL;
char filename[1000] = "";

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

// OpenGL rendering and UI
GLFWvidmode vmode;
int windowpos[2];
int windowsize[2];
mjrContext con;
GLFWwindow *window = NULL;
mjuiState uistate;
mjUI ui0, ui1;

int key_ui = 0;

int com_latency = 0;

// UI settings not contained in MuJoCo structures
struct setting_
{
    // file
    int exitrequest = 0;

    // option
    int spacing = 0;
    int color = 0;
    int font = 0;
    int ui0 = 1;
    int ui1 = 0;
    int help = 0;
    int info = 1;
    int profiler = 0;
    int sensor = 0;
    int fullscreen = 0;
    int vsync = 1;
    int busywait = 1;
    int realtime = 0;
    int debug = 0;
    int testbtn2 = 1;
    int timecheck = 0;
    int controlui = 0;
    int link_info = 0;

    // simulation
    int run = 0;
    int key = 0;
    char key_s[40] = "0";
    int loadrequest = 0;

    // watch
    char field[mjMAXUITEXT] = "qpos";
    int index = 0;

    // physics: need sync
    int disable[mjNDISABLE];
    int enable[mjNENABLE];

    // rendering: need sync
    int camera = 0;
};

setting_ settings;

// section ids
enum
{
    // left ui
    SECT_FILE = 0,
    SECT_OPTION,
    SECT_SIMULATION,
    SECT_PHYSICS,
    SECT_RENDERING,
    SECT_GROUP,
    NSECT0,

    // right ui
    SECT_WATCH = 0,
    SECT_JOINT,
    SECT_CONTROL,
    NSECT1
};

// file section of UI
const mjuiDef defFile[] =
    {
        {mjITEM_SECTION, "File", 1, NULL, "AF"},
        {mjITEM_BUTTON, "Save xml", 2, NULL, ""},
        {mjITEM_BUTTON, "Save mjb", 2, NULL, ""},
        {mjITEM_BUTTON, "Print model", 2, NULL, "CM"},
        {mjITEM_BUTTON, "Print data", 2, NULL, "CD"},
        {mjITEM_BUTTON, "Quit", 1, NULL, "CQ"},
        {mjITEM_END}};

// option section of UI
const mjuiDef defOption[] =
    {
        {mjITEM_SECTION, "Option", 1, NULL, "AO"},
        {mjITEM_SELECT, "Spacing", 1, &settings.spacing, "Tight\nWide"},
        {mjITEM_SELECT, "Color", 1, &settings.color, "Default\nOrange\nWhite\nBlack"},
        {mjITEM_SELECT, "Font", 1, &settings.font, "50 %\n100 %\n150 %\n200 %\n250 %\n300 %"},
        {mjITEM_CHECKINT, "Left UI (Tab)", 1, &settings.ui0, " #258"},
        {mjITEM_CHECKINT, "Right UI", 1, &settings.ui1, "S#258"},
        {mjITEM_CHECKINT, "Help", 2, &settings.help, " #290"},
        {mjITEM_CHECKINT, "Info", 2, &settings.info, " #291"},
        {mjITEM_CHECKINT, "Profiler", 2, &settings.profiler, " #292"},
        {mjITEM_CHECKINT, "Sensor", 2, &settings.sensor, " #293"},
#ifdef __APPLE__
        {mjITEM_CHECKINT, "Fullscreen", 0, &settings.fullscreen, " #294"},
#else
        {mjITEM_CHECKINT, "Fullscreen", 1, &settings.fullscreen, " #294"},
#endif
        {mjITEM_CHECKINT, "Vertical Sync", 1, &settings.vsync, " #295"},
        {mjITEM_CHECKINT, "Real Time", 1, &settings.busywait, " #296"},
        {mjITEM_CHECKINT, "Debug", 1, &settings.debug, " #297"},
        {mjITEM_CHECKINT, "Time Check", 1, &settings.timecheck, " #298"},
        {mjITEM_CHECKINT, "Control by UI", 1, &settings.controlui, ""},
        {mjITEM_CHECKINT, "Link info", 1, &settings.link_info, ""},
        {mjITEM_END}};

// simulation section of UI
mjuiDef defSimulation[] =
    {
        {mjITEM_SECTION, "Simulation", 1, NULL, "AS"},
        {mjITEM_RADIO, "", 2, &settings.run, "Pause\nRun"},
        {mjITEM_BUTTON, "Reset", 2, NULL, "C#259"},
        {mjITEM_BUTTON, "Reload", 2, NULL, "CL"},
        {mjITEM_BUTTON, "Align", 2, NULL, "CA"},
        {mjITEM_BUTTON, "Copy pose", 2, NULL, "CC"},
        //{mjITEM_SLIDERINT, "Key", 3, &settings.key, "0 0"},
        {mjITEM_BUTTON, "Key + ", 2, NULL, " #266"},
        {mjITEM_BUTTON, "Key - ", 2, NULL, " #267"},
        {mjITEM_STATIC, "Key", 2, NULL, " 0"},
        {mjITEM_BUTTON, "Latency + ", 2, NULL, ""},
        {mjITEM_BUTTON, "Latency - ", 2, NULL, ""},
        {mjITEM_STATIC, "Latency", 2, NULL, " 0"},
        {mjITEM_BUTTON, "Reset to key", 3, NULL, " #259"},
        {mjITEM_BUTTON, "Set key", 3},
        {mjITEM_END}};

// watch section of UI
const mjuiDef defWatch[] =
    {
        {mjITEM_SECTION, "Watch", 0, NULL, "AW"},
        {mjITEM_EDITTXT, "Field", 2, settings.field, "qpos"},
        {mjITEM_EDITINT, "Index", 2, &settings.index, "1"},
        {mjITEM_STATIC, "Value", 2, NULL, " "},
        {mjITEM_END}};

// help strings
const char help_content[] =
    "Alt mouse button\n"
    "UI right hold\n"
    "UI title double-click\n"
    "Space\n"
    "Esc\n"
    "Right arrow\n"
    "Left arrow\n"
    "Down arrow\n"
    "Up arrow\n"
    "Page Up\n"
    "Double-click\n"
    "Right double-click\n"
    "Ctrl Right double-click\n"
    "Scroll, middle drag\n"
    "Left drag\n"
    "[Shift] right drag\n"
    "Ctrl [Shift] drag\n"
    "Ctrl [Shift] right drag";

const char help_title[] =
    "Swap left-right\n"
    "Show UI shortcuts\n"
    "Expand/collapse all  \n"
    "Pause\n"
    "Free camera\n"
    "Step forward\n"
    "Step back\n"
    "Step forward 100\n"
    "Step back 100\n"
    "Select parent\n"
    "Select\n"
    "Center\n"
    "Track camera\n"
    "Zoom\n"
    "View rotate\n"
    "View translate\n"
    "Object rotate\n"
    "Object translate";

// info strings
char info_title[1000];
char info_content[1000];

void profilerinit(void);
void profilerupdate(void);
void profilershow(mjrRect rect);
void sensorinit(void);
void sensorupdate(void);
void sensorshow(mjrRect rect);
void infotext(char *title, char *content, double interval);
void printfield(char *str, void *ptr);
void watch(void);
void makephysics(int oldstate);
void makerendering(int oldstate);
void makegroup(int oldstate);
void makejoint(int oldstate);
void makecontrol(int oldstate);
void makesections(void);
void alignscale(void);
void copykey(void);
mjtNum timer(void);
void cleartimers(void);
void updatesettings(void);
void drop(GLFWwindow *window, int count, const char **paths);
void loadmodel(void);
int uiPredicate(int category, void *userdata);
void uiLayout(mjuiState *state);
void uiEvent(mjuiState *state);
void prepare(void);
void render(GLFWwindow *window);
void simulate(void);
void init();
void rosPollEvents();

std::mutex mtx;

//---------------ROS Var-----------------------
ros::Publisher joint_state_pub;
ros::Publisher sensor_state_pub;
ros::Subscriber joint_set;
ros::Subscriber joint_init;
ros::Subscriber sim_command_sub;
ros::Publisher sim_command_pub;
ros::Publisher sim_status_pub;

// apply external force
ros::Subscriber force_apply_sub;
// std_msgs::Float32MultiArray ext_force_msg_;
mujoco_ros_msgs::applyforce ext_force_msg_;
bool ext_force_applied_ = false;
std::vector<float> applied_ext_force_;
unsigned int force_appiedd_link_idx_;
mjvGeom* arrow;
void arrowshow(mjvGeom* arrow);
void makeArrow(mjvGeom* arrow);
void force_apply_callback(const std_msgs::Float32MultiArray &msg);

//mujoco_ros_msgs::JointState joint_state_msg_;
//mujoco_ros_msgs::JointSet joint_set_msg_;
mujoco_ros_msgs::SensorState sensor_state_msg_;
mujoco_ros_msgs::SimStatus sim_status_msg_;
sensor_msgs::JointState joint_state_msg_;
//sensor_msgs::JointState joint_set_msg_;
mujoco_ros_msgs::JointSet joint_set_msg_;
std_msgs::Float32 sim_time;
ros::Publisher sim_time_pub;

std::vector<float> command;
std::vector<float> command2;

int loadmodel_request = 0;

bool ros_time_sync_reset;

//reset start time
bool ros_sim_started = true;
bool controller_reset_check = true;
bool controller_init_check = true;
bool reset_request = false;

bool pause_check = true;

bool pub_total_mode = false;

bool use_shm = false;

//bool for custom applied force
bool custom_ft_applied = false;

ros::Duration sim_time_ros;
ros::Time sim_time_run;

ros::Duration sim_time_now_ros;

ros::Duration ros_sim_runtime;
ros::Time sync_time_test;

std::string ctrlstat = "Missing";

std::vector<float> ctrl_command_temp_;

std::deque<std::vector<float>> ctrl_cmd_que_;

mjtNum *ctrl_command;
mjtNum *ctrl_command2;

bool cmd_rcv = false;

// user state for pub

float com_time;
float dif_time;

double t_bf = 0;

double sim_cons_time = 0;
//void c_pause();
//void c_slowmotion();
void c_reset();

//---------------------callback functions --------------------------------

void jointset_callback(const mujoco_ros_msgs::JointSetConstPtr &msg);
void sim_command_callback(const std_msgs::StringConstPtr &msg);
void state_publisher_init();
void state_publisher();
void mujoco_ros_connector_init();
void mycontroller(const mjModel *m, mjData *d);

#endif