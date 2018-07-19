#include "mujoco_dyros.h"








//-------------------------------- main function ----------------------------------------

int main(int argc, char** argv)
{
    //ros init
    ros::init(argc, argv, "mujoco_ros");
    ros::NodeHandle nh("~");
    std::string key_file;
    nh.param<std::string>("license", key_file, "mjkey.txt");

    // print version, check compatibility
    printf("MuJoCo Pro library version %.2lf\n", 0.01*mj_version());
    if( mjVERSION_HEADER!=mj_version() )
        mju_error("Headers and library have different versions");


    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != NULL)
        ROS_INFO_COND(false,"Current working dir: %s\n", cwd);

    ROS_INFO("license is at %s", key_file.c_str());
    // activate MuJoCo license
    // locate mjkey.txt file at /home/<usrname>
    mj_activate(key_file.c_str());

    // init GLFW
    if (!glfwInit())
        return 1;

    // get refreshrate
    refreshrate = glfwGetVideoMode(glfwGetPrimaryMonitor())->refreshRate;

    // multisampling
    glfwWindowHint(GLFW_SAMPLES, 4);

    // create widdow
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Simulate", NULL, NULL);
    //GLFWwindow* window2;// = glfwCreateWindow(800, 600, "Sub camera", NULL, NULL);
    if( !window )
    {
        glfwTerminate();
        return 1;
    }
    /*
    if( !window2 )
    {
      glfwTerminate();
      return 1;
    }
*/
    // make context current, disable v-sync
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);

    // save window-to-framebuffer pixel scaling (needed for OSX scaling)
    int width, width1, height;
    glfwGetWindowSize(window, &width, &height);
    glfwGetFramebufferSize(window, &width1, &height);
    //window2buffer = (double)width1 / (double)width;

    // init MuJoCo rendering, get OpenGL info
    mjv_makeScene(&scn, 1000);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&vopt);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, fontscale);
    profilerinit();
    sensorinit();

    // Second display for fixed cam, but not working. why?//
    /*
    glfwMakeContextCurrent(window2);
    mjv_makeScene(&scn2, 1000);
    mjv_defaultCamera(&cam2);
    mjr_defaultContext(&con2);
    mjr_makeContext(m,&con2,fontscale);
    */

    glfwMakeContextCurrent(window);
    // set GLFW callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    glfwSetDropCallback(window, drop);
    glfwSetWindowRefreshCallback(window, render);



    // set MuJoCo time callback for profiling
    mjcb_time = timer;

    // load model if filename given as ros::param
    std::string model_file;
    if(nh.getParam("model_file",model_file))
        loadmodel(window,model_file.c_str());
    ROS_INFO("model is at %s", model_file.c_str());

    //register callback function//
    mjcb_control=mycontroller;
    mjcb_sensor=sensor_callback;




    //register publisher & subscriber
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/mujoco_ros_interface/joint_states", 1);
    sim_time_pub = nh.advertise<std_msgs::Float32>("/mujoco_ros_interface/sim_time",1);
    sensor_state_pub = nh.advertise<mujoco_ros_msgs::SensorState>("/mujoco_ros_interface/sensor_states",1);
    joint_set = nh.subscribe<sensor_msgs::JointState>("/mujoco_ros_interface/joint_set",1,jointset_callback,ros::TransportHints().tcpNoDelay(true));
    sim_command_sub = nh.subscribe("/mujoco_ros_interface/sim_command_con2sim",100,sim_command_callback);
    sim_command_pub = nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_sim2con",1);



    // main loop
    while(( !glfwWindowShouldClose(window) )&&ros::ok() )
    {
        // simulate and render
        render(window);
        //render_depth(window, window2);

        // handle events (this calls all callbacks)
        glfwPollEvents();
    }

    // delete everything we allocated
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    mjr_freeContext(&con2);
    mjv_freeScene(&scn2);

    // terminate
    glfwTerminate();
    mj_deactivate();

    return 0;
}


