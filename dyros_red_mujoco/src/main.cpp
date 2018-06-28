#include "mujoco_dyros.h"


void state_publisher_init(const mjModel* m, mjData* d){
  joint_state_msg_.name.resize(m->nu);
  joint_state_msg_.position.resize(m->nu);
  joint_state_msg_.velocity.resize(m->nu);
  joint_state_msg_.effort.resize(m->nu);


  ROS_INFO("STATE_PUB_INIT");

  for(int i=0;i<m->nu;i++){
  joint_state_msg_.name[i]=m->names[m->name_actuatoradr[i]];
  }


  ROS_INFO("number of generalized coordinates nq = %d", m->nq);
  ROS_INFO("number of degrees of freedom nv = %d", m->nv);
  ROS_INFO("number of actuators/controls nu = %d", m->nu);
  ROS_INFO("number of joints njnt = %d", m->njnt);


}




void state_publisher(const mjModel* m, mjData* d){
  for(int i=0;i<m->nu;i++){
    joint_state_msg_.position[i]=d->qpos[i+7];
    joint_state_msg_.velocity[i]=d->qvel[i+6];
  }
  joint_state_msg_.header.stamp = ros::Time::now();
  joint_state_pub_.publish(joint_state_msg_);

}










void mycontroller(const mjModel* m, mjData* d)
{
  //ROS_INFO("SIMUL_LOOP");
  static double time_ros_start=0;
  if(time_ros_start==0){

    state_publisher_init(m,d);

    ROS_INFO("CONTROL INIT");
    time_ros_start++;

  }



  state_publisher(m,d);



  ros::spinOnce();

  //ROS_INFO("MJ_TIME:%10.5f ros:%10.5f", d->time, ros::Time::now().toSec()-time_ros_start);
}



//-------------------------------- main function ----------------------------------------

int main(int argc, char** argv)
{

    ros::init(argc, argv, "mujoco");
    ros::NodeHandle nh("~");



    ros_sim_start_time = ros::Time::now().toSec();


    // print version, check compatibility
    printf("MuJoCo Pro library version %.2lf\n", 0.01*mj_version());
    if( mjVERSION_HEADER!=mj_version() )
        mju_error("Headers and library have different versions");

    // activate MuJoCo license
    mj_activate("/home/saga/mjpro150/mjkey.txt");

    // init GLFW
    if (!glfwInit())
        return 1;

    // get refreshrate
    refreshrate = glfwGetVideoMode(glfwGetPrimaryMonitor())->refreshRate;

    // multisampling
    glfwWindowHint(GLFW_SAMPLES, 4);

    // create widdow
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Simulate", NULL, NULL);
    if( !window )
    {
        glfwTerminate();
        return 1;
    }

    // make context current, disable v-sync
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // save window-to-framebuffer pixel scaling (needed for OSX scaling)
    int width, width1, height;
    glfwGetWindowSize(window, &width, &height);
    glfwGetFramebufferSize(window, &width1, &height);
    window2buffer = (double)width1 / (double)width;

    // init MuJoCo rendering, get OpenGL info
    mjv_makeScene(&scn, 1000);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&vopt);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, fontscale);
    profilerinit();
    sensorinit();

    // set GLFW callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    glfwSetDropCallback(window, drop);
    glfwSetWindowRefreshCallback(window, render);

    // set MuJoCo time callback for profiling
    mjcb_time = timer;

    // load model if filename given as argument
    if( argc==2 )
        loadmodel(window, argv[1]);





    mjcb_control=mycontroller;



    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/mujoco_ros_interface/joint_set", 1);






    // main loop
    while( !glfwWindowShouldClose(window) )
    {
        // simulate and render
        render(window);



        // handle events (this calls all callbacks)
        glfwPollEvents();
    }

    // delete everything we allocated
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);

    // terminate
    glfwTerminate();
    mj_deactivate();
    return 0;
}


