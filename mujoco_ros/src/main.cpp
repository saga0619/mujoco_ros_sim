/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/

#include "mjros.h"

// drop file callback
void drop(GLFWwindow *window, int count, const char **paths)
{
    // make sure list is non-empty
    if (count > 0)
    {
        mju_strncpy(filename, paths[0], 1000);
        settings.loadrequest = 1;
        ROS_INFO("DROP REQUEST");
    }
}

// load mjb or xml model
void loadmodel(void)
{
    // clear request
    settings.loadrequest = 0;

    // make sure filename is not empty
    if (!filename[0])
        return;

    // load and compile
    char error[500] = "";
    mjModel *mnew = 0;
    if (strlen(filename) > 4 && !strcmp(filename + strlen(filename) - 4, ".mjb"))
    {
        mnew = mj_loadModel(filename, NULL);
        if (!mnew)
            strcpy(error, "could not load binary model");
    }
    else
    {
        mnew = mj_loadXML(filename, NULL, error, 500);
    }
    if (!mnew)
    {
        printf("%s\n", error);
        return;
    }

    // compiler warning: print and pause
    if (error[0])
    {
        // mj_forward() below will print the warning message
        printf("Model compiled, but simulation warning (paused):\n  %s\n\n",
               error);
        settings.run = 0;
    }

    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    d = mj_makeData(m);

    int i = settings.key;
    d->time = m->key_time[i];
    mju_copy(d->qpos, m->key_qpos + i * m->nq, m->nq);
    mju_copy(d->qvel, m->key_qvel + i * m->nv, m->nv);
    mju_copy(d->act, m->key_act + i * m->na, m->na);

    mj_forward(m, d);

    ros_sim_started = true;
    torque_mj = mj_stackAlloc(d, (int)m->nu);

    // re-create scene and context
    mjv_makeScene(m, &scn, maxgeom);
    mjr_makeContext(m, &con, 50 * (settings.font + 1));

    // clear perturbation state
    pert.active = 0;
    pert.select = 0;
    pert.skinselect = -1;

    // align and scale view, update scene
    alignscale();
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // set window title to model name
    if (window && m->names)
    {
        char title[200] = "Simulate : ";
        strcat(title, m->names);
        glfwSetWindowTitle(window, title);
    }

    // rebuild UI sections
    makesections();

    // full ui update
    uiModify(window, &ui0, &uistate, &con);
    uiModify(window, &ui1, &uistate, &con);

    updatesettings();
    mujoco_ros_connector_init();
    std::cout << " MODEL LOADED " << std::endl;
}
// run event loop
int main(int argc, char **argv)
{
    // :: ROS CUSTUM :: initialize ros
    ros::init(argc, argv, "mujoco_ros");
    ros::NodeHandle nh("~");

    std::string key_file;
    nh.param<std::string>("license", key_file, "mjkey.txt");

    //register publisher & subscriber
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/mujoco_ros_interface/joint_states", 1);
    sim_time_pub = nh.advertise<std_msgs::Float32>("/mujoco_ros_interface/sim_time", 1);
    sensor_state_pub = nh.advertise<mujoco_ros_msgs::SensorState>("/mujoco_ros_interface/sensor_states", 1);
    joint_set = nh.subscribe<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/joint_set", 1, jointset_callback, ros::TransportHints().tcpNoDelay(true));
    //joint_set_mujoco = nh.subscribe<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/joint_set_mujoco",1,joint)
    sim_command_sub = nh.subscribe<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 100, sim_command_callback);
    sim_command_pub = nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_sim2con", 1);

    ROS_INFO("ROS initialize complete");
    sim_time_ros = ros::Duration(0);
    sim_time_run = ros::Time::now();
    sim_time_now_ros = ros::Duration(0);

    // initialize everything
    init(key_file);

    std::string model_file;
    // request loadmodel if file given (otherwise drag-and-drop)
    if (nh.getParam("model_file", model_file))
    {
        mju_strncpy(filename, model_file.c_str(), 1000);
        settings.loadrequest = 2;
        ROS_INFO("model is at %s", model_file.c_str());
    }

    // start simulation thread
    std::thread simthread(simulate);

    // event loop
    while ((!glfwWindowShouldClose(window) && !settings.exitrequest) && ros::ok())
    {
        // start exclusive access (block simulation thread)
        mtx.lock();
        // load model (not on first pass, to show "loading" label)
        if (settings.loadrequest == 1)
        {
            ROS_INFO("Load Request");
            loadmodel();
        }
        else if (settings.loadrequest > 1)
            settings.loadrequest = 1;

        // handle events (calls all callbacks)
        glfwPollEvents();

        // prepare to render
        prepare();

        // ros events
        rosPollEvents();

        // end exclusive access (allow simulation thread to run)
        mtx.unlock();

        // render while simulation is running
        render(window);
    }

    // stop simulation thread
    settings.exitrequest = 1;
    simthread.join();

    // delete everything we allocated
    uiClearCallback(window);
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // deactive MuJoCo
    mj_deactivate();

// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 0;
}
