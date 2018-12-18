#include "mujoco_dyros.h"

void mujoco_ros_connector_init(const mjModel *m, mjData *d)
{
    std::cout << "::::::::::::::::::::: MUJOCO_ROS_CONNECTOR INITIALIZE ::::::::::::::::::::" << std::endl;

    std_msgs::String rst_msg_;
    rst_msg_.data = "RESET";
    sim_command_pub.publish(rst_msg_);
    std::cout << " CONTROLLER RESET COMMAND " << std::endl;

    ros::Rate poll_r(60);
    ros::Time wait_time;
    wait_time = ros::Time::now();

    while ((controller_reset_check) && ((ros::Time::now() - wait_time).toSec() < 1.0))
    {
        ros::spinOnce();
        poll_r.sleep();
    }
    if (!((ros::Time::now() - wait_time).toSec() < 5.0))
    {

        ROS_ERROR("NO RESPONSE FROM CONTROLLER ");
    }

    state_publisher_init(m, d);

    std::cout << " PUBLISHER INIT COMPLETE " << std::endl;
    state_publisher(m, d);

    std::cout << " PUBLISH TEST COMPLETE" << std::endl;

    rst_msg_.data = "INIT";
    sim_command_pub.publish(rst_msg_);
    controller_reset_check = true;

    wait_time = ros::Time::now();
    while ((controller_init_check) && ((ros::Time::now() - wait_time).toSec() < 5.0))
    {
        ros::spinOnce();
        poll_r.sleep();
    }
    controller_init_check = true;

    std::cout << " MUJOCO_ROS_CONNTECTOR INITIALIZE COMPLETE" << std::endl;
}

void state_publisher_init(const mjModel *m, mjData *d)
{

    joint_set_msg_.position.resize(m->nu);
    joint_set_msg_.torque.resize(m->nu);

    command.resize(m->nu);

    for (int i = 0; i < m->nu; i++)
    {
        torque_mj[i] = 0.0;
        command[i] = 0.0;
    }

    joint_state_msg_.name.resize(m->nu + 6);
    joint_state_msg_.position.resize(m->nu + 7);
    joint_state_msg_.velocity.resize(m->nu + 6);
    joint_state_msg_.effort.resize(m->nu + 6);
    for (int i = 0; i < 6; i++)
        joint_state_msg_.effort[i] = 0.0;

    sensor_state_msg_.sensor.resize(m->nsensor + 1);

    //
    for (int i = 0; i < m->nu; i++)
    {
        std::string buffer(m->names + m->name_actuatoradr[i]);
        joint_state_msg_.name[i + 6] = buffer;
    }

    joint_state_msg_.name[0] = "virtual_x";
    joint_state_msg_.name[1] = "virtual_y";
    joint_state_msg_.name[2] = "virtual_z";
    joint_state_msg_.name[3] = "virtual_roll";
    joint_state_msg_.name[4] = "virtual_pitch";
    joint_state_msg_.name[5] = "virtual_yaw";

    for (int i = 0; i < m->nsensor; i++)
    {
        std::string buffer(m->names + m->name_sensoradr[i]);
        sensor_state_msg_.sensor[i].name = buffer;
        sensor_state_msg_.sensor[i].data.resize(m->sensor_dim[i]);
    }

    sensor_state_msg_.sensor[m->nsensor].name = "user information";

    sensor_state_msg_.sensor[m->nsensor].data.resize(2);

    std::cout << "force range " << std::endl;

    for (int i = 0; i < m->nu; i++)
    {
        std::cout << "actuator : " << i << " f1 :   " << m->actuator_ctrlrange[i * 2] << " f2 : " << m->actuator_ctrlrange[i * 2 + 1] << std::endl;
    }
}

void state_publisher(const mjModel *m, mjData *d)
{
    sim_time.data = d->time;

    for (int i = 0; i < m->nu; i++)
    {
        joint_state_msg_.position[i + 6] = d->qpos[i + 7];
        joint_state_msg_.velocity[i + 6] = d->qvel[i + 6];
        joint_state_msg_.effort[i + 6] = d->qacc[i + 6];
        //joint_state_msg_.effort[i + 6] = command[i];
    }

    for (int i = 0; i < 3; i++)
    {
        joint_state_msg_.position[i] = d->qpos[i];
        joint_state_msg_.position[i + 3] = d->qpos[i + 4];
        joint_state_msg_.velocity[i] = d->qvel[i];
        joint_state_msg_.velocity[i + 3] = d->qvel[i + 3];
        joint_state_msg_.effort[i] = d->qacc[i];
        joint_state_msg_.effort[i] = d->qacc[i + 3];
    }

    joint_state_msg_.position[m->nu + 6] = d->qpos[3];
    joint_state_msg_.header.stamp = ros::Time::now();

    for (int i = 0; i < m->nsensor; i++)
    {
        for (int n = 0; n < m->sensor_dim[i]; n++)
        {
            sensor_state_msg_.sensor[i].data[n] = d->sensordata[m->sensor_adr[i] + n];
        }
    }
    sensor_state_msg_.sensor[m->nsensor].data[0] = dif_time;
    sensor_state_msg_.sensor[m->nsensor].data[1] = com_time;

    sensor_state_msg_.header.stamp = ros::Time::now();
    sensor_state_pub.publish(sensor_state_msg_);
    joint_state_pub.publish(joint_state_msg_);
    sim_time_pub.publish(sim_time);
}

void mycontrollerinit()
{
    ros_sim_started = true;
}

void mycontroller(const mjModel *m, mjData *d)
{
    state_publisher(m, d);
    ros::spinOnce();

    for (int i = 0; i < m->nu; i++)
    {
        torque_mj[i] = command[i];
    }
    mju_copy(d->ctrl, torque_mj, m->nu);

    ROS_INFO_COND(showdebug == 1, "MJ_TIME:%10.5f ros:%10.5f dif:%10.5f", d->time, ros_sim_runtime.toSec(), d->time - ros_sim_runtime.toSec());
    ROS_INFO_COND(showdebug == 1, "TEST FOR THERE ");

    if (showdebug == 1)
    {
        std::cout << "command torque " << std::endl;
        for (int i = 0; i < m->nu; i++)
        {

            std::cout << command[i] << std::endl;
        }
    }
}

//---------------------callback functions --------------------------------

void jointset_callback(const mujoco_ros_msgs::JointSetConstPtr &msg)
{

    com_time = ros::Time::now().toSec() - msg->header.stamp.toSec();
    dif_time = d->time - msg->time;

    ROS_INFO_COND(showdebug == 2, "TIME INFORMATION :::: state time : %10.5f , command time : %10.5f, time dif : %10.5f , com time : %10.5f ", msg->time, d->time, dif_time, com_time);

    if ((msg->time) > (d->time))
    {
        ROS_ERROR("JOINT SET COMMAND IS IN FUTURE : current sim time : %10.5f command time : %10.5f", d->time, msg->time);
    }
    else if ((msg->time + 0.01) < (d->time))
    {
        ROS_ERROR("Sim time and Command time error exceeds 0.01 sim time : %10.5f command time : %10.5f", d->time, msg->time);
    }
    else
    {
        //MODE 0 position
        //MODE 1 torque
        if (msg->MODE == 1)
        {
            if (joint_set_msg_.torque.size() == m->nu)
            {
                for (int i = 0; i < m->nu; i++)
                    command[i] = msg->torque[i];
            }
            else
            {
                ROS_ERROR("TORQUE_MODE :::: Actuator Size Not match ");
            }
        }
        else if (msg->MODE == 0)
        {
            if (joint_set_msg_.torque.size() == m->nu)
            {
                for (int i = 0; i < m->nu; i++)
                    command[i] = msg->position[i];
            }
            else
            {
                ROS_ERROR("POSITION_MODE ::::  Actuator Size Not match ");
            }
        }
    }
}

void sim_command_callback(const std_msgs::StringConstPtr &msg)
{
    std::cout << "MSG FROM CONTROLLER " << msg->data << std::endl;

    if (msg->data == "RESET")
    {

        controller_reset_check = false;
        std::cout << "RESET CHECK COMPLETE " << std::endl;
    }
    else if (msg->data == "INIT")
    {
        controller_init_check = false;
        std::cout << "INIT CHECK COMPLETE " << std::endl;
    }
    else if (msg->data == "pause")
    {
        c_pause();
        std::cout << "SIM PAUSED by msg" << std::endl;
    }
    else if (msg->data == "mjreset")
    {
        c_reset();
        std::cout << "SIM RESET by msg" << std::endl;
    }
    else if (msg->data == "mjslowmotion")
    {
        c_slowmotion();
        std::cout << "SIM slowmotion by msg" << std::endl;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *--------------------------------------------------------------------------------------- utility functions ----------------------------------------------------------------------------------------
 *--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

// center and scale view
void autoscale(GLFWwindow *window)
{
    // autoscale
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;

    // set to free camera
    cam.type = mjCAMERA_FREE;
}

// load mjb or xml model
void loadmodel(GLFWwindow *window, const char *filename)
{
    // make sure filename is given
    if (!filename)
        return;

    // load and compile
    char error[1000] = "could not load binary model";
    mjModel *mnew = 0;
    if (strlen(filename) > 4 && !strcmp(filename + strlen(filename) - 4, ".mjb"))
        mnew = mj_loadModel(filename, 0);
    else
        mnew = mj_loadXML(filename, 0, error, 1000);
    if (!mnew)
    {
        printf("%s\n", error);
        return;
    }

    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    d = mj_makeData(m);

    vopt.geomgroup[2] = false; //collision group invisible, since group 2 in model file is collision

    if (keyreset >= 0 && keyreset < m->nkey)
    {
        d->time = m->key_time[keyreset];
        mju_copy(d->qpos, m->key_qpos + keyreset * m->nq, m->nq);
        mju_copy(d->qvel, m->key_qvel + keyreset * m->nv, m->nv);
        mju_copy(d->act, m->key_act + keyreset * m->na, m->na);
    }
    ros_sim_started = true;
    mj_forward(m, d);
    torque_mj = mj_stackAlloc(d, (int)m->nu);

    mujoco_ros_connector_init(m, d);
    // save filename for reload
    strcpy(lastfile, filename);

    // re-create custom context
    mjr_makeContext(m, &con, fontscale);

    // clear perturbation state and keyreset
    pert.active = 0;
    pert.select = 0;
    keyreset = 0;

    // center and scale view, update scene
    autoscale(window);
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // set window title to mode name
    if (window && m->names)
        glfwSetWindowTitle(window, m->names);

    //print model detail data

    std::cout << " MODEL LOADED : DETAIL " << std::endl;
    std::cout << " number of generalized coordinated nq = " << m->nq << std::endl;
    std::cout << "number of degrees of freedom nv = " << m->nv << std::endl;
    std::cout << "number of actuators/controls nu = " << m->nu << std::endl;
    std::cout << "number of joints njnt = " << m->njnt << std::endl;
    std::cout << "number of keyframe = " << m->nkey << std::endl;
    std::cout << "BODY NAMES" << std::endl;
    for (int i = 0; i < m->nbody; i++)
    {
        std::string buffer(m->names + m->name_bodyadr[i]);
        std::cout << "id : " << i << " name : " << buffer << std::endl;
    }

    std::cout << "ACTUATOR NAMES" << std::endl;
    for (int i = 0; i < m->nu; i++)
    {
        std::string buffer(m->names + m->name_actuatoradr[i]);
        std::cout << "id : " << i << "  name : " << buffer << std::endl;
    }
    std::cout << "JOINT NAMES" << std::endl;
    for (int i = 0; i < m->njnt; i++)
    {
        std::string buffer(m->names + m->name_jntadr[i]);
        std::cout << "id : " << i << "  name : " << buffer << std::endl;
    }
    std::cout << "QPOS" << std::endl;
    for (int i = 0; i < m->nq; i++)
    {
        std::cout << d->qpos[i] << "\t";
    }
    std::cout << std::endl
              << "SENSOR NAMES " << std::endl;
    for (int i = 0; i < m->nsensor; i++)
    {
        std::string buffer(m->names + m->name_sensoradr[i]);
        std::cout << "sensor id : " << i << "  name : " << buffer << "  data size : " << m->sensor_dim[i] << std::endl;
    }
}

// timer in milliseconds
mjtNum timer(void)
{
    // save start time
    static double starttm = 0;
    if (starttm == 0)
        starttm = ros::Time::now().toSec();

    // return time since start
    return (mjtNum)(1000 * (ros::Time::now().toSec() - starttm));
}

// clear all times
void cleartimers(mjData *d)
{
    for (int i = 0; i < mjNTIMER; i++)
    {
        d->timer[i].duration = 0;
        d->timer[i].number = 0;
    }
}

//-------------------------------- simulation and rendering -----------------------------

// make option string
void makeoptionstring(const char *name, char key, char *buf)
{
    int i = 0, cnt = 0;

    // copy non-& characters
    while (name[i] && i < 50)
    {
        if (name[i] != '&')
            buf[cnt++] = name[i];

        i++;
    }

    // finish
    buf[cnt] = ' ';
    buf[cnt + 1] = '(';
    buf[cnt + 2] = key;
    buf[cnt + 3] = ')';
    buf[cnt + 4] = 0;
}

// advance simulation
void simulation(void)
{
    // no model
    if (!m)
        return;

    // clear timers
    cleartimers(d);

    // paused
    if (paused)
    {
        ros::Time ref_time;
        ref_time = ros::Time::now();

        while ((ros::Time::now() - ref_time).toSec() < 1.0 / (refreshrate + 2))
        {

            ros::spinOnce();
            // apply pose perturbations, run mj_forward
            if (pert.active)
            {
                mjv_applyPerturbPose(m, d, &pert, 1); // move mocap and dynamic bodies
                mj_forward(m, d);
            }
            if ((d->time > 0.0) && ros_sim_started)
                break;
        }
    }

    // running
    else
    {
        // slow motion factor: 10x
        mjtNum factor = (slowmotion ? 10 : 1);

        //ROS_INFO("factor : %f", factor);

        // advance effective simulation time by 1/refreshrate
        mjtNum startsimtm = d->time;
        float r_fd = 5 * (60 - current_refresh_rate);

        if (r_fd > 10)
            r_fd = 30;
        else if (r_fd < 0)
            r_fd = 0;

        ros::Rate r(60 + 8);
        if (pert.select > 0)
        {
            std::string buffer(m->names + m->name_bodyadr[pert.select]);
            //mjtNum res[3];
            //mju_rotVecMat(res,pert.localpos,d->xmat[pert.select*9]);

            Eigen::Vector3d euler;
            tf::Quaternion q_(d->xquat[pert.select * 4 + 1], d->xquat[pert.select * 4 + 2], d->xquat[pert.select * 4 + 3], d->xquat[pert.select * 4]);
            tf::Matrix3x3 m_(q_);
            tf::Vector3 global_p_(d->xpos[pert.select * 3], d->xpos[pert.select * 3 + 1], d->xpos[pert.select * 3 + 2]);
            tf::Vector3 local_p_(pert.localpos[0], pert.localpos[1], pert.localpos[2]);

            tf::Vector3 r_ = global_p_ + m_ * local_p_;
            m_.getRPY(euler(0), euler(1), euler(2));

            double radd = 180.0 / 3.141592;

            ROS_INFO("pert.select : %d, name : %s \n\t body pos : %10.7f, %10.7f, %10.7f, \n\tlocal pos : %10.7f, %10.7f, %10.7f \n\tpoint pos : %10.7f, %10.7f, %10.7f\n\tEuler angle : %9.4f, %9.4f, %9.4f",
                     pert.select, buffer.c_str(),
                     global_p_[0], global_p_[1], global_p_[2],
                     local_p_[0], local_p_[1], local_p_[2],
                     r_[0], r_[1], r_[2],
                     radd * euler(0), radd * euler(1), radd * euler(2));
        }

        while ((d->time - startsimtm) * factor < 1.0 / refreshrate)
        {

            // clear old perturbations, apply new
            mju_zero(d->xfrc_applied, 6 * m->nbody);
            if (pert.select > 0)
            {

                mjv_applyPerturbPose(m, d, &pert, 0); // move mocap bodies only
                mjv_applyPerturbForce(m, d, &pert);
            }

            //ros_timer

            // run mj_step and count
            mj_step(m, d);

            if (ros_sim_started == true)
            {
                ros_sim_starttm = ros::Time::now();
                ros_sim_started = false;
            }
            ros_sim_runtime = ros::Time::now() - ros_sim_starttm;

            if (!slowmotion)
            {
                if (d->time > ros_sim_runtime.toSec())
                {
                    double timddif = d->time - ros_sim_runtime.toSec();
                    ros::Duration(timddif).sleep();
                    timesync_count++;
                    timesync_mean = (timesync_mean * timesync_count + timddif) / (timesync_count + 1);
                    ROS_INFO_COND(showdebug == 1, "TIMESYNC ACTIVE :: %8.5f ms  MEAN : %8.5f ms", timddif * 1000, timesync_mean * 1000);
                }
            }
            //ROS_INFO("TIME_COMPARE : %8.4f %8.4f", d->time-startsimtm, ros::Time::now().toSec()-rostm_);

            // break on reset
            if (d->time < startsimtm)
                break;
        }
        if (slowmotion)
        {
            r.sleep();
        }
    }
}

//detpth output for vision simulation, but not working
void render_depth(GLFWwindow *main_window, GLFWwindow *sub_window)
{

    mjrRect rect = {0, 0, 0, 0};
    glfwGetFramebufferSize(sub_window, &rect.width, &rect.height);

    ROS_INFO_COND(showdebug == 1, "SHOWFIXCAM");
    cam2.fixedcamid = 0;
    cam2.type = mjCAMERA_FIXED;

    mjv_updateScene(m, d, &vopt, NULL, &cam2, mjCAT_ALL, &scn2);

    // render
    mjr_render(rect, &scn2, &con2);

    /*static double lastrendertm = 0;
  mjrRect rect_sub={0,0,0,0};

  // get the depth buffer
  mjr_readPixels(NULL, depth_buffer, rect, &con);

  // convert to RGB, subsample by 4
  for( int r=0; r<rect.height; r+=4 )
      for( int c=0; c<rect.width; c+=4 )      {
          // get subsampled address
          int adr = (r/4)*(rect.width/4) + c/4;
          // assign rgb
          depth_rgb[3*adr] = depth_rgb[3*adr+1] = depth_rgb[3*adr+2] =
              (unsigned char)((1.0f-depth_buffer[r*rect.width+c])*255.0f);
      }

  // show in bottom-right corner, offset for profiler and sensor
  mjrRect bottomright = {
      rect.left+rect.width/4,
      rect.bottom+rect.height/4,
      rect.width/4,
      rect.height/4
  };
*/
    //mjr_drawPixels(depth_rgb, NULL, bottomright, &con);
    glfwSwapBuffers(sub_window);
    glfwMakeContextCurrent(main_window);
}

// render
void render(GLFWwindow *window)
{
    // past data for FPS calculation
    static double lastrendertm = 0;

    // get current framebuffer rectangle
    mjrRect rect = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &rect.width, &rect.height);
    mjrRect smallrect = rect;

    // reduce rectangle when profiler is on
    if (showprofiler)
        smallrect.width = rect.width - rect.width / 5;

    // no model: empty screen
    if (!m)
    {
        mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, "Drag-and-drop model file here", 0, &con);

        // swap buffers
        glfwSwapBuffers(window);
        //glFinish();
        return;
    }

    // advance simulation
    simulation();

    // update simulation statistics
    //if( !paused )
    // {
    // camera string
    char camstr[20];
    if (cam.type == mjCAMERA_FREE)
        strcpy(camstr, "Free");
    else if (cam.type == mjCAMERA_TRACKING)
        strcpy(camstr, "Tracking");
    else
        sprintf(camstr, "Fixed %d", cam.fixedcamid);

    // keyreset string
    char keyresetstr[20];
    if (keyreset < 0)
        strcpy(keyresetstr, "qpos0");
    else
        sprintf(keyresetstr, "Key %d", keyreset);
    char keydebug[20];
    if (showdebug)
        strcpy(keydebug, "TRUE");
    else
        strcpy(keydebug, "FALSE");

    // solver error
    mjtNum solerr = 0;
    if (d->solver_iter)
    {
        int ind = mjMIN(d->solver_iter - 1, mjNSOLVER - 1);
        solerr = mju_min(d->solver[ind].improvement, d->solver[ind].gradient);
        if (solerr == 0)
            solerr = mju_max(d->solver[ind].improvement, d->solver[ind].gradient);
    }
    solerr = mju_log10(mju_max(mjMINVAL, solerr));

    static std::vector<double> rf_rate(10, 59.0);
    rf_rate.erase(rf_rate.begin());
    rf_rate.push_back(1.0 / (glfwGetTime() - lastrendertm));

    current_refresh_rate = 0.0;
    for (int i = 0; i < 10; i++)
        current_refresh_rate += rf_rate[i];
    current_refresh_rate = current_refresh_rate / 10;
    if (current_refresh_rate > 200)
    {
        ROS_WARN("%6.2f FPS limit broken !!!! ", current_refresh_rate);
    }

    // status
    sprintf(status, "%-20.3f\n%-20.3f\n%d  (%d con)\n%.3f\n%.0f\n%.2f\n%.1f  (%d it)\n%.1f %.1f\n%s\n%s\n%s\n%s\n%s",
            d->time,
            ros_sim_runtime.toSec(),
            d->nefc,
            d->ncon,
            d->timer[mjTIMER_STEP].duration / mjMAX(1, d->timer[mjTIMER_STEP].number),
            current_refresh_rate,
            d->energy[0] + d->energy[1],
            solerr,
            d->solver_iter,
            mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[0])),
            mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[1])),
            camstr,
            mjFRAMESTRING[vopt.frame],
            mjLABELSTRING[vopt.label],
            keyresetstr,
            keydebug);
    // status
    sprintf(status_brief, "%-20.3f\n%-20.3f\n%d  (%d con)\n%.3f\n%.0f",
            d->time,
            ros_sim_runtime.toSec(),
            d->nefc,
            d->ncon,
            d->timer[mjTIMER_STEP].duration / mjMAX(1, d->timer[mjTIMER_STEP].number),
            current_refresh_rate);
    //}

    // FPS timing satistics
    lastrendertm = glfwGetTime();

    // update scene
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);
    mjr_render(rect, &scn, &con);

    // show depth map
    if (showdepth)
    {
        // get the depth buffer
        mjr_readPixels(NULL, depth_buffer, rect, &con);

        // convert to RGB, subsample by 4
        for (int r = 0; r < rect.height; r += 4)
            for (int c = 0; c < rect.width; c += 4)
            {
                // get subsampled address
                int adr = (r / 4) * (rect.width / 4) + c / 4;

                // assign rgb
                depth_rgb[3 * adr] = depth_rgb[3 * adr + 1] = depth_rgb[3 * adr + 2] =
                    (unsigned char)((1.0f - depth_buffer[r * rect.width + c]) * 255.0f);
            }

        // show in bottom-right corner, offset for profiler and sensor
        mjrRect bottomright = {
            smallrect.left + smallrect.width - rect.width / 4,
            smallrect.bottom,
            rect.width / 4,
            rect.height / 4};
        if (showsensor)
            bottomright.left -= smallrect.width / 4;
        mjr_drawPixels(depth_rgb, NULL, bottomright, &con);
    }
    // show overlays
    if (showhelp == 1)
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, smallrect, "Help  ", "F1  ", &con);
    else if (showhelp == 2)
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, smallrect, help_title, help_content, &con);

    // show info
    if (showinfo == 2)
    {
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect,
                    "Time\nRTime\nSize\nCPU\nFPS\nEnergy\nSolver\nFwdInv\nCamera\nFrame\nLabel\nReset\nDebug", status, &con);
    }
    else if (showinfo == 1)
    {

        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect,
                    "Time\nRTime\nSize\nCPU\nFPS", status_brief, &con);
    }

    mjrRect bottomcenter = {
        smallrect.left + smallrect.width / 2,
        smallrect.bottom,
        smallrect.width,
        smallrect.height};

    if (paused)
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, bottomcenter, " PAUSED", 0, &con);

    // show options
    if (showoption)
    {
        int i;
        char buf[100];

        // fill titles on first pass
        if (!opt_title[0])
        {
            for (i = 0; i < mjNRNDFLAG; i++)
            {
                makeoptionstring(mjRNDSTRING[i][0], mjRNDSTRING[i][2][0], buf);
                strcat(opt_title, buf);
                strcat(opt_title, "\n");
            }
            for (i = 0; i < mjNVISFLAG; i++)
            {
                makeoptionstring(mjVISSTRING[i][0], mjVISSTRING[i][2][0], buf);
                strcat(opt_title, buf);
                if (i < mjNVISFLAG - 1)
                    strcat(opt_title, "\n");
            }
        }

        // fill content
        opt_content[0] = 0;
        for (i = 0; i < mjNRNDFLAG; i++)
        {
            strcat(opt_content, scn.flags[i] ? " + " : "   ");
            strcat(opt_content, "\n");
        }
        for (i = 0; i < mjNVISFLAG; i++)
        {
            strcat(opt_content, vopt.flags[i] ? " + " : "   ");
            if (i < mjNVISFLAG - 1)
                strcat(opt_content, "\n");
        }

        // show
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, smallrect, opt_title, opt_content, &con);
    }

    // show profiler
    if (showprofiler)
    {
        if (!paused)
            profilerupdate();
        profilershow(rect);
    }

    // show sensor
    if (showsensor)
    {
        if (!paused)
            sensorupdate();
        sensorshow(smallrect);
    }

    // swap buffers
    glfwSwapBuffers(window);
}

//-------------------------------- profiler and sensor ----------------------------------

// init profiler
void profilerinit(void)
{
    int i, n;

    // set figures to default
    mjv_defaultFigure(&figconstraint);
    mjv_defaultFigure(&figcost);
    mjv_defaultFigure(&figtimer);
    mjv_defaultFigure(&figsize);

    // titles
    strcpy(figconstraint.title, "Counts");
    strcpy(figcost.title, "Convergence (log 10)");
    strcpy(figsize.title, "Dimensions");
    strcpy(figtimer.title, "CPU time (msec)");

    // x-labels
    strcpy(figconstraint.xlabel, "Solver iteration");
    strcpy(figcost.xlabel, "Solver iteration");
    strcpy(figsize.xlabel, "Video frame");
    strcpy(figtimer.xlabel, "Video frame");

    // y-tick nubmer formats
    strcpy(figconstraint.yformat, "%.0f");
    strcpy(figcost.yformat, "%.1f");
    strcpy(figsize.yformat, "%.0f");
    strcpy(figtimer.yformat, "%.2f");

    // colors
    figconstraint.figurergba[0] = 0.1f;
    figcost.figurergba[2] = 0.2f;
    figsize.figurergba[0] = 0.1f;
    figtimer.figurergba[2] = 0.2f;

    // legends
    strcpy(figconstraint.linename[0], "total");
    strcpy(figconstraint.linename[1], "active");
    strcpy(figconstraint.linename[2], "changed");
    strcpy(figconstraint.linename[3], "evals");
    strcpy(figconstraint.linename[4], "updates");
    strcpy(figcost.linename[0], "improvement");
    strcpy(figcost.linename[1], "gradient");
    strcpy(figcost.linename[2], "lineslope");
    strcpy(figsize.linename[0], "dof");
    strcpy(figsize.linename[1], "body");
    strcpy(figsize.linename[2], "constraint");
    strcpy(figsize.linename[3], "sqrt(nnz)");
    strcpy(figsize.linename[4], "contact");
    strcpy(figsize.linename[5], "iteration");
    strcpy(figtimer.linename[0], "total");
    strcpy(figtimer.linename[1], "collision");
    strcpy(figtimer.linename[2], "prepare");
    strcpy(figtimer.linename[3], "solve");
    strcpy(figtimer.linename[4], "other");

    // grid sizes
    figconstraint.gridsize[0] = 5;
    figconstraint.gridsize[1] = 5;
    figcost.gridsize[0] = 5;
    figcost.gridsize[1] = 5;
    figsize.gridsize[0] = 3;
    figsize.gridsize[1] = 5;
    figtimer.gridsize[0] = 3;
    figtimer.gridsize[1] = 5;

    // minimum ranges
    figconstraint.range[0][0] = 0;
    figconstraint.range[0][1] = 20;
    figconstraint.range[1][0] = 0;
    figconstraint.range[1][1] = 80;
    figcost.range[0][0] = 0;
    figcost.range[0][1] = 20;
    figcost.range[1][0] = -15;
    figcost.range[1][1] = 5;
    figsize.range[0][0] = -200;
    figsize.range[0][1] = 0;
    figsize.range[1][0] = 0;
    figsize.range[1][1] = 100;
    figtimer.range[0][0] = -200;
    figtimer.range[0][1] = 0;
    figtimer.range[1][0] = 0;
    figtimer.range[1][1] = 0.4f;

    // init x axis on history figures (do not show yet)
    for (n = 0; n < 6; n++)
        for (i = 0; i < mjMAXLINEPNT; i++)
        {
            figtimer.linedata[n][2 * i] = (float)-i;
            figsize.linedata[n][2 * i] = (float)-i;
        }
}

// show profiler
void profilerupdate(void)
{
    int i, n;

    // update constraint figure
    figconstraint.linepnt[0] = mjMIN(mjMIN(d->solver_iter, mjNSOLVER), mjMAXLINEPNT);
    for (i = 1; i < 5; i++)
        figconstraint.linepnt[i] = figconstraint.linepnt[0];
    if (m->opt.solver == mjSOL_PGS)
    {
        figconstraint.linepnt[3] = 0;
        figconstraint.linepnt[4] = 0;
    }
    if (m->opt.solver == mjSOL_CG)
        figconstraint.linepnt[4] = 0;
    for (i = 0; i < figconstraint.linepnt[0]; i++)
    {
        // x
        figconstraint.linedata[0][2 * i] = (float)i;
        figconstraint.linedata[1][2 * i] = (float)i;
        figconstraint.linedata[2][2 * i] = (float)i;
        figconstraint.linedata[3][2 * i] = (float)i;
        figconstraint.linedata[4][2 * i] = (float)i;

        // y
        figconstraint.linedata[0][2 * i + 1] = (float)d->nefc;
        figconstraint.linedata[1][2 * i + 1] = (float)d->solver[i].nactive;
        figconstraint.linedata[2][2 * i + 1] = (float)d->solver[i].nchange;
        figconstraint.linedata[3][2 * i + 1] = (float)d->solver[i].neval;
        figconstraint.linedata[4][2 * i + 1] = (float)d->solver[i].nupdate;
    }

    // update cost figure
    figcost.linepnt[0] = mjMIN(mjMIN(d->solver_iter, mjNSOLVER), mjMAXLINEPNT);
    for (i = 1; i < 3; i++)
        figcost.linepnt[i] = figcost.linepnt[0];
    if (m->opt.solver == mjSOL_PGS)
    {
        figcost.linepnt[1] = 0;
        figcost.linepnt[2] = 0;
    }

    for (i = 0; i < figcost.linepnt[0]; i++)
    {
        // x
        figcost.linedata[0][2 * i] = (float)i;
        figcost.linedata[1][2 * i] = (float)i;
        figcost.linedata[2][2 * i] = (float)i;

        // y
        figcost.linedata[0][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, d->solver[i].improvement));
        figcost.linedata[1][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, d->solver[i].gradient));
        figcost.linedata[2][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, d->solver[i].lineslope));
    }

    // get timers: total, collision, prepare, solve, other
    int itotal = (d->timer[mjTIMER_STEP].duration > d->timer[mjTIMER_FORWARD].duration ? mjTIMER_STEP : mjTIMER_FORWARD);
    float tdata[5] = {
        (float)(d->timer[itotal].duration / mjMAX(1, d->timer[itotal].number)),
        (float)(d->timer[mjTIMER_POS_COLLISION].duration / mjMAX(1, d->timer[mjTIMER_POS_COLLISION].number)),
        (float)(d->timer[mjTIMER_POS_MAKE].duration / mjMAX(1, d->timer[mjTIMER_POS_MAKE].number)) +
            (float)(d->timer[mjTIMER_POS_PROJECT].duration / mjMAX(1, d->timer[mjTIMER_POS_PROJECT].number)),
        (float)(d->timer[mjTIMER_CONSTRAINT].duration / mjMAX(1, d->timer[mjTIMER_CONSTRAINT].number)),
        0};
    tdata[4] = tdata[0] - tdata[1] - tdata[2] - tdata[3];

    // update figtimer
    int pnt = mjMIN(201, figtimer.linepnt[0] + 1);
    for (n = 0; n < 5; n++)
    {
        // shift data
        for (i = pnt - 1; i > 0; i--)
            figtimer.linedata[n][2 * i + 1] = figtimer.linedata[n][2 * i - 1];

        // assign new
        figtimer.linepnt[n] = pnt;
        figtimer.linedata[n][1] = tdata[n];
    }

    // get sizes: nv, nbody, nefc, sqrt(nnz), ncont, iter
    float sdata[6] = {
        (float)m->nv,
        (float)m->nbody,
        (float)d->nefc,
        (float)mju_sqrt((mjtNum)d->solver_nnz),
        (float)d->ncon,
        (float)d->solver_iter};

    // update figsize
    pnt = mjMIN(201, figsize.linepnt[0] + 1);
    for (n = 0; n < 6; n++)
    {
        // shift data
        for (i = pnt - 1; i > 0; i--)
            figsize.linedata[n][2 * i + 1] = figsize.linedata[n][2 * i - 1];

        // assign new
        figsize.linepnt[n] = pnt;
        figsize.linedata[n][1] = sdata[n];
    }
}

// show profiler
void profilershow(mjrRect rect)
{
    mjrRect viewport = {rect.width - rect.width / 5, rect.bottom, rect.width / 5, rect.height / 4};
    mjr_figure(viewport, &figtimer, &con);
    viewport.bottom += rect.height / 4;
    mjr_figure(viewport, &figsize, &con);
    viewport.bottom += rect.height / 4;
    mjr_figure(viewport, &figcost, &con);
    viewport.bottom += rect.height / 4;
    mjr_figure(viewport, &figconstraint, &con);
}

// init sensor figure
void sensorinit(void)
{
    // set figure to default
    mjv_defaultFigure(&figsensor);

    // set flags
    figsensor.flg_extend = 1;
    figsensor.flg_barplot = 1;

    // title
    strcpy(figsensor.title, "Sensor data");

    // y-tick nubmer format
    strcpy(figsensor.yformat, "%.0f");

    // grid size
    figsensor.gridsize[0] = 2;
    figsensor.gridsize[1] = 3;

    // minimum range
    figsensor.range[0][0] = 0;
    figsensor.range[0][1] = 0;
    figsensor.range[1][0] = -1;
    figsensor.range[1][1] = 1;
}

// update sensor figure
void sensorupdate(void)
{
    static const int maxline = 10;

    // clear linepnt
    for (int i = 0; i < maxline; i++)
        figsensor.linepnt[i] = 0;

    // start with line 0
    int lineid = 0;

    // loop over sensors
    for (int n = 0; n < m->nsensor; n++)
    {
        // go to next line if type is different
        if (n > 0 && m->sensor_type[n] != m->sensor_type[n - 1])
            lineid = mjMIN(lineid + 1, maxline - 1);

        // get info about this sensor
        mjtNum cutoff = (m->sensor_cutoff[n] > 0 ? m->sensor_cutoff[n] : 1);
        int adr = m->sensor_adr[n];
        int dim = m->sensor_dim[n];

        // data pointer in line
        int p = figsensor.linepnt[lineid];

        // fill in data for this sensor
        for (int i = 0; i < dim; i++)
        {
            // check size
            if ((p + 2 * i) >= mjMAXLINEPNT / 2)
                break;

            // x
            figsensor.linedata[lineid][2 * p + 4 * i] = (float)(adr + i);
            figsensor.linedata[lineid][2 * p + 4 * i + 2] = (float)(adr + i);

            // y
            figsensor.linedata[lineid][2 * p + 4 * i + 1] = 0;
            figsensor.linedata[lineid][2 * p + 4 * i + 3] = (float)(d->sensordata[adr + i] / cutoff);
        }

        // update linepnt
        figsensor.linepnt[lineid] = mjMIN(mjMAXLINEPNT - 1,
                                          figsensor.linepnt[lineid] + 2 * dim);
    }
}

// show sensor figure
void sensorshow(mjrRect rect)
{
    // render figure on the right
    mjrRect viewport = {rect.width - rect.width / 4, rect.bottom, rect.width / 4, rect.height / 3};
    mjr_figure(viewport, &figsensor, &con);
}

void c_pause()
{
    paused = !paused;
    if (paused)
    {
        ros_time_paused_starttm = ros::Time::now();
        ROS_INFO_COND(showdebug == 1, " ---- SIMULATION PAUSED ----");
    }
    else
    {
        ros_time_paused_stoptm = ros::Time::now();
        ROS_INFO_COND(showdebug == 1, " ---- SIMULATION RESTARTED ----");
        if (!slowmotion)
            ros_sim_starttm = ros_sim_starttm + (ros_time_paused_stoptm - ros_time_paused_starttm);
    }
}

void c_slowmotion()
{
    slowmotion = !slowmotion;

    if (!slowmotion)
    {
        ROS_INFO_COND(showdebug == 1, " ---- SLOW MOTION DISABLED ----");
        ros_sim_starttm = ros::Time::now() - ros::Duration(d->time);
    }
    else
        ROS_INFO_COND(showdebug == 1, " ---- SLOW MOTION ENABLED ----");
}

void c_reset()
{
    mj_resetData(m, d);
    if (keyreset >= 0 && keyreset < m->nkey)
    {
        d->time = m->key_time[keyreset];
        mju_copy(d->qpos, m->key_qpos + keyreset * m->nq, m->nq);
        mju_copy(d->qvel, m->key_qvel + keyreset * m->nv, m->nv);
        mju_copy(d->act, m->key_act + keyreset * m->na, m->na);
    }
    ros_sim_started = true;
    controller_reset_check = true;
    controller_init_check = true;
    mj_forward(m, d);
    profilerupdate();
    sensorupdate();
    mujoco_ros_connector_init(m, d);
}

//--------------------------------- GLFW callbacks --------------------------------------

// keyboard
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    int n;

    // require model
    if (!m)
        return;

    // do not act on release
    if (act == GLFW_RELEASE)
        return;

    switch (key)
    {
    case GLFW_KEY_F1: // help

        showhelp++;
        if (showhelp > 2)
            showhelp = 0;
        break;

    case GLFW_KEY_F2: // option
        showoption = !showoption;
        break;

    case GLFW_KEY_F3: // info

        showinfo++;
        if (showinfo > 2)
        {
            showinfo = 0;
        }
        break;

    case GLFW_KEY_F4: // depth
        showdepth = !showdepth;
        break;

    case GLFW_KEY_F5: // toggle full screen
        showfullscreen = !showfullscreen;
        if (showfullscreen)
            glfwMaximizeWindow(window);
        else
            glfwRestoreWindow(window);
        break;

    case GLFW_KEY_F6: // stereo
        scn.stereo = (scn.stereo == mjSTEREO_NONE ? mjSTEREO_QUADBUFFERED : mjSTEREO_NONE);
        break;

    case GLFW_KEY_F7: // sensor figure
        showsensor = !showsensor;
        break;

    case GLFW_KEY_F8: // profiler
        showprofiler = !showprofiler;
        break;

    case GLFW_KEY_F9: // profiler
        showdebug++;
        if (showdebug > 2)
            showdebug = 0;
        break;

    case GLFW_KEY_F10: // showfixcam
        showfixcam = !showfixcam;
        break;

    case GLFW_KEY_ENTER: // slow motion
        c_slowmotion();
        break;

    case GLFW_KEY_SPACE: // pause
        c_pause();
        break;

    case GLFW_KEY_PAGE_UP:   // previous keyreset
    case GLFW_KEY_PAGE_DOWN: // next keyreset
        if (key == GLFW_KEY_PAGE_UP)
            keyreset = mjMAX(-1, keyreset - 1);
        else
            keyreset = mjMIN(m->nkey - 1, keyreset + 1);

        // continue with reset

    case GLFW_KEY_BACKSPACE: // reset

        ROS_INFO("RESET BY BACKSPACE");
        c_reset();
        break;

    case GLFW_KEY_RIGHT: // step forward
        if (paused)
        {
            mj_step(m, d);
            profilerupdate();
            sensorupdate();
        }
        break;

    case GLFW_KEY_LEFT: // step back
        if (paused)
        {
            m->opt.timestep = -m->opt.timestep;
            cleartimers(d);
            mj_step(m, d);
            m->opt.timestep = -m->opt.timestep;
            profilerupdate();
            sensorupdate();
        }
        break;

    case GLFW_KEY_DOWN: // step forward 100
        if (paused)
        {
            cleartimers(d);
            for (n = 0; n < 100; n++)
                mj_step(m, d);
            profilerupdate();
            sensorupdate();
        }
        break;

    case GLFW_KEY_UP: // step back 100
        if (paused)
        {
            m->opt.timestep = -m->opt.timestep;
            cleartimers(d);
            for (n = 0; n < 100; n++)
                mj_step(m, d);
            m->opt.timestep = -m->opt.timestep;
            profilerupdate();
            sensorupdate();
        }
        break;

    case GLFW_KEY_ESCAPE: // free camera
        cam.type = mjCAMERA_FREE;
        break;

    case '=': // bigger font
        if (fontscale < 200)
        {
            fontscale += 50;
            mjr_makeContext(m, &con, fontscale);
        }
        break;

    case '-': // smaller font
        if (fontscale > 100)
        {
            fontscale -= 50;
            mjr_makeContext(m, &con, fontscale);
        }
        break;

    case '[': // previous fixed camera or free
        if (m->ncam && cam.type == mjCAMERA_FIXED)
        {
            if (cam.fixedcamid > 0)
                cam.fixedcamid--;
            else
                cam.type = mjCAMERA_FREE;
        }
        break;

    case ']': // next fixed camera
        if (m->ncam)
        {
            if (cam.type != mjCAMERA_FIXED)
            {
                cam.type = mjCAMERA_FIXED;
                cam.fixedcamid = 0;
            }
            else if (cam.fixedcamid < m->ncam - 1)
                cam.fixedcamid++;
        }
        break;

    case ';': // cycle over frame rendering modes
        vopt.frame = mjMAX(0, vopt.frame - 1);
        break;

    case '\'': // cycle over frame rendering modes
        vopt.frame = mjMIN(mjNFRAME - 1, vopt.frame + 1);
        break;

    case '.': // cycle over label rendering modes
        vopt.label = mjMAX(0, vopt.label - 1);
        break;

    case '/': // cycle over label rendering modes
        vopt.label = mjMIN(mjNLABEL - 1, vopt.label + 1);
        break;

    default: // toggle flag
        // control keys
        if (mods & GLFW_MOD_CONTROL)
        {
            if (key == GLFW_KEY_A)
                autoscale(window);
            else if (key == GLFW_KEY_L && lastfile[0])
                loadmodel(window, lastfile);

            break;
        }

        // toggle visualization flag
        for (int i = 0; i < mjNVISFLAG; i++)
            if (key == mjVISSTRING[i][2][0])
                vopt.flags[i] = !vopt.flags[i];

        // toggle rendering flag
        for (int i = 0; i < mjNRNDFLAG; i++)
            if (key == mjRNDSTRING[i][2][0])
                scn.flags[i] = !scn.flags[i];

        // toggle geom/site group
        for (int i = 0; i < mjNGROUP; i++)
            if (key == i + '0')
            {
                if (mods & GLFW_MOD_SHIFT)
                    vopt.sitegroup[i] = !vopt.sitegroup[i];
                else
                    vopt.geomgroup[i] = !vopt.geomgroup[i];
            }
    }
}

// mouse button
void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // past data for double-click detection
    static int lastbutton = 0;
    static double lastclicktm = 0;

    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // Alt: swap left and right
    if ((mods & GLFW_MOD_ALT))
    {
        bool tmp = button_left;
        button_left = button_right;
        button_right = tmp;

        if (button == GLFW_MOUSE_BUTTON_LEFT)
            button = GLFW_MOUSE_BUTTON_RIGHT;
        else if (button == GLFW_MOUSE_BUTTON_RIGHT)
            button = GLFW_MOUSE_BUTTON_LEFT;
    }

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);

    // require model
    if (!m)
        return;

    // set perturbation
    int newperturb = 0;
    if (act == GLFW_PRESS && (mods & GLFW_MOD_CONTROL) && pert.select > 0)
    {
        // right: translate;  left: rotate
        if (button_right)
            newperturb = mjPERT_TRANSLATE;
        else if (button_left)
            newperturb = mjPERT_ROTATE;

        // perturbation onset: reset reference
        if (newperturb && !pert.active)
            mjv_initPerturb(m, d, &scn, &pert);
    }
    pert.active = newperturb;

    // detect double-click (250 msec)
    if (act == GLFW_PRESS && ros::Time::now().toSec() - lastclicktm < 0.25 && button == lastbutton)
    {
        // determine selection mode
        int selmode;
        if (button == GLFW_MOUSE_BUTTON_LEFT)
            selmode = 1;
        else if (mods & GLFW_MOD_CONTROL)
            selmode = 3;
        else
            selmode = 2;

        // get current window size
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        // find geom and 3D click point, get corresponding body
        mjtNum selpnt[3];
        int selgeom = mjv_select(m, d, &vopt,
                                 (mjtNum)width / (mjtNum)height,
                                 (mjtNum)lastx / (mjtNum)width,
                                 (mjtNum)(height - lasty) / (mjtNum)height,
                                 &scn, selpnt);
        int selbody = (selgeom >= 0 ? m->geom_bodyid[selgeom] : 0);

        // set lookat point, start tracking is requested
        if (selmode == 2 || selmode == 3)
        {
            // copy selpnt if geom clicked
            if (selgeom >= 0)
                mju_copy3(cam.lookat, selpnt);

            // switch to tracking camera
            if (selmode == 3 && selbody)
            {
                cam.type = mjCAMERA_TRACKING;
                cam.trackbodyid = selbody;
                cam.fixedcamid = -1;
            }
        }

        // set body selection
        else
        {
            if (selbody)
            {
                // record selection
                pert.select = selbody;

                // compute localpos
                mjtNum tmp[3];
                mju_sub3(tmp, selpnt, d->xpos + 3 * pert.select);
                mju_mulMatTVec(pert.localpos, d->xmat + 9 * pert.select, tmp, 3, 3);
            }
            else
                pert.select = 0;
        }

        // stop perturbation on select
        pert.active = 0;
    }

    // save info
    if (act == GLFW_PRESS)
    {
        lastbutton = button;
        lastclicktm = ros::Time::now().toSec();
    }
}

// mouse move
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // require model
    if (!m)
        return;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move perturb or camera
    if (pert.active)
        mjv_movePerturb(m, d, action, dx / height, dy / height, &scn, &pert);
    else
        mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // require model
    if (!m)
        return;

    // scroll: emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// drop
void drop(GLFWwindow *window, int count, const char **paths)
{
    // make sure list is non-empty
    if (count > 0)
        loadmodel(window, paths[0]);
}
