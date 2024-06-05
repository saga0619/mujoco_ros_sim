#include "mjros.h"
#include <algorithm>
// custum function

void c_reset()
{
    ros_sim_started = false;
    mj_resetData(m, d);
    int i = settings.key;
    d->time = m->key_time[i];
    mju_copy(d->qpos, m->key_qpos + i * m->nq, m->nq);
    mju_copy(d->qvel, m->key_qvel + i * m->nv, m->nv);
    mju_copy(d->act, m->key_act + i * m->na, m->na);
    if (m->actuator_biastype[0])
    {
        mju_copy(d->ctrl, m->key_qpos + 7 + i * m->nq, m->nu);
    }

    ros_sim_started = true;
    controller_reset_check = true;
    controller_init_check = true;
    cmd_rcv = false;

    sim_time_now_ros = ros::Duration(d->time);

    mujoco_ros_connector_init();
    mj_forward(m, d);

    sim_time_ros = ros::Duration(d->time);
    sim_time_run = ros::Time::now();

    profilerupdate();
    sensorupdate();
    updatesettings();
    if (use_shm)
    {
#ifdef COMPILE_SHAREDMEMORY
        mj_shm_->statusCount = 0;
#else
        std::cout << "WARNING : SHM_NOT_COMPILED " << std::endl;
#endif
    }
}

//---------------------callback functions --------------------------------

void jointset_callback(const mujoco_ros_msgs::JointSetConstPtr &msg)
{

    com_time = ros::Time::now().toSec() - msg->header.stamp.toSec();
    dif_time = d->time - msg->time;
    cmd_rcv = true;

    sim_cons_time = dif_time;

    ROS_INFO_COND(settings.debug, "TIME INFORMATION :::: state time : %10.5f , command time : %10.5f, time dif : %10.5f , com time : %10.5f ", msg->time, d->time, dif_time, com_time);

    if ((msg->time) > (d->time))
    {
        ROS_ERROR("JOINT SET COMMAND IS IN FUTURE : current sim time : %10.5f command time : %10.5f", d->time, msg->time);
        cmd_rcv = false;
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
        //c_pause();
        settings.run = !settings.run;
        std::cout << "SIM PAUSED by msg" << std::endl;
    }
    else if (msg->data == "mjreset")
    {
        reset_request = true;
        std::cout << "SIM RESET by msg" << std::endl;
    }
    else if (msg->data == "mjslowmotion")
    {
        //c_slowmotion();
        std::cout << "SIM slowmotion by msg" << std::endl;
    }
}

void force_apply_callback(const mujoco_ros_msgs::applyforce &msg)
{

    applied_ext_force_[0] = msg.wrench.force.x;
    applied_ext_force_[1] = msg.wrench.force.y;
    applied_ext_force_[2] = msg.wrench.force.z;
    applied_ext_force_[3] = msg.wrench.torque.x;
    applied_ext_force_[4] = msg.wrench.torque.y;
    applied_ext_force_[5] = msg.wrench.torque.z;

    force_appiedd_link_idx_ = msg.link_idx;

    ext_force_applied_ = true;
}

void rosPollEvents()
{
    if (reset_request)
    {
        c_reset();
        reset_request = false;
    }
}

void state_publisher_init()
{

    cmd_rcv = false;

    joint_set_msg_.position.resize(m->nu);
    joint_set_msg_.torque.resize(m->nu);
    command.resize(m->nu);
    command2.resize(m->nbody * 6);
    ctrl_command_temp_.resize(m->nu);

    for (int i = 0; i < m->nu; i++)
    {
        ctrl_command[i] = 0.0;
        command[i] = 0.0;
    }

    for (int i = 0; i < m->nbody * 6; i++)
    {
        ctrl_command2[i] = 0.0;
        command2[i] = 0.0;
    }

    if (m->jnt_type[0] == 0) //if first joint type is floating.
    {
        ROS_INFO("ROBOT is floating !");
        joint_state_msg_.name.resize(m->nu + 6);
        joint_state_msg_.position.resize(m->nu + 7);
        joint_state_msg_.velocity.resize(m->nu + 6);
        joint_state_msg_.effort.resize(m->nu + 6);
        for (int i = 0; i < 6; i++)
            joint_state_msg_.effort[i] = 0.0;

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
    }
    else if ((m->jnt_type[0] == 2) || (m->jnt_type[0] == 3))
    {
        ROS_INFO("ROBOT is fixed !");

        for (int i = 0; i < m->njnt; i++)
        {
            if ((m->jnt_type[i] != 3) && (m->jnt_type[i] != 2))
            {
                ROS_ERROR("The robot contains ball/free type of joint in the middle. \n Current state publisher doesn't support this type of robot. \n Copy following line and contact me for update : june992@snu.ac.kr ");
                ROS_ERROR("Total number of generalized coordinates(qpos) : %d", m->nq);
                ROS_ERROR("Total number of degrees of freedom(qvel) : %d", m->nv);
                ROS_ERROR("Total number of actuator : %d ", m->nu);
                ROS_ERROR("Total Joint number is : %d ", m->njnt);
                for (int i = 0; i < m->njnt; i++)
                {
                    ROS_ERROR("Joint %d - Type : %d", i, m->jnt_type[i]);
                }
            }
        }

        joint_state_msg_.name.resize(m->njnt);
        joint_state_msg_.position.resize(m->njnt);
        joint_state_msg_.velocity.resize(m->njnt);
        joint_state_msg_.effort.resize(m->njnt);

        for (int i = 0; i < m->njnt; i++)
        {
            std::string buffer(m->names + m->name_jntadr[i]);
            joint_state_msg_.name[i] = buffer;
        }
    }

    sim_status_msg_.name = joint_state_msg_.name;
    sim_status_msg_.position = joint_state_msg_.position;
    sim_status_msg_.velocity = joint_state_msg_.velocity;
    sim_status_msg_.effort = joint_state_msg_.effort;

    //Sensor size setting. sensor size : model sensor size + 1
    // + 1 additional size is for user information

    sensor_state_msg_.sensor.resize(m->nsensor + 1);
    for (int i = 0; i < m->nsensor; i++)
    {
        std::string buffer(m->names + m->name_sensoradr[i]);
        sensor_state_msg_.sensor[i].name = buffer;
        sensor_state_msg_.sensor[i].data.resize(m->sensor_dim[i]);
    }
    sensor_state_msg_.sensor[m->nsensor].name = "user information";
    sensor_state_msg_.sensor[m->nsensor].data.resize(2);

    sim_status_msg_.sensor = sensor_state_msg_.sensor;

    applied_ext_force_.resize(6);    
    // std::cout << "force range " << std::endl;
    // for (int i = 0; i < m->nu; i++)
    // {
    //     std::cout << "actuator : " << i << " f1 :   " << m->actuator_ctrlrange[i * 2] << " f2 : " << m->actuator_ctrlrange[i * 2 + 1] << std::endl;
    // }

    //     if (use_shm)
    //     {
    // #ifdef COMPILE_SHAREDMEMORY
    //         //init_shm_master();
    // #else
    //         std::cout << "WARNING : SHM_NOT_COMPILED " << std::endl;
    // #endif
    //     }
}

void state_publisher()
{
    if (!use_shm)
    {

        sim_time.data = d->time;

        if (m->jnt_type[0] == 0)
        {

            for (int i = 0; i < m->nu; i++)
            {
                joint_state_msg_.position[i + 6] = d->qpos[i + 7];
                joint_state_msg_.velocity[i + 6] = d->qvel[i + 6];
                joint_state_msg_.effort[i + 6] = d->qacc[i + 6];
            }

            for (int i = 0; i < 3; i++)
            {
                joint_state_msg_.position[i] = d->qpos[i];
                joint_state_msg_.position[i + 3] = d->qpos[i + 4];
                joint_state_msg_.velocity[i] = d->qvel[i];
                joint_state_msg_.velocity[i + 3] = d->qvel[i + 3];
                joint_state_msg_.effort[i] = d->qacc[i];
                joint_state_msg_.effort[i + 3] = d->qacc[i + 3];
            }
            joint_state_msg_.position[m->nu + 6] = d->qpos[3];
        }
        else
        {
            for (int i = 0; i < m->njnt; i++)
            {
                joint_state_msg_.position[i] = d->qpos[i];
                joint_state_msg_.velocity[i] = d->qvel[i];
                joint_state_msg_.effort[i] = d->qacc[i];
            }
        }

        for (int i = 0; i < m->nsensor; i++)
        {
            for (int n = 0; n < m->sensor_dim[i]; n++)
            {
                sensor_state_msg_.sensor[i].data[n] = d->sensordata[m->sensor_adr[i] + n];
            }
        }
        sensor_state_msg_.sensor[m->nsensor].data[0] = dif_time;
        sensor_state_msg_.sensor[m->nsensor].data[1] = com_time;

        if (pub_total_mode)
        {
            sim_status_msg_.position = joint_state_msg_.position;
            sim_status_msg_.velocity = joint_state_msg_.velocity;
            sim_status_msg_.effort = joint_state_msg_.effort;
            sim_status_msg_.sensor = sensor_state_msg_.sensor;
            sim_status_msg_.time = d->time;

            sim_status_msg_.header.stamp = ros::Time::now();
            sim_status_pub.publish(sim_status_msg_);
        }
        else
        {
            joint_state_msg_.header.stamp = ros::Time::now();
            sensor_state_msg_.header.stamp = ros::Time::now();
            sensor_state_pub.publish(sensor_state_msg_);
            joint_state_pub.publish(joint_state_msg_);
            sim_time_pub.publish(sim_time);
        }
    }
    else
    {
#ifdef COMPILE_SHAREDMEMORY
        static int cnt = 0;

        mj_shm_->statusWriting = true;

        std::copy(d->qpos + 7, d->qpos + 40, mj_shm_->pos);
        std::copy(d->qvel + 6, d->qvel + 39, mj_shm_->vel);
        std::copy(d->qacc + 6, d->qacc + 39, mj_shm_->torqueActual);

        //memcpy(&mj_shm_->pos, &d->qpos[7], m->na * sizeof(float));
        //memcpy(&mj_shm_->vel, &d->qvel[6], m->na * sizeof(float));
        //memcpy(&mj_shm_->torqueActual, &d->qacc[6], m->na * sizeof(float));

        //std::copy(d->qpos, d->qpos + 3, mj_shm_->pos_virtual);

        //std::copy(d->qpos + 4, d->qpos + 7, mj_shm_->pos_virtual + 3);

        mj_shm_->pos_virtual[0] = d->qpos[0];
        mj_shm_->pos_virtual[1] = d->qpos[1];
        mj_shm_->pos_virtual[2] = d->qpos[2];
        mj_shm_->pos_virtual[3] = d->qpos[4];
        mj_shm_->pos_virtual[4] = d->qpos[5];
        mj_shm_->pos_virtual[5] = d->qpos[6];
        mj_shm_->pos_virtual[6] = d->qpos[3];

        for (int i = 0; i < m->nsensor; i++)
        {

            std::string sensor_name = sensor_state_msg_.sensor[i].name;
            if (sensor_name == "Acc_Pelvis_IMU")
            {
                mj_shm_->imu_acc[0] = d->sensordata[m->sensor_adr[8] + 0];
                mj_shm_->imu_acc[1] = d->sensordata[m->sensor_adr[8] + 1];
                mj_shm_->imu_acc[2] = d->sensordata[m->sensor_adr[8] + 2];
            }
            else if (sensor_name == "LF_Force_sensor")
            {
                for (int j = 0; j < 3; j++)
                    mj_shm_->ftSensor[j] = d->sensordata[m->sensor_adr[i] + j];
            }
            else if (sensor_name == "LF_Torque_sensor")
            {
                for (int j = 0; j < 3; j++)
                    mj_shm_->ftSensor[j + 3] = d->sensordata[m->sensor_adr[i] + j];
            }
            else if (sensor_name == "RF_Force_sensor")
            {
                for (int j = 0; j < 3; j++)
                    mj_shm_->ftSensor[j + 6] = d->sensordata[m->sensor_adr[i] + j];
            }
            else if (sensor_name == "RF_Torque_sensor")
            {
                for (int j = 0; j < 3; j++)
                    mj_shm_->ftSensor[j + 9] = d->sensordata[m->sensor_adr[i] + j];
            }
            else if (sensor_name == "LH_Force_sensor")
            {
                for (int j = 0; j < 3; j++)
                    mj_shm_->ftSensor2[j] = d->sensordata[m->sensor_adr[i] + j];
            }
            else if (sensor_name == "LH_Torque_sensor")
            {
                for (int j = 0; j < 3; j++)
                    mj_shm_->ftSensor2[j + 3] = d->sensordata[m->sensor_adr[i] + j];
            }
            else if (sensor_name == "RH_Force_sensor")
            {
                for (int j = 0; j < 3; j++)
                    mj_shm_->ftSensor2[j + 6] = d->sensordata[m->sensor_adr[i] + j];
            }
            else if (sensor_name == "RH_Torque_sensor")
            {
                for (int j = 0; j < 3; j++)
                    mj_shm_->ftSensor2[j + 9] = d->sensordata[m->sensor_adr[i] + j];
            }
        }

        std::copy(d->qvel, d->qvel + 6, mj_shm_->vel_virtual);

        //memcpy(&mj_shm_->pos_virtual, d->qpos, 7 * sizeof(float));
        //memcpy(&mj_shm_->vel_virtual, d->qvel, 6 * sizeof(float));
        mj_shm_->control_time_us_ = (int)d->time * 1000000;

        mj_shm_->statusWriting = false;

        mj_shm_->statusCount++; // = cnt++;

        mj_shm_->triggerS1 = true;

        //std::cout << d->qpos[7] << "\t" << mj_shm_->pos[0] << std::endl;
        //std::cout<< "pub.."<<std::endl;
#else
        std::cout << "WARNING : SHM_NOT_COMPILED " << std::endl;
#endif
    }
}

void mycontrollerinit()
{
    ros_sim_started = true;
    sync_time_test = ros::Time::now();
}
void mujoco_ros_connector_init()
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
    if (!((ros::Time::now() - wait_time).toSec() < 1.0))
    {

        ROS_ERROR("NO RESPONSE FROM CONTROLLER ");
    }

    state_publisher_init();

    std::cout << " PUBLISHER INIT COMPLETE " << std::endl;
    state_publisher();

    std::cout << " PUBLISH TEST COMPLETE" << std::endl;

    rst_msg_.data = "INIT";
    sim_command_pub.publish(rst_msg_);
    wait_time = ros::Time::now();
    while ((controller_init_check) && ((ros::Time::now() - wait_time).toSec() < 1.0))
    {
        ros::spinOnce();
        poll_r.sleep();
    }

    mjcb_control = mycontroller;
    mycontrollerinit();
    std::cout << " MUJOCO_ROS_CONNTECTOR INITIALIZE COMPLETE" << std::endl;

    if ((!controller_init_check) && (!controller_reset_check))
    {
        ROS_INFO("CONNECT COMPLETE");
        ctrlstat = "CONNECTED";
    }
    else
    {
        ctrlstat = "MISSING";
    }

    controller_reset_check = true;
    controller_init_check = true;
}

void mycontroller(const mjModel *m, mjData *d)
{
    ros::spinOnce();

    if (settings.run)
    {
        if (ros_sim_started)
        {
            state_publisher();
            double ros_time_now, ros_time_avatar_mode11;
            ros_time_now = ros::Time::now().toSec();
            // ros::param::get("tocabi_avatar_thread11_start_time", ros_time_avatar_mode11);
            //apply force (dg add)
            int link_idx = 6*force_appiedd_link_idx_;
            if( ext_force_applied_ )
            {
                // force on the pelvis in global frame
                // d->qfrc_applied[0] = 100; // x-axis
                // d->qfrc_applied[1] = 10; // y-axis

                // d->qfrc_applied[2] = -50;
                // d->xfrc_applied[0] = 50;
                // d->xfrc_applied[1] = 20;

                //L_Shoulder1_Link
                // d->xfrc_applied[6*19+0] = 00;
                // d->xfrc_applied[6*19+1] = 00;
                // d->xfrc_applied[6*19+2] = -50;
                // d->qfrc_applied[9] = 1;
                // d->qfrc_applied[15] = 1;

                d->xfrc_applied[link_idx+0] = applied_ext_force_[0];
                d->xfrc_applied[link_idx+1] = applied_ext_force_[1];
                d->xfrc_applied[link_idx+2] = applied_ext_force_[2];
                d->xfrc_applied[link_idx+3] = applied_ext_force_[3];
                d->xfrc_applied[link_idx+4] = applied_ext_force_[4];
                d->xfrc_applied[link_idx+5] = applied_ext_force_[5];
            }

            if (use_shm)
            {
#ifdef COMPILE_SHAREDMEMORY

                while (mj_shm_->commanding)
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(1));
                }
                cmd_rcv = true;
                //std::copy(mj_shm_->torqueCommand, mj_shm_->torqueCommand + m->nu, ctrl_command);
                for (int i = 0; i < m->nu; i++)
                    ctrl_command_temp_[i] = mj_shm_->torqueCommand[i];
#else
                std::cout << "WARNING : Getting command, while SHM_NOT_COMPILED " << std::endl;
#endif
            }
            else
            {
                std::copy(command.begin(), command.end(), ctrl_command_temp_.begin());
            }

            static int clat_l = 0;

            ctrl_cmd_que_.push_back(ctrl_command_temp_);

            std::copy(ctrl_cmd_que_[0].begin(), ctrl_cmd_que_[0].end(), ctrl_command);

            while (ctrl_cmd_que_.size() > com_latency)
            {
                ctrl_cmd_que_.pop_front();
            }

            clat_l = com_latency;

            if (!settings.controlui)
            {
                if (cmd_rcv)
                {
                    mju_copy(d->ctrl, ctrl_command, m->nu);
                }
                if (custom_ft_applied)
                {
                    mju_copy(d->xfrc_applied, ctrl_command2, m->nbody * 6);
                }
            }

            ROS_INFO_COND(settings.debug == 1, "MJ_TIME:%10.5f ros:%10.5f dif:%10.5f", d->time, ros_sim_runtime.toSec(), d->time - ros_sim_runtime.toSec());
            ROS_INFO_COND(settings.debug == 1, "TEST FOR THERE ");

            if (settings.debug == 1)
            {
                std::cout << "command torque " << std::endl;
                for (int i = 0; i < m->nu; i++)
                {

                    std::cout << ctrl_command[i] << "\t";
                }
                std::cout << std::endl;
            }
            static std::chrono::high_resolution_clock::time_point t_before = std::chrono::high_resolution_clock::now();
            std::chrono::high_resolution_clock::time_point rt_now = std::chrono::high_resolution_clock::now();

            //static double t_before = ros::Time::now().toSec();
            //double rt_now = ros::Time::now().toSec();

            if (settings.timecheck)
            {
                std::cout << "rdif : " << std::setw(8) << (double)(rt_now - t_before).count() / 1000000.0 << "ms \t t_now : " << d->time << " \t r_now : " << ros::Time::now().toSec() - sync_time_test.toSec() << " \t t dif : " << ros::Time::now().toSec() - sync_time_test.toSec() - d->time << std::endl;
            }

            t_before = rt_now;
        }
    }
    //ros::V_string temp;
    //ros::master::getNodes(temp);

    //ROS_INFO("mycon :: time %f ", d->time);
    //for (int i = 0; i < temp.size(); i++)
    //    std::cout << temp[i] << std::endl;
}

//----------------------- profiler, sensor, info, watch ---------------------------------

// init profiler figures
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
    figconstraint.figurergba[3] = 0.5f;
    figcost.figurergba[3] = 0.5f;
    figsize.figurergba[3] = 0.5f;
    figtimer.figurergba[3] = 0.5f;

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

// update profiler figures
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
    mjtNum total = d->timer[mjTIMER_STEP].duration;
    int number = d->timer[mjTIMER_STEP].number;
    if (!number)
    {
        total = d->timer[mjTIMER_FORWARD].duration;
        number = d->timer[mjTIMER_FORWARD].number;
    }
    number = mjMAX(1, number);
    float tdata[5] = {
        (float)(total / number),
        (float)(d->timer[mjTIMER_POS_COLLISION].duration / number),
        (float)(d->timer[mjTIMER_POS_MAKE].duration / number) +
            (float)(d->timer[mjTIMER_POS_PROJECT].duration / number),
        (float)(d->timer[mjTIMER_CONSTRAINT].duration / number),
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

// show profiler figures
void profilershow(mjrRect rect)
{
    mjrRect viewport = {
        rect.left + rect.width - rect.width / 4,
        rect.bottom,
        rect.width / 4,
        rect.height / 4};
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
    figsensor.figurergba[3] = 0.5f;

    // set flags
    figsensor.flg_extend = 1;
    figsensor.flg_barplot = 1;
    figsensor.flg_symmetric = 1;

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
    // constant width with and without profiler
    int width = settings.profiler ? rect.width / 3 : rect.width / 4;

    // render figure on the right
    mjrRect viewport = {
        rect.left + rect.width - width,
        rect.bottom,
        width,
        rect.height / 3};
    mjr_figure(viewport, &figsensor, &con);
}

// prepare info text
void infotext(char *title, char *content, double interval)
{
    char tmp[20];

    // compute solver error
    mjtNum solerr = 0;
    if (d->solver_iter)
    {
        int ind = mjMIN(d->solver_iter - 1, mjNSOLVER - 1);
        solerr = mju_min(d->solver[ind].improvement, d->solver[ind].gradient);
        if (solerr == 0)
            solerr = mju_max(d->solver[ind].improvement, d->solver[ind].gradient);
    }
    solerr = mju_log10(mju_max(mjMINVAL, solerr));

    // get ROS time
    if (settings.run)
    {
        if (pause_check)
        {
            sim_time_ros = ros::Duration(d->time);
            sim_time_run = ros::Time::now();
        }
        pause_check = false;
        sim_time_now_ros = ros::Time::now() - sim_time_run + sim_time_ros;
    }
    else
    {
        pause_check = true;
    }

    // prepare info text
    strcpy(title, "Time\nRTime\nt_diff\nSize\nCPU\nSolver   \nFPS\nstack\nconbuf\nefcbuf\nController\ntdiff");
    sprintf(content, "%-20.5f\n%-20.5f\n%-20.5f\n%d  (%d con)\n%.3f\n%.1f  (%d it)\n%.0f\n%.3f\n%.3f\n%.3f\n%s\n%-20.6f",
            d->time, sim_time_now_ros.toSec(), sim_time_now_ros.toSec() - d->time,
            d->nefc, d->ncon,
            settings.run ? d->timer[mjTIMER_STEP].duration / mjMAX(1, d->timer[mjTIMER_STEP].number) : d->timer[mjTIMER_FORWARD].duration / mjMAX(1, d->timer[mjTIMER_FORWARD].number),
            solerr, d->solver_iter,
            1 / interval,
            d->maxuse_stack / (double)d->nstack,
            d->maxuse_con / (double)m->nconmax,
            d->maxuse_efc / (double)m->njmax,
            ctrlstat.c_str(),
            sim_cons_time);

    // add Energy if enabled
    if (mjENABLED(mjENBL_ENERGY))
    {
        sprintf(tmp, "\n%.3f", d->energy[0] + d->energy[1]);
        strcat(content, tmp);
        strcat(title, "\nEnergy");
    }

    // add FwdInv if enabled
    if (mjENABLED(mjENBL_FWDINV))
    {
        sprintf(tmp, "\n%.1f %.1f",
                mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[0])),
                mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[1])));
        strcat(content, tmp);
        strcat(title, "\nFwdInv");
    }
}

// sprintf forwarding, to avoid compiler warning in x-macro
void printfield(char *str, void *ptr)
{
    sprintf(str, "%g", *(mjtNum *)ptr);
}

// update watch
void watch(void)
{
    // clear
    ui1.sect[SECT_WATCH].item[2].multi.nelem = 1;
    strcpy(ui1.sect[SECT_WATCH].item[2].multi.name[0], "invalid field");

    // prepare constants for NC
    int nv = m->nv;
    int njmax = m->njmax;

// find specified field in mjData arrays, update value
#define X(TYPE, NAME, NR, NC)                                       \
    if (!strcmp(#NAME, settings.field) && !strcmp(#TYPE, "mjtNum")) \
    {                                                               \
        if (settings.index >= 0 && settings.index < m->NR * NC)     \
            printfield(ui1.sect[SECT_WATCH].item[2].multi.name[0],  \
                       d->NAME + settings.index);                   \
        else                                                        \
            strcpy(ui1.sect[SECT_WATCH].item[2].multi.name[0],      \
                   "invalid index");                                \
        return;                                                     \
    }

    MJDATA_POINTERS
#undef X
}

//-------------------------------- UI construction --------------------------------------

// make physics section of UI
void makephysics(int oldstate)
{
    int i;

    mjuiDef defPhysics[] =
        {
            {mjITEM_SECTION, "Physics", oldstate, NULL, "AP"},
            {mjITEM_SELECT, "Integrator", 2, &(m->opt.integrator), "Euler\nRK4"},
            {mjITEM_SELECT, "Collision", 2, &(m->opt.collision), "All\nPair\nDynamic"},
            {mjITEM_SELECT, "Cone", 2, &(m->opt.cone), "Pyramidal\nElliptic"},
            {mjITEM_SELECT, "Jacobian", 2, &(m->opt.jacobian), "Dense\nSparse\nAuto"},
            {mjITEM_SELECT, "Solver", 2, &(m->opt.solver), "PGS\nCG\nNewton"},
            {mjITEM_SEPARATOR, "Algorithmic Parameters", 1},
            {mjITEM_EDITNUM, "Timestep", 2, &(m->opt.timestep), "1 0 1"},
            {mjITEM_EDITINT, "Iterations", 2, &(m->opt.iterations), "1 0 1000"},
            {mjITEM_EDITNUM, "Tolerance", 2, &(m->opt.tolerance), "1 0 1"},
            {mjITEM_EDITINT, "Noslip Iter", 2, &(m->opt.noslip_iterations), "1 0 1000"},
            {mjITEM_EDITNUM, "Noslip Tol", 2, &(m->opt.noslip_tolerance), "1 0 1"},
            {mjITEM_EDITINT, "MRR Iter", 2, &(m->opt.mpr_iterations), "1 0 1000"},
            {mjITEM_EDITNUM, "MPR Tol", 2, &(m->opt.mpr_tolerance), "1 0 1"},
            {mjITEM_EDITNUM, "API Rate", 2, &(m->opt.apirate), "1 0 1000"},
            {mjITEM_SEPARATOR, "Physical Parameters", 1},
            {mjITEM_EDITNUM, "Gravity", 2, m->opt.gravity, "3"},
            {mjITEM_EDITNUM, "Wind", 2, m->opt.wind, "3"},
            {mjITEM_EDITNUM, "Magnetic", 2, m->opt.magnetic, "3"},
            {mjITEM_EDITNUM, "Density", 2, &(m->opt.density), "1"},
            {mjITEM_EDITNUM, "Viscosity", 2, &(m->opt.viscosity), "1"},
            {mjITEM_EDITNUM, "Imp Ratio", 2, &(m->opt.impratio), "1"},
            {mjITEM_SEPARATOR, "Disable Flags", 1},
            {mjITEM_END}};
    mjuiDef defEnableFlags[] =
        {
            {mjITEM_SEPARATOR, "Enable Flags", 1},
            {mjITEM_END}};
    mjuiDef defOverride[] =
        {
            {mjITEM_SEPARATOR, "Contact Override", 1},
            {mjITEM_EDITNUM, "Margin", 2, &(m->opt.o_margin), "1"},
            {mjITEM_EDITNUM, "Sol Imp", 2, &(m->opt.o_solimp), "5"},
            {mjITEM_EDITNUM, "Sol Ref", 2, &(m->opt.o_solref), "2"},
            {mjITEM_END}};

    // add physics
    mjui_add(&ui0, defPhysics);

    // add flags programmatically
    mjuiDef defFlag[] =
        {
            {mjITEM_CHECKINT, "", 2, NULL, ""},
            {mjITEM_END}};
    for (i = 0; i < mjNDISABLE; i++)
    {
        strcpy(defFlag[0].name, mjDISABLESTRING[i]);
        defFlag[0].pdata = settings.disable + i;
        mjui_add(&ui0, defFlag);
    }
    mjui_add(&ui0, defEnableFlags);
    for (i = 0; i < mjNENABLE; i++)
    {
        strcpy(defFlag[0].name, mjENABLESTRING[i]);
        defFlag[0].pdata = settings.enable + i;
        mjui_add(&ui0, defFlag);
    }

    // add contact override
    mjui_add(&ui0, defOverride);
}

// make rendering section of UI
void makerendering(int oldstate)
{
    int i, j;

    mjuiDef defRendering[] =
        {
            {mjITEM_SECTION, "Rendering", oldstate, NULL, "AR"},
            {mjITEM_SELECT, "Camera", 2, &(settings.camera), "Free\nTracking"},
            {mjITEM_SELECT, "Label", 2, &(vopt.label),
             "None\nBody\nJoint\nGeom\nSite\nCamera\nLight\nTendon\nActuator\nConstraint\nSkin\nSelection\nSel Pnt\nForce"},
            {mjITEM_SELECT, "Frame", 2, &(vopt.frame),
             "None\nBody\nGeom\nSite\nCamera\nLight\nWorld"},
            {mjITEM_SEPARATOR, "Model Elements", 1},
            {mjITEM_END}};
    mjuiDef defOpenGL[] =
        {
            {mjITEM_SEPARATOR, "OpenGL Effects", 1},
            {mjITEM_END}};

    // add model cameras, up to UI limit
    for (i = 0; i < mjMIN(m->ncam, mjMAXUIMULTI - 2); i++)
    {
        // prepare name
        char camname[mjMAXUITEXT] = "\n";
        if (m->names[m->name_camadr[i]])
            strcat(camname, m->names + m->name_camadr[i]);
        else
            sprintf(camname, "\nCamera %d", i);

        // check string length
        if (strlen(camname) + strlen(defRendering[1].other) >= mjMAXUITEXT - 1)
            break;

        // add camera
        strcat(defRendering[1].other, camname);
    }

    // add rendering standard
    mjui_add(&ui0, defRendering);

    // add flags programmatically
    mjuiDef defFlag[] =
        {
            {mjITEM_CHECKBYTE, "", 2, NULL, ""},
            {mjITEM_END}};
    for (i = 0; i < mjNVISFLAG; i++)
    {
        // set name, remove "&"
        strcpy(defFlag[0].name, mjVISSTRING[i][0]);
        for (j = 0; j < strlen(mjVISSTRING[i][0]); j++)
            if (mjVISSTRING[i][0][j] == '&')
            {
                strcpy(defFlag[0].name + j, mjVISSTRING[i][0] + j + 1);
                break;
            }

        // set shortcut and data
        sprintf(defFlag[0].other, " %s", mjVISSTRING[i][2]);
        defFlag[0].pdata = vopt.flags + i;
        mjui_add(&ui0, defFlag);
    }
    mjui_add(&ui0, defOpenGL);
    for (i = 0; i < mjNRNDFLAG; i++)
    {
        strcpy(defFlag[0].name, mjRNDSTRING[i][0]);
        sprintf(defFlag[0].other, " %s", mjRNDSTRING[i][2]);
        defFlag[0].pdata = scn.flags + i;
        mjui_add(&ui0, defFlag);
    }
}

// make group section of UI
void makegroup(int oldstate)
{
    mjuiDef defGroup[] =
        {
            {mjITEM_SECTION, "Group enable", oldstate, NULL, "AG"},
            {mjITEM_SEPARATOR, "Geom groups", 1},
            {mjITEM_CHECKBYTE, "Geom 0", 2, vopt.geomgroup, " 0"},
            {mjITEM_CHECKBYTE, "Geom 1", 2, vopt.geomgroup + 1, " 1"},
            {mjITEM_CHECKBYTE, "Geom 2", 2, vopt.geomgroup + 2, " 2"},
            {mjITEM_CHECKBYTE, "Geom 3", 2, vopt.geomgroup + 3, " 3"},
            {mjITEM_CHECKBYTE, "Geom 4", 2, vopt.geomgroup + 4, " 4"},
            {mjITEM_CHECKBYTE, "Geom 5", 2, vopt.geomgroup + 5, " 5"},
            {mjITEM_SEPARATOR, "Site groups", 1},
            {mjITEM_CHECKBYTE, "Site 0", 2, vopt.sitegroup, "S0"},
            {mjITEM_CHECKBYTE, "Site 1", 2, vopt.sitegroup + 1, "S1"},
            {mjITEM_CHECKBYTE, "Site 2", 2, vopt.sitegroup + 2, "S2"},
            {mjITEM_CHECKBYTE, "Site 3", 2, vopt.sitegroup + 3, "S3"},
            {mjITEM_CHECKBYTE, "Site 4", 2, vopt.sitegroup + 4, "S4"},
            {mjITEM_CHECKBYTE, "Site 5", 2, vopt.sitegroup + 5, "S5"},
            {mjITEM_SEPARATOR, "Joint groups", 1},
            {mjITEM_CHECKBYTE, "Joint 0", 2, vopt.jointgroup, ""},
            {mjITEM_CHECKBYTE, "Joint 1", 2, vopt.jointgroup + 1, ""},
            {mjITEM_CHECKBYTE, "Joint 2", 2, vopt.jointgroup + 2, ""},
            {mjITEM_CHECKBYTE, "Joint 3", 2, vopt.jointgroup + 3, ""},
            {mjITEM_CHECKBYTE, "Joint 4", 2, vopt.jointgroup + 4, ""},
            {mjITEM_CHECKBYTE, "Joint 5", 2, vopt.jointgroup + 5, ""},
            {mjITEM_SEPARATOR, "Tendon groups", 1},
            {mjITEM_CHECKBYTE, "Tendon 0", 2, vopt.tendongroup, ""},
            {mjITEM_CHECKBYTE, "Tendon 1", 2, vopt.tendongroup + 1, ""},
            {mjITEM_CHECKBYTE, "Tendon 2", 2, vopt.tendongroup + 2, ""},
            {mjITEM_CHECKBYTE, "Tendon 3", 2, vopt.tendongroup + 3, ""},
            {mjITEM_CHECKBYTE, "Tendon 4", 2, vopt.tendongroup + 4, ""},
            {mjITEM_CHECKBYTE, "Tendon 5", 2, vopt.tendongroup + 5, ""},
            {mjITEM_SEPARATOR, "Actuator groups", 1},
            {mjITEM_CHECKBYTE, "Actuator 0", 2, vopt.actuatorgroup, ""},
            {mjITEM_CHECKBYTE, "Actuator 1", 2, vopt.actuatorgroup + 1, ""},
            {mjITEM_CHECKBYTE, "Actuator 2", 2, vopt.actuatorgroup + 2, ""},
            {mjITEM_CHECKBYTE, "Actuator 3", 2, vopt.actuatorgroup + 3, ""},
            {mjITEM_CHECKBYTE, "Actuator 4", 2, vopt.actuatorgroup + 4, ""},
            {mjITEM_CHECKBYTE, "Actuator 5", 2, vopt.actuatorgroup + 5, ""},
            {mjITEM_END}};

    // add section
    mjui_add(&ui0, defGroup);
}

// make joint section of UI
void makejoint(int oldstate)
{
    int i;

    mjuiDef defJoint[] =
        {
            {mjITEM_SECTION, "Joint", oldstate, NULL, "AJ"},
            {mjITEM_END}};
    mjuiDef defSlider[] =
        {
            {mjITEM_SLIDERNUM, "", 2, NULL, "0 1"},
            {mjITEM_END}};

    // add section
    mjui_add(&ui1, defJoint);
    defSlider[0].state = 4;

    // add scalar joints, exit if UI limit reached
    int itemcnt = 0;
    for (i = 0; i < m->njnt && itemcnt < mjMAXUIITEM; i++)
        if ((m->jnt_type[i] == mjJNT_HINGE || m->jnt_type[i] == mjJNT_SLIDE))
        {
            // skip if joint group is disabled
            if (!vopt.jointgroup[mjMAX(0, mjMIN(mjNGROUP - 1, m->jnt_group[i]))])
                continue;

            // set data and name
            defSlider[0].pdata = d->qpos + m->jnt_qposadr[i];
            if (m->names[m->name_jntadr[i]])
                mju_strncpy(defSlider[0].name, m->names + m->name_jntadr[i],
                            mjMAXUINAME);
            else
                sprintf(defSlider[0].name, "joint %d", i);

            // set range
            if (m->jnt_limited[i])
                sprintf(defSlider[0].other, "%.4g %.4g",
                        m->jnt_range[2 * i], m->jnt_range[2 * i + 1]);
            else if (m->jnt_type[i] == mjJNT_SLIDE)
                strcpy(defSlider[0].other, "-1 1");
            else
                strcpy(defSlider[0].other, "-3.1416 3.1416");

            // add and count
            mjui_add(&ui1, defSlider);
            itemcnt++;
        }
}

// make control section of UI
void makecontrol(int oldstate)
{
    int i;

    mjuiDef defControl[] =
        {
            {mjITEM_SECTION, "Control", oldstate, NULL, "AC"},
            {mjITEM_BUTTON, "Clear all", 2},
            {mjITEM_END}};
    mjuiDef defSlider[] =
        {
            {mjITEM_SLIDERNUM, "", 2, NULL, "0 1"},
            {mjITEM_END}};

    // add section
    mjui_add(&ui1, defControl);
    defSlider[0].state = 2;

    // add controls, exit if UI limit reached (Clear button already added)
    int itemcnt = 1;
    for (i = 0; i < m->nu && itemcnt < mjMAXUIITEM; i++)
    {
        // skip if actuator group is disabled
        if (!vopt.actuatorgroup[mjMAX(0, mjMIN(mjNGROUP - 1, m->actuator_group[i]))])
            continue;

        // set data and name
        defSlider[0].pdata = d->ctrl + i;
        if (m->names[m->name_actuatoradr[i]])
            mju_strncpy(defSlider[0].name, m->names + m->name_actuatoradr[i],
                        mjMAXUINAME);
        else
            sprintf(defSlider[0].name, "control %d", i);

        // set range
        if (m->actuator_ctrllimited[i])
            sprintf(defSlider[0].other, "%.4g %.4g",
                    m->actuator_ctrlrange[2 * i], m->actuator_ctrlrange[2 * i + 1]);
        else
            strcpy(defSlider[0].other, "-1 1");

        // add and count
        mjui_add(&ui1, defSlider);
        itemcnt++;
    }
}

// make model-dependent UI sections
void makesections(void)
{
    int i;

    // get section open-close state, UI 0
    int oldstate0[NSECT0];
    for (i = 0; i < NSECT0; i++)
    {
        oldstate0[i] = 0;
        if (ui0.nsect > i)
            oldstate0[i] = ui0.sect[i].state;
    }

    // get section open-close state, UI 1
    int oldstate1[NSECT1];
    for (i = 0; i < NSECT1; i++)
    {
        oldstate1[i] = 0;
        if (ui1.nsect > i)
            oldstate1[i] = ui1.sect[i].state;
    }

    // clear model-dependent sections of UI
    ui0.nsect = SECT_PHYSICS;
    ui1.nsect = SECT_JOINT;

    // make
    makephysics(oldstate0[SECT_PHYSICS]);
    makerendering(oldstate0[SECT_RENDERING]);
    makegroup(oldstate0[SECT_GROUP]);
    makejoint(oldstate1[SECT_JOINT]);
    makecontrol(oldstate1[SECT_CONTROL]);
}

//-------------------------------- utility functions ------------------------------------

// align and scale view
void alignscale(void)
{
    // autoscale
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;

    // set to free camera
    cam.type = mjCAMERA_FREE;
}

// copy qpos to clipboard as key
void copykey(void)
{
    char clipboard[5000] = "<key qpos='";
    char buf[200];

    // prepare string
    for (int i = 0; i < m->nq; i++)
    {
        sprintf(buf, i == m->nq - 1 ? "%g" : "%g ", d->qpos[i]);
        strcat(clipboard, buf);
    }
    strcat(clipboard, "'/>");

    // copy to clipboard
    glfwSetClipboardString(window, clipboard);
}

// millisecond timer, for MuJoCo built-in profiler
mjtNum timer(void)
{
    return (mjtNum)(1000 * ros::Time::now().toSec());
}

// clear all times
void cleartimers(void)
{
    for (int i = 0; i < mjNTIMER; i++)
    {
        d->timer[i].duration = 0;
        d->timer[i].number = 0;
    }
}

// update UI 0 when MuJoCo structures change (except for joint sliders)
void updatesettings(void)
{
    int i;

    // physics flags
    for (i = 0; i < mjNDISABLE; i++)
        settings.disable[i] = ((m->opt.disableflags & (1 << i)) != 0);
    for (i = 0; i < mjNENABLE; i++)
        settings.enable[i] = ((m->opt.enableflags & (1 << i)) != 0);

    // camera
    if (cam.type == mjCAMERA_FIXED)
        settings.camera = 2 + cam.fixedcamid;
    else if (cam.type == mjCAMERA_TRACKING)
        settings.camera = 1;
    else
        settings.camera = 0;

    // update UI
    mjui_update(-1, -1, &ui0, &uistate, &con);
}

//--------------------------------- UI hooks (for uitools.c) ----------------------------

// determine enable/disable item state given category
int uiPredicate(int category, void *userdata)
{
    switch (category)
    {
    case 2: // require model
        return (m != NULL);

    case 3: // require model and nkey
        return (m && m->nkey);

    case 4: // require model and paused
        return (m && !settings.run);

    default:
        return 1;
    }
}

// set window layout
void uiLayout(mjuiState *state)
{
    mjrRect *rect = state->rect;

    // set number of rectangles
    state->nrect = 4;

    // rect 0: entire framebuffer
    rect[0].left = 0;
    rect[0].bottom = 0;
    glfwGetFramebufferSize(window, &rect[0].width, &rect[0].height);

    // rect 1: UI 0
    rect[1].left = 0;
    rect[1].width = settings.ui0 ? ui0.width : 0;
    rect[1].bottom = 0;
    rect[1].height = rect[0].height;

    // rect 2: UI 1
    rect[2].width = settings.ui1 ? ui1.width : 0;
    rect[2].left = mjMAX(0, rect[0].width - rect[2].width);
    rect[2].bottom = 0;
    rect[2].height = rect[0].height;

    // rect 3: 3D plot (everything else is an overlay)
    rect[3].left = rect[1].width;
    rect[3].width = mjMAX(0, rect[0].width - rect[1].width - rect[2].width);
    rect[3].bottom = 0;
    rect[3].height = rect[0].height;
}

// handle UI event
void uiEvent(mjuiState *state)
{
    int i;
    char err[200];

    // call UI 0 if event is directed to it
    if ((state->dragrect == ui0.rectid) ||
        (state->dragrect == 0 && state->mouserect == ui0.rectid) ||
        state->type == mjEVENT_KEY)
    {
        // process UI event
        mjuiItem *it = mjui_event(&ui0, state, &con);

        // file section
        if (it && it->sectionid == SECT_FILE)
        {
            switch (it->itemid)
            {
            case 0: // Save xml
                if (!mj_saveLastXML("mjmodel.xml", m, err, 200))
                    printf("Save XML error: %s", err);
                break;

            case 1: // Save mjb
                mj_saveModel(m, "mjmodel.mjb", NULL, 0);
                break;

            case 2: // Print model
                mj_printModel(m, "MJMODEL.TXT");
                break;

            case 3: // Print data
                mj_printData(m, d, "MJDATA.TXT");
                break;

            case 4: // Quit
                settings.exitrequest = 1;
                break;
            }
        }

        // option section
        else if (it && it->sectionid == SECT_OPTION)
        {
            switch (it->itemid)
            {
            case 0: // Spacing
                ui0.spacing = mjui_themeSpacing(settings.spacing);
                ui1.spacing = mjui_themeSpacing(settings.spacing);
                break;

            case 1: // Color
                ui0.color = mjui_themeColor(settings.color);
                ui1.color = mjui_themeColor(settings.color);
                break;

            case 2: // Font
                mjr_changeFont(50 * (settings.font + 1), &con);
                break;

            case 9: // Full screen
                if (glfwGetWindowMonitor(window))
                {
                    // restore window from saved data
                    glfwSetWindowMonitor(window, NULL, windowpos[0], windowpos[1],
                                         windowsize[0], windowsize[1], 0);
                }

                // currently windowed: switch to full screen
                else
                {
                    // save window data
                    glfwGetWindowPos(window, windowpos, windowpos + 1);
                    glfwGetWindowSize(window, windowsize, windowsize + 1);

                    // switch
                    glfwSetWindowMonitor(window, glfwGetPrimaryMonitor(), 0, 0,
                                         vmode.width, vmode.height, vmode.refreshRate);
                }

                // reinstante vsync, just in case
                glfwSwapInterval(settings.vsync);
                break;

            case 10: // Vertical sync
                glfwSwapInterval(settings.vsync);
                break;

            case 12: //DEBUG mode btn
                //ROS_INFO("DEBUG mode on ");
                break;
            case 13: //Time check btn
                //ROS_INFO("Time check on ");
                break;
            }

            // modify UI
            uiModify(window, &ui0, state, &con);
            uiModify(window, &ui1, state, &con);
        }

        // simulation section
        else if (it && it->sectionid == SECT_SIMULATION)
        {
            switch (it->itemid)
            {
            case 1: // Reset
                if (m)
                {
                    settings.key = 0;
                    c_reset();
                }
                break;

            case 2: // Reload
                settings.loadrequest = 1;
                ROS_INFO("Reload press %d ", settings.loadrequest);
                break;

            case 3: // Align
                alignscale();
                updatesettings();
                break;

            case 4: // Copy pose
                copykey();
                break;

            case 5: // key +
                settings.key++;
                if (settings.key > m->nkey - 1)
                    settings.key = 0;
                char s_key[10];
                sprintf(s_key, "%d", (unsigned short)settings.key);
                strcpy(ui0.sect[SECT_SIMULATION].item[7].multi.name[0], s_key);
                c_reset();
                break;

            case 6: // key -
                settings.key--;
                if (settings.key < 0)
                    settings.key = m->nkey - 1;
                s_key[10];
                sprintf(s_key, "%d", (unsigned short)settings.key);
                strcpy(ui0.sect[SECT_SIMULATION].item[7].multi.name[0], s_key);
                c_reset();
                break;

            case 8: // Latency ++
                com_latency++;
                char com_key[10];
                //std::cout << "com_latency ++" << std::endl;
                sprintf(com_key, "%5.2f ms", (com_latency * m->opt.timestep * 1000.0));
                strcpy(ui0.sect[SECT_SIMULATION].item[10].multi.name[0], com_key);
                mjui_update(-1, -1, &ui0, &uistate, &con);
                break;

            case 9: // Latency --
                com_latency--;
                if (com_latency < 0)
                    com_latency = 0;
                com_key[10];
                //std::cout << "com_latency --" << std::endl;
                sprintf(com_key, "%5.2f ms", (com_latency * m->opt.timestep * 1000.0));
                strcpy(ui0.sect[SECT_SIMULATION].item[10].multi.name[0], com_key);
                mjui_update(-1, -1, &ui0, &uistate, &con);
                break;

            case 11: // Reset to key
                c_reset();
                break;

            case 12: // Set key
                i = settings.key;
                m->key_time[i] = d->time;
                mju_copy(m->key_qpos + i * m->nq, d->qpos, m->nq);
                mju_copy(m->key_qvel + i * m->nv, d->qvel, m->nv);
                mju_copy(m->key_act + i * m->na, d->act, m->na);
                break;
            }
        }

        // physics section
        else if (it && it->sectionid == SECT_PHYSICS)
        {
            // update disable flags in mjOption
            m->opt.disableflags = 0;
            for (i = 0; i < mjNDISABLE; i++)
                if (settings.disable[i])
                    m->opt.disableflags |= (1 << i);

            // update enable flags in mjOption
            m->opt.enableflags = 0;
            for (i = 0; i < mjNENABLE; i++)
                if (settings.enable[i])
                    m->opt.enableflags |= (1 << i);
        }

        // rendering section
        else if (it && it->sectionid == SECT_RENDERING)
        {
            // set camera in mjvCamera
            if (settings.camera == 0)
                cam.type = mjCAMERA_FREE;
            else if (settings.camera == 1)
            {
                if (pert.select > 0)
                {
                    cam.type = mjCAMERA_TRACKING;
                    cam.trackbodyid = pert.select;
                    cam.fixedcamid = -1;
                }
                else
                {
                    cam.type = mjCAMERA_FREE;
                    settings.camera = 0;
                    mjui_update(SECT_RENDERING, -1, &ui0, &uistate, &con);
                }
            }
            else
            {
                cam.type = mjCAMERA_FIXED;
                cam.fixedcamid = settings.camera - 2;
            }
        }

        // group section
        else if (it && it->sectionid == SECT_GROUP)
        {
            // remake joint section if joint group changed
            if (it->name[0] == 'J' && it->name[1] == 'o')
            {
                ui1.nsect = SECT_JOINT;
                makejoint(ui1.sect[SECT_JOINT].state);
                ui1.nsect = NSECT1;
                uiModify(window, &ui1, state, &con);
            }

            // remake control section if actuator group changed
            if (it->name[0] == 'A' && it->name[1] == 'c')
            {
                ui1.nsect = SECT_CONTROL;
                makecontrol(ui1.sect[SECT_CONTROL].state);
                ui1.nsect = NSECT1;
                uiModify(window, &ui1, state, &con);
            }
        }

        // stop if UI processed event
        if (it != NULL || (state->type == mjEVENT_KEY && state->key == 0))
            return;
    }

    // call UI 1 if event is directed to it
    if ((state->dragrect == ui1.rectid) ||
        (state->dragrect == 0 && state->mouserect == ui1.rectid) ||
        state->type == mjEVENT_KEY)
    {
        // process UI event
        mjuiItem *it = mjui_event(&ui1, state, &con);

        // control section
        if (it && it->sectionid == SECT_CONTROL)
        {
            // clear controls
            if (it->itemid == 0)
            {
                mju_zero(d->ctrl, m->nu);
                mjui_update(SECT_CONTROL, -1, &ui1, &uistate, &con);
            }
        }

        // stop if UI processed event
        if (it != NULL || (state->type == mjEVENT_KEY && state->key == 0))
            return;
    }

    // shortcut not handled by UI
    if (state->type == mjEVENT_KEY && state->key != 0)
    {
        switch (state->key)
        {
        case ' ': // Mode
            if (m)
            {
                settings.run = 1 - settings.run;
                pert.active = 0;
                mjui_update(-1, -1, &ui0, state, &con);
            }
            break;

        case mjKEY_RIGHT: // step forward
            if (m && !settings.run)
            {
                cleartimers();
                ros::spinOnce();
                mj_step(m, d);
                profilerupdate();
                sensorupdate();
                updatesettings();
            }
            break;

        case mjKEY_LEFT: // step back
            if (m && !settings.run)
            {
                m->opt.timestep = -m->opt.timestep;
                cleartimers();
                ros::spinOnce();
                mj_step(m, d);
                m->opt.timestep = -m->opt.timestep;
                profilerupdate();
                sensorupdate();
                updatesettings();
            }
            break;

        case mjKEY_DOWN: // step forward 100
            if (m && !settings.run)
            {
                cleartimers();
                for (i = 0; i < 100; i++)
                {
                    ros::spinOnce();
                    mj_step(m, d);
                }
                profilerupdate();
                sensorupdate();
                updatesettings();
            }
            break;

        case mjKEY_UP: // step back 100
            if (m && !settings.run)
            {
                m->opt.timestep = -m->opt.timestep;
                cleartimers();
                for (i = 0; i < 100; i++)
                {
                    ros::spinOnce();
                    mj_step(m, d);
                }
                m->opt.timestep = -m->opt.timestep;
                profilerupdate();
                sensorupdate();
                updatesettings();
            }
            break;

        case mjKEY_PAGE_UP: // select parent body
            if (m && pert.select > 0)
            {
                pert.select = m->body_parentid[pert.select];
                pert.skinselect = -1;

                // stop perturbation if world reached
                if (pert.select <= 0)
                    pert.active = 0;
            }

            break;

        case mjKEY_ESCAPE: // free camera
            cam.type = mjCAMERA_FREE;
            settings.camera = 0;
            mjui_update(SECT_RENDERING, -1, &ui0, &uistate, &con);
            break;
        }

        return;
    }

    // 3D scroll
    if (state->type == mjEVENT_SCROLL && state->mouserect == 3 && m)
    {
        // emulate vertical mouse motion = 5% of window height
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * state->sy, &scn, &cam);

        return;
    }

    // 3D press
    if (state->type == mjEVENT_PRESS && state->mouserect == 3 && m)
    {
        // set perturbation
        int newperturb = 0;
        if (state->control && pert.select > 0)
        {
            // right: translate;  left: rotate
            if (state->right)
                newperturb = mjPERT_TRANSLATE;
            else if (state->left)
                newperturb = mjPERT_ROTATE;

            // perturbation onset: reset reference
            if (newperturb && !pert.active)
                mjv_initPerturb(m, d, &scn, &pert);
        }
        pert.active = newperturb;

        // handle double-click
        if (state->doubleclick)
        {
            // determine selection mode
            int selmode;
            if (state->button == mjBUTTON_LEFT)
                selmode = 1;
            else if (state->control)
                selmode = 3;
            else
                selmode = 2;

            // find geom and 3D click point, get corresponding body
            mjrRect r = state->rect[3];
            mjtNum selpnt[3];
            int selgeom, selskin;
            int selbody = mjv_select(m, d, &vopt,
                                     (mjtNum)r.width / (mjtNum)r.height,
                                     (mjtNum)(state->x - r.left) / (mjtNum)r.width,
                                     (mjtNum)(state->y - r.bottom) / (mjtNum)r.height,
                                     &scn, selpnt, &selgeom, &selskin);

            // set lookat point, start tracking is requested
            if (selmode == 2 || selmode == 3)
            {
                // copy selpnt if anything clicked
                if (selbody >= 0)
                    mju_copy3(cam.lookat, selpnt);

                // switch to tracking camera if dynamic body clicked
                if (selmode == 3 && selbody > 0)
                {
                    // mujoco camera
                    cam.type = mjCAMERA_TRACKING;
                    cam.trackbodyid = selbody;
                    cam.fixedcamid = -1;

                    // UI camera
                    settings.camera = 1;
                    mjui_update(SECT_RENDERING, -1, &ui0, &uistate, &con);
                }
            }

            // set body selection
            else
            {
                if (selbody >= 0)
                {
                    // record selection
                    pert.select = selbody;
                    pert.skinselect = selskin;

                    // compute localpos
                    mjtNum tmp[3];
                    mju_sub3(tmp, selpnt, d->xpos + 3 * pert.select);
                    mju_mulMatTVec(pert.localpos, d->xmat + 9 * pert.select, tmp, 3, 3);
                }
                else
                {
                    pert.select = 0;
                    pert.skinselect = -1;
                }
            }

            // stop perturbation on select
            pert.active = 0;
        }

        return;
    }

    // 3D release
    if (state->type == mjEVENT_RELEASE && state->dragrect == 3 && m)
    {
        // stop perturbation
        pert.active = 0;

        return;
    }

    // 3D move
    if (state->type == mjEVENT_MOVE && state->dragrect == 3 && m)
    {
        // determine action based on mouse button
        mjtMouse action;
        if (state->right)
            action = state->shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        else if (state->left)
            action = state->shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        else
            action = mjMOUSE_ZOOM;

        // move perturb or camera
        mjrRect r = state->rect[3];
        if (pert.active)
            mjv_movePerturb(m, d, action, state->dx / r.height, -state->dy / r.height,
                            &scn, &pert);
        else
            mjv_moveCamera(m, action, state->dx / r.height, -state->dy / r.height,
                           &scn, &cam);

        return;
    }
}

void arrowshow(mjvGeom* arrow)
{
    arrow = scn.geoms + scn.ngeom; 
    makeArrow(arrow);
    scn.ngeom++;

    mjtNum force_vec[3] = {-applied_ext_force_[1], applied_ext_force_[0], 0.0};
    double force = mju_normalize3(force_vec);
    double arrow_length_ = 1.5;
    double theta = atan2(-applied_ext_force_[1],applied_ext_force_[0]);

    if(force > 0.0 && (applied_ext_force_[0] == 0.0 || applied_ext_force_[1] == 0.0))
    {
        arrow->size[0] = 0.04f;
        arrow->size[1] = 0.04f;
        arrow->size[2] = arrow_length_;

        arrow->pos[0] = d->xpos[3 * force_appiedd_link_idx_ + 0] - 0.5*arrow_length_*mju_sign(force_vec[1]);
        arrow->pos[1] = d->xpos[3 * force_appiedd_link_idx_ + 1] + 0.5*arrow_length_*mju_sign(force_vec[0]);
        arrow->pos[2] = d->xpos[3 * force_appiedd_link_idx_ + 2] + 0.2; // You can adjust the z position of the arrow by modifying the constant.
    }
    else if (force > 0.0)
    {
        arrow->size[0] = 0.04f;
        arrow->size[1] = 0.04f;
        arrow->size[2] = arrow_length_;

        arrow->pos[0] = d->xpos[3 * force_appiedd_link_idx_ + 0] - 0.5*arrow_length_*abs(cos(theta))*mju_sign(force_vec[1]);
        arrow->pos[1] = d->xpos[3 * force_appiedd_link_idx_ + 1] + 0.5*arrow_length_*abs(sin(theta))*mju_sign(force_vec[0]);
        arrow->pos[2] = d->xpos[3 * force_appiedd_link_idx_ + 2] + 0.2;
    }
    else
    {
        arrow->size[0] = 0.0;
        arrow->size[1] = 0.0;
        arrow->size[2] = 0.0;
    }      

    mjtNum quat[4], mat[9];
    
    mju_axisAngle2Quat(quat, force_vec, 0.5 * mjPI * ((force > 0) ? 1 : -1));
    mju_quat2Mat(mat, quat);
    mju_n2f(arrow->mat, mat, 9);

    // std::cout << "applied_ext_force_ " << applied_ext_force_[0] << " " << applied_ext_force_[1] << " " << applied_ext_force_[2] << std::endl;
}

void makeArrow(mjvGeom* arrow)
{
    
	arrow->type = mjGEOM_ARROW;
	arrow->dataid = -1;
	arrow->objtype = mjOBJ_SITE;
	arrow->objid = -1;
	arrow->category = mjCAT_DECOR;
	arrow->texid = -1;
	arrow->texuniform = 0;
	arrow->texrepeat[0] = 1;
	arrow->texrepeat[1] = 1;
	arrow->emission = 0;
	arrow->specular = 0.5;
	arrow->shininess = 0.5;
	arrow->reflectance = 0;
	arrow->label[0] = 0;
	arrow->size[0] = 0.04f;
	arrow->size[1] = 0.04f;
	arrow->size[2] = 1.0f;
	arrow->rgba[0] = 1.0f;
	arrow->rgba[1] = 0.1f;
	arrow->rgba[2] = 0.1f;
	arrow->rgba[3] = 1.0f;
	arrow->pos[0] = d->xpos[3*force_appiedd_link_idx_ + 0];
	arrow->pos[1] = d->xpos[3*force_appiedd_link_idx_ + 1];
	arrow->pos[2] = d->xpos[3*force_appiedd_link_idx_ + 2];
}

//--------------------------- rendering and simulation ----------------------------------

// sim thread synchronization

// prepare to render
void prepare(void)
{
    // data for FPS calculation
    static double lastupdatetm = 0;

    // update interval, save update time
    double tmnow = ros::Time::now().toSec();
    double interval = tmnow - lastupdatetm;
    interval = mjMIN(1, mjMAX(0.0001, interval));
    lastupdatetm = tmnow;

    // no model: nothing to do
    if (!m)
        return;

    // update scene
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // update watch and joint sections
    if (settings.ui1)
    {
        // watch
        if (ui1.sect[SECT_WATCH].state)
        {
            watch();
            mjui_update(SECT_WATCH, -1, &ui1, &uistate, &con);
        }

        // joint
        if (ui1.sect[SECT_JOINT].state)
            mjui_update(SECT_JOINT, -1, &ui1, &uistate, &con);
    }

    // update info text
    if (settings.info)
        infotext(info_title, info_content, interval);

    // update profiler
    if (settings.profiler && settings.run)
        profilerupdate();

    // update sensor
    if (settings.sensor && settings.run)
        sensorupdate();

    // clear timers once profiler info has been copied
    cleartimers();
}

// render im main thread (while simulating in background thread)
void render(GLFWwindow *window)
{
    // get 3D rectangle and reduced for profiler
    mjrRect rect = uistate.rect[3];
    mjrRect smallrect = rect;
    if (settings.profiler)
        smallrect.width = rect.width - rect.width / 4;

    // no model
    if (!m)
    {
        // blank screen
        mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);

        // label
        if (settings.loadrequest)
            mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect,
                        "loading", NULL, &con);
        else
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect,
                        "Drag-and-drop model file here", 0, &con);

        // render uis
        if (settings.ui0)
            mjui_render(&ui0, &uistate, &con);
        if (settings.ui1)
            mjui_render(&ui1, &uistate, &con);

        // finalize
        glfwSwapBuffers(window);

        return;
    }

    arrowshow(arrow);

    // render scene
    mjr_render(rect, &scn, &con);

    // show pause/loading label
    if (!settings.run || settings.loadrequest)
        mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect,
                    settings.loadrequest ? "loading" : "pause", NULL, &con);

    // show ui 0
    if (settings.ui0)
    {
        mjui_render(&ui0, &uistate, &con);
    }

    // show ui 1
    if (settings.ui1)
        mjui_render(&ui1, &uistate, &con);

    // show help
    if (settings.help)
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, help_title, help_content, &con);

    // show info
    if (settings.info)
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect,
                    info_title, info_content, &con);

    // show profiler
    if (settings.profiler)
        profilershow(rect);

    // show sensor
    if (settings.sensor)
        sensorshow(smallrect);

    if (settings.run && settings.link_info)
    {
        if (pert.select > 0)
        {
            Eigen::Vector3d euler;
            std::string buffer(m->names + m->name_bodyadr[pert.select]);
            /*
            std::string bnlist[m->nbody];
            for (int i = 0; i < m->nbody; i++)
            {
                bnlist[i] = std::string(m->names + m->name_bodyadr[i]);
            }
            mjtnum *ss = d->xfrc_applied;

            ss[id * 6] = x_f ss[id * 6 + 1] = y_f;
*/
            tf::Quaternion q_(d->xquat[pert.select * 4 + 1], d->xquat[pert.select * 4 + 2], d->xquat[pert.select * 4 + 3], d->xquat[pert.select * 4]);
            tf::Matrix3x3 m_(q_);
            tf::Vector3 global_p_(d->xpos[pert.select * 3], d->xpos[pert.select * 3 + 1], d->xpos[pert.select * 3 + 2]);
            tf::Vector3 local_p_(pert.localpos[0], pert.localpos[1], pert.localpos[2]);

            tf::Vector3 r_ = global_p_ + m_ * local_p_;
            m_.getRPY(euler(0), euler(1), euler(2));

            double radd = 180.0 / 3.141592;

            printf("pert.select : %d, name : %s at sim time : %10.5f\n\t body pos : %10.7f, %10.7f, %10.7f, \n\tlocal pos : %10.7f, %10.7f, %10.7f \n\tpoint pos : %10.7f, %10.7f, %10.7f\n\tEuler angle : %9.4f, %9.4f, %9.4f\n",
                   pert.select, buffer.c_str(), d->time,
                   global_p_[0], global_p_[1], global_p_[2],
                   local_p_[0], local_p_[1], local_p_[2],
                   r_[0], r_[1], r_[2],
                   radd * euler(0), radd * euler(1), radd * euler(2));
        }
    }
    // finalize
    glfwSwapBuffers(window);
}

// simulate in background thread (while rendering in main thread)
void simulate(void)
{
    // cpu-sim syncronization point
    double cpusync = 0;
    mjtNum simsync = 0;

    // run until asked to exit
    while ((!settings.exitrequest) && ros::ok())
    {

        // sleep for 1 ms or yield, to let main thread run
        //  yield results in busy wait - which has better timing but kills battery life
        if (settings.run && settings.busywait)
            std::this_thread::yield();
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // start exclusive access
        mtx.lock();

        // run only if model is present
        if (m)
        {
            // record start time
            double startwalltm = ros::Time::now().toSec();

            // running
            if (settings.run)
            {
                if (settings.debug)
                {
                    std::cout << "sim hz check :::::: " << d->time << std::endl;
                }
                // record cpu time at start of iteration
                double tmstart = ros::Time::now().toSec();

                // out-of-sync (for any reason)
                if (d->time < simsync || tmstart < cpusync || cpusync == 0 ||
                    mju_abs((d->time - simsync) - (tmstart - cpusync)) > syncmisalign)
                {
                    // re-sync
                    cpusync = tmstart;
                    simsync = d->time;

                    // clear old perturbations, apply new
                    mju_zero(d->xfrc_applied, 6 * m->nbody);
                    mjv_applyPerturbPose(m, d, &pert, 0); // move mocap bodies only
                    mjv_applyPerturbForce(m, d, &pert);

                    // run single step, let next iteration deal with timing
                    mj_step(m, d);
                }

                // in-sync
                else
                {
                    // step while simtime lags behind cputime, and within safefactor
                    while ((d->time - simsync) < (ros::Time::now().toSec() - cpusync) &&
                           (ros::Time::now().toSec() - tmstart) < refreshfactor / vmode.refreshRate)
                    {
                        // clear old perturbations, apply new
                        mju_zero(d->xfrc_applied, 6 * m->nbody);
                        mjv_applyPerturbPose(m, d, &pert, 0); // move mocap bodies only
                        mjv_applyPerturbForce(m, d, &pert);

                        // run mj_step
                        mjtNum prevtm = d->time;
                        mj_step(m, d);
                        if (settings.timecheck)
                        {
                        }
                        // break on reset
                        if (d->time < prevtm)
                            break;
                    }
                }
            }

            // paused
            else
            {
                // apply pose perturbation
                mjv_applyPerturbPose(m, d, &pert, 1); // move mocap and dynamic bodies

                // run mj_forward, to update rendering and joint sliders
                mj_forward(m, d);
            }
        }

        // end exclusive access
        mtx.unlock();
    }
}

//-------------------------------- init and main ----------------------------------------

// initalize everything
void init()
{
    // print version, check compatibility
    printf("MuJoCo Pro version %.2lf\n", 0.01 * mj_version());
    if (mjVERSION_HEADER != mj_version())
        mju_error("Headers and library have different versions");

    // activate MuJoCo license
    //ROS_INFO("license file located at %s", key_file.c_str());
    // mj_activate(key_file.c_str());

    // init GLFW, set timer callback (milliseconds)
    if (!glfwInit())
        mju_error("could not initialize GLFW");
    mjcb_time = timer;

    // multisampling
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_VISIBLE, 1);

    // get videomode and save
    vmode = *glfwGetVideoMode(glfwGetPrimaryMonitor());

    // create window
    window = glfwCreateWindow((2 * vmode.width) / 3, (2 * vmode.height) / 3,
                              "Simulate", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        mju_error("could not create window");
    }
    // save window position and size
    glfwGetWindowPos(window, windowpos, windowpos + 1);
    glfwGetWindowSize(window, windowsize, windowsize + 1);

    // make context current, set v-sync
    glfwMakeContextCurrent(window);
    glfwSwapInterval(settings.vsync);

    // init abstract visualization
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&vopt);

    // custum init option
    vopt.geomgroup[2] = false;

    profilerinit();
    sensorinit();

    // make empty scene
    mjv_defaultScene(&scn);
    mjv_makeScene(NULL, &scn, maxgeom);

    // select default font
    int fontscale = uiFontScale(window);
    settings.font = fontscale / 50 - 1;

    // make empty context
    mjr_defaultContext(&con);
    mjr_makeContext(NULL, &con, fontscale);

    // set GLFW callbacks
    uiSetCallback(window, &uistate, uiEvent, uiLayout);
    glfwSetWindowRefreshCallback(window, render);
    glfwSetDropCallback(window, drop);

    // init state and uis
    memset(&uistate, 0, sizeof(mjuiState));
    memset(&ui0, 0, sizeof(mjUI));
    memset(&ui1, 0, sizeof(mjUI));
    ui0.spacing = mjui_themeSpacing(settings.spacing);
    ui0.color = mjui_themeColor(settings.color);
    ui0.predicate = uiPredicate;
    ui0.rectid = 1;
    ui0.auxid = 0;
    ui1.spacing = mjui_themeSpacing(settings.spacing);
    ui1.color = mjui_themeColor(settings.color);
    ui1.predicate = uiPredicate;
    ui1.rectid = 2;
    ui1.auxid = 1;

    // populate uis with standard sections
    mjui_add(&ui0, defFile);
    mjui_add(&ui0, defOption);
    mjui_add(&ui0, defSimulation);
    mjui_add(&ui1, defWatch);
    uiModify(window, &ui0, &uistate, &con);
    uiModify(window, &ui1, &uistate, &con);
}
