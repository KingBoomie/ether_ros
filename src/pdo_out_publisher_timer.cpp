/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2018 Mike Karamousadakis, NTUA CSL
 *
 *  This file is part of the IgH EtherCAT master userspace program in the ROS environment.
 *
 *  The IgH EtherCAT master userspace program in the ROS environment is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU General
 *  Public License as published by the Free Software Foundation; version 2
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace program in the ROS environment. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  Contact information: mkaramousadakis@zoho.eu
 *****************************************************************************/
/**
   \file pdo_out_publisher.cpp
   \brief Implementation of PDOOutPublisher class.

    Used for streaming the "raw" \a pdo_out data inside the \a process_data_buffer to
    the \a /pdo_out_timer topic and transforming them into useful, human-readable format,
    consisted of the EtherCAT output variables used by our application, at a certain rate.
    It's been created for logging and debugging reasons.
*/

/*****************************************************************************/

#include "pdo_out_publisher_timer.h"
#include "ether_ros/PDOOut.h"
#include "ethercat_slave.h"
#include "utilities.h"
#include "vector"
#include "ether_ros.h"
#include <iostream>
#include <string>

void PDOOutPublisherTimer::timer_callback(const ros::TimerEvent &event)
{
    size_t pos;
    uint8_t *data_ptr;
    using namespace utilities;
    copy_process_data_buffer_to_buf(data_ptr_);

    for (int i = 0; i < master_info.slave_count; i++)
    {
        pos = i * (num_process_data_out + num_process_data_in); //The size of every entry is num_process_data_out
        data_ptr = (uint8_t *)(data_ptr_ + pos);
        ether_ros::PDOOut pdo_out;
        pdo_out.slave_id = i;

        // change the following code to match your needs
        /*

        Insert code here ...

        */
        pdo_out.tx1_control_word = process_input_uint16(data_ptr,0);
        pdo_out.tx1_mode_of_operation = process_input_uint8(data_ptr,2);
        pdo_out.tx1_target_position = process_input_uint32(data_ptr,3);
        pdo_out.tx2_control_word = process_input_uint16(data_ptr,7);
        pdo_out.tx2_mode_of_operation = process_input_uint8(data_ptr,9);
        pdo_out.tx2_target_position = process_input_uint32(data_ptr,10);
        pdo_out.tx2_velocity = process_input_uint32(data_ptr,14);
        pdo_out.tx2_target_acceleration = process_input_uint32(data_ptr,18);
        pdo_out.tx2_target_deceleration = process_input_uint32(data_ptr,22);
        pdo_out.tx3_control_word = process_input_uint16(data_ptr,26);
        pdo_out.tx3_mode_of_operation = process_input_uint8(data_ptr,28);
        pdo_out.tx3_target_acceleration = process_input_uint32(data_ptr,29);
        pdo_out.tx3_target_deceleration = process_input_uint32(data_ptr,33);
        pdo_out.tx3_target_velocity = process_input_uint32(data_ptr,37);

        /*
            .....

        */
        pdo_out_pub_.publish(pdo_out);
    }
}

void PDOOutPublisherTimer::init(ros::NodeHandle &n)
{
    data_ptr_ = (uint8_t *)malloc(total_process_data * sizeof(uint8_t));
    memset(data_ptr_, 0, total_process_data); // fill the buffer with zeros

    //Create  ROS publisher for the Ethercat formatted data
    pdo_out_pub_ = n.advertise<ether_ros::PDOOut>("pdo_out_timer", 1000);

    if (!pdo_out_pub_)
    {
        ROS_FATAL("Unable to start publisher in ProcessDataTimer\n");
        exit(1);
    }
    else
    {
        ROS_INFO("Started ProcessDataTimer publisher\n");
    }
    pdo_out_timer_ = n.createTimer(ros::Duration(5), &PDOOutPublisherTimer::timer_callback, &pdo_out_publisher_timer);
    //Create  ROS timer
    //first parameter is in seconds...

    if (!pdo_out_timer_)
    {
        ROS_FATAL("Unable to start ProcessDataTimer\n");
        exit(1);
    }
    else
    {
        ROS_INFO("Started ProcessDataTimer\n");
    }
    // ros::spinOnce();
}
