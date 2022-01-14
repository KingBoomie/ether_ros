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

   Used for handling the "raw" output data, received from EtherCAT Communicator and transforming them into
   useful, human-readable format, consisted of the EtherCAT variables used by our
   application. Transforms the indeces to variables.
*/

/*****************************************************************************/

#include "pdo_out_publisher.h"
#include "ether_ros/PDOOut.h"
#include "ether_ros/PDORaw.h"
#include "ethercat_slave.h"
#include "utilities.h"
#include "vector"
#include "ether_ros.h"
#include <iostream>
#include <string>

void PDOOutPublisher::pdo_raw_callback(const ether_ros::PDORaw::ConstPtr &pdo_raw)
{
    std::vector<uint8_t> pdo_out_raw = pdo_raw->pdo_out_raw;
    uint8_t *data_ptr;
    size_t pos;
    for (int i = 0; i < master_info.slave_count; i++)
    {
        pos = i * num_process_data_out; //The size of every entry is num_process_data_out
        data_ptr = (uint8_t *)&pdo_out_raw[pos];
        ether_ros::PDOOut pdo_out;
        pdo_out.slave_id = i;
        using namespace utilities;

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



        pdo_out_pub_.publish(pdo_out);
    }
}

void PDOOutPublisher::init(ros::NodeHandle &n)
{
    //Create  ROS subscriber for the Ethercat RAW data
    pdo_raw_sub_ = n.subscribe("pdo_raw", 1000, &PDOOutPublisher::pdo_raw_callback, &pdo_out_publisher);

    //Create  ROS publisher for the Ethercat formatted data
    pdo_out_pub_ = n.advertise<ether_ros::PDOOut>("pdo_out", 1000);
}
