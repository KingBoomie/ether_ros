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

#include "pdo_out_listener.h"
// #include "ethercat_slave.h"
#include "utilities.h"
#include "vector"
#include <pthread.h>
#include "ether_ros.h"
#include <iostream>
#include <string>
#include <ether_ros/PDOOut.h>


void PDOOutListener::pdo_out_callback(const ether_ros::PDOOut::ConstPtr &new_var)
{
    pthread_spin_lock(&lock);
    uint8_t slave_id = new_var->slave_id;
    //check if we are broadcasting a variable's value to all slaves
    if (slave_id == 255)
    {
        ROS_INFO("slave_id is 255\n");
        for (int i = 0; i < master_info.slave_count; i++)
        {
            modify_pdo_variable(i, new_var);
        }
    }
    else
    {
        modify_pdo_variable((int)slave_id, new_var);
    }
    pthread_spin_unlock(&lock);
}
/** Write a 16-bit unsigned value to EtherCAT and advance the data pointer.
 *
 * \param DATA EtherCAT data pointer
 * \param VAL new value
 */
#define WRITE_U16(DATA, VAL) \
    do { \
        EC_WRITE_U16(DATA, VAL); \
        ROS_INFO_STREAM("" << sizeof(VAL) << "-byte write of \t" << unsigned(VAL) << " to \t" << (DATA)); \
        (DATA) += sizeof(VAL); \
    } while (0)

/** Write a 8-bit unsigned value to EtherCAT and advance the data pointer.
 *
 * \param DATA EtherCAT data pointer
 * \param VAL new value
 */
#define WRITE_U8(DATA, VAL) \
    do { \
        EC_WRITE_U8(DATA, VAL); \
        ROS_INFO_STREAM("" << sizeof(VAL) << "-byte write of \t" << unsigned(VAL) << " to \t" << (DATA)); \
        (DATA) += sizeof(VAL); \
    } while (0)

/** Write a 32-bit unsigned value to EtherCAT and advance the data pointer.
 *
 * \param DATA EtherCAT data pointer
 * \param VAL new value
 */
#define WRITE_U32(DATA, VAL) \
    do { \
        EC_WRITE_U32(DATA, VAL); \
        ROS_INFO_STREAM("" << sizeof(VAL) << "-byte write of \t" << unsigned(VAL) << " to \t" << (DATA)); \
        (DATA) += sizeof(VAL); \
    } while (0)


#define TX1_INDEX 0
#define TX2_INDEX 7
#define TX3_INDEX 26

void PDOOutListener::modify_pdo_variable(int slave_id, const ether_ros::PDOOut::ConstPtr &new_var)
{
    int8_t type = new_var->txtype;

    if (type == ether_ros::PDOOut::TX1) {
        uint8_t *dataPtr = (process_data_buf +
                                               slave_id * (num_process_data_out + num_process_data_in) + TX1_INDEX);

        EC_WRITE_U16(dataPtr + 0, new_var->tx1_control_word);
        EC_WRITE_U8 (dataPtr + 2, new_var->tx1_mode_of_operation);
        EC_WRITE_U32(dataPtr + 3, new_var->tx1_target_position);

    } else if (type == ether_ros::PDOOut::TX2) {
        auto *dataPtr = (process_data_buf + slave_id * (num_process_data_out + num_process_data_in) + TX2_INDEX);

        WRITE_U16(dataPtr, new_var->tx2_control_word);
        WRITE_U8(dataPtr, new_var->tx2_mode_of_operation);
        WRITE_U32(dataPtr, new_var->tx2_target_position);
        WRITE_U32(dataPtr, new_var->tx2_velocity);
        WRITE_U32(dataPtr, new_var->tx2_target_acceleration);
        WRITE_U32(dataPtr, new_var->tx2_target_deceleration);

    } else if (type == ether_ros::PDOOut::TX3) {
        auto *dataPtr = (process_data_buf + slave_id * (num_process_data_out + num_process_data_in) + TX3_INDEX);

        WRITE_U16(dataPtr, new_var->tx3_control_word);
        WRITE_U8(dataPtr,  new_var->tx3_mode_of_operation);
        WRITE_U32(dataPtr, new_var->tx3_target_acceleration);
        WRITE_U32(dataPtr, new_var->tx3_target_deceleration);
        WRITE_U32(dataPtr, new_var->tx3_target_velocity);

    } else if (type == ether_ros::PDOOut::READONLY) {
        ROS_ERROR("PDOOutListener got a readonly PDO");
    }
}
void PDOOutListener::init(ros::NodeHandle &n)
{
    //Create  ROS subscriber for the Ethercat RAW data
    pdo_out_listener_ = n.subscribe("pdo_listener", 1000, &PDOOutListener::pdo_out_callback, &pdo_out_listener);
}
