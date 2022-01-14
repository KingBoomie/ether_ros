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
   \file ethercat_slave.cpp
   \brief Implementation of EthercatSlave class.

   Used for containing all the useful information
   of an EtherCAT slave, from the userspace program perspective. Receives all the
   useful information via the ROS Parameter Server (after they are loaded from ethercat_slaves.yaml).
*/

/*****************************************************************************/
#include <iostream>
#include "ethercat_slave.h"
#include "ether_ros.h"

/* Master 0, Slave 0, "ECR60V202"
 * Vendor ID:       0x00000a88
 * Product code:    0x0a880001
 * Revision number: 0x00000202
 */

ec_pdo_entry_info_t pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Control Word */
    {0x6060, 0x00, 8}, /* ModeOfOperation */
    {0x607a, 0x00, 32}, /* Profile Target Position */
    {0x6081, 0x00, 32}, /* Profile Velocity */
    {0x6083, 0x00, 32}, /* Profile Target Acceleration */
    {0x6084, 0x00, 32}, /* Profile Target Deceleration */

    {0x6041, 0x00, 16}, /* Status Word */
    {0x6061, 0x00, 8}, /* Modes of Operation display */
    {0x6064, 0x00, 32}, /* Position Actual Value */
    {0x60FD, 0x00, 32}, /* Digital Inputs */
};

ec_pdo_entry_info_t pdo_entries_tx3[] = {
    {0x6040, 0x00, 16}, /* Control Word */
    {0x6060, 0x00, 8}, /* ModeOfOperation */
    {0x6083, 0x00, 32}, /* Profile Target Acceleration */
    {0x6084, 0x00, 32}, /* Profile Target Deceleration */
    {0x60FF, 0x00, 32}, /* Target Velocity */
};

ec_pdo_entry_info_t pdo_entries_rx2[] = {
    {0x6041, 0x00, 16}, /* Status Word */
    {0x6061, 0x00, 8}, /* Modes of Operation display */
    {0x606C, 0x00, 32}, /* Velocity Actual Value */
    {0x60FD, 0x00, 32}, /* Digital Inputs */
};

ec_pdo_info_t pdos[] = {
    {0x1600, 3, pdo_entries + 0}, /* Receive PDO 1 */
    {0x1601, 6, pdo_entries + 0}, /* Receive PDO 2 */
    {0x1602, 5, pdo_entries_tx3}, /* Receive PDO 3 */
    {0x1a00, 4, pdo_entries + 6}, /* Transmit PDO 1 */
    {0x1a01, 4, pdo_entries_rx2}, /* Transmit PDO 2 */
};

ec_sync_info_t syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_ENABLE},
    {2, EC_DIR_OUTPUT, 3, pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, pdos + 3, EC_WD_ENABLE},
    {0xff}
};

void EthercatSlave::init(std::string slave, ros::NodeHandle &n)
{

    slave_id_ = slave;
    std::string slave_root_loc = std::string("/ethercat_slaves/") + slave + "/";

    //<editor-fold desc="Parameters">
    while (!n.getParam(slave_root_loc + "vendor_id", vendor_id_))
    {
        ROS_INFO("Waiting the parameter server to initialize\n");
    }
    ROS_INFO("Got param: slave_root_loc + vendor_id = %2.2x\n", vendor_id_);

    if (n.getParam(slave_root_loc + "alias", alias_))
    {
        ROS_INFO("Got param: slave_root_loc + alias = %d\n", alias_);
    }
    else
    {
        ROS_FATAL("Failed to get param 'slave_root_loc + alias'\n");
    }

    if (n.getParam(slave_root_loc + "position", position_))
    {
        ROS_INFO("Got param: slave_root_loc + position = %d\n", position_);
    }
    else
    {
        ROS_FATAL("Failed to get param 'slave_root_loc + position'\n");
    }

    if (n.getParam(slave_root_loc + "product_code", product_code_))
    {
        ROS_INFO("Got param: slave_root_loc + product_code = %2.2x\n", product_code_);
    }
    else
    {
        ROS_FATAL("Failed to get param 'slave_root_loc + product_code'\n");
    }

    if (n.getParam(slave_root_loc + "assign_activate", assign_activate_))
    {
        ROS_INFO("Got param: slave_root_loc + assign_activate = %2.2x\n", assign_activate_);
    }
    else
    {
        ROS_FATAL("Failed to get param 'slave_root_loc + assign_activate'\n");
    }

    if (n.getParam(slave_root_loc + "input_port", input_port_))
    {
        ROS_INFO("Got param: slave_root_loc + input_port = %2.2x\n", input_port_);
    }
    else
    {
        ROS_FATAL("Failed to get param 'slave_root_loc + input_port'\n");
    }

    if (n.getParam(slave_root_loc + "output_port", output_port_))
    {
        ROS_INFO("Got param: slave_root_loc + output_port = %2.2x\n", output_port_);
    }
    else
    {
        ROS_FATAL("Failed to get param 'slave_root_loc + output_port'\n");
    }
    //</editor-fold>

    ethercat_slave_ = ecrt_master_slave_config(master, alias_, position_, vendor_id_, product_code_);

    if (!ethercat_slave_)
    {
        ROS_FATAL("Failed to get slave configuration.\n");
        exit(1);
    }
//    ECR60_remap(ethercat_slave_);
    // manual PDO configuration
    
    int retval = ecrt_slave_config_pdos(ethercat_slave_, EC_END, syncs);
    if (retval < 0) {
      ROS_FATAL("Failed to configure PDOs.\n");
      exit(1);
    }

    ecrt_slave_config_sdo32(ethercat_slave_, 0x2000, 0x00, 1000);

    pdo_out_ = ecrt_slave_config_reg_pdo_entry(ethercat_slave_, output_port_, 0, domain1, NULL);
    ROS_INFO("PDO out config %d.\n", pdo_out_);
    if (pdo_out_ < 0)
    {
        ROS_FATAL("Failed to configure pdo out.\n");
        exit(1);
    }
    ROS_INFO("Offset pdo out is: %d\n", pdo_out_);

    pdo_in_ = ecrt_slave_config_reg_pdo_entry(ethercat_slave_, input_port_, 0, domain1, NULL);
    if (pdo_in_ < 0)
    {
        ROS_FATAL("Failed to configure pdo in.\n");
        exit(1);
    }
    ROS_INFO("Offset pdo in is: %d\n", pdo_in_);

    if (n.getParam("/ethercat_slaves/sync0_shift", sync0_shift_))
    {
        ROS_INFO("Got param: /ethercat_slaves/sync0_shift = %2.2x\n", sync0_shift_);
    }
    else
    {
        ROS_FATAL("Failed to get param '/ethercat_slaves/sync0_shift'\n");
    }
    // configure SYNC signals for this slave
    //For XMC use: 0x0300
    //For Beckhoff FB1111 use: 0x0700
    //Use PERIOD_NS as the period, and 50 μs shift time
    ecrt_slave_config_dc(ethercat_slave_, assign_activate_, PERIOD_NS, sync0_shift_, 0, 0);
}

int EthercatSlave::get_pdo_in()
{
    return pdo_in_;
}

int EthercatSlave::get_pdo_out()
{
    return pdo_out_;
}
ec_slave_config_t *EthercatSlave::get_slave_config()
{
    return ethercat_slave_;
}
