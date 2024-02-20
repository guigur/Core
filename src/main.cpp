/*
 * Copyright (c) 2021-2024 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGLPV2.1
 */

/**
 * @brief  This file it the main entry point of the
 *         OwnTech Power API. Please check the OwnTech
 *         documentation for detailed information on
 *         how to use Power API: https://docs.owntech.org/
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

//--------------OWNTECH APIs----------------------------------
#include "DataAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "SpinAPI.h"
#include "Rs485Communication.h"
#include "SyncCommunication.h"

#include "zephyr/console/console.h"

//----------- USER INCLUDE ----------------------

#define GET_ID(x) ((x >> 6) & 0x3)        // retrieve identifiant
#define GET_STATUS(x) (x & 1) // check the status (IDLE MODE or POWER MODE)

//-------------LOOP FUNCTIONS DECLARATION----------------------
void loop_communication_task(); // code to be executed in the slow communication task
int8_t CommTask_num;            // Communication Task number
void loop_application_task();   // code to be executed in the fast application task
int8_t AppTask_num;             // Application Task number
void loop_control_task();       // code to be executed in real-time at 20kHz

//--------------USER VARIABLES DECLARATIONS----------------------

static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;

struct consigne_struct
{
    uint8_t buf_vab[3];    // Contains Voltage DATA A and Voltage DATA B
    uint8_t test_RS485;    // variable for testing RS485
    uint8_t test_Sync;    // variable for testing Sync
    uint16_t analog_value_measure; // Contains analog measure
    uint8_t id_and_status; // Contains status
};

// Future work : replace union
struct consigne_struct tx_consigne;
struct consigne_struct rx_consigne;
uint8_t* buffer_tx = (uint8_t*)&tx_consigne;
uint8_t* buffer_rx =(uint8_t*)&rx_consigne;

uint8_t status;
uint32_t counter_time = 0;

/* analog test parameters*/
uint16_t analog_value;
uint16_t analog_value_ref = 1000;

/* RS485 test parameters*/
static uint8_t rs485_send;
static uint8_t rs485_receive;

/* Sync test parameters*/
static uint8_t sync_master_counter = 0;

/* BOOL value for testing */
static bool test_start = false; // start the test after a certain period of time
static bool RS485_success = false;
static bool Sync_success = false;
static bool Analog_success = false;
static bool Can_success = false;

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE,
};

uint8_t mode = IDLEMODE;

void reception_function(void)
{
    if ( GET_ID(rx_consigne.id_and_status) == 2)
    {
        analog_value = rx_consigne.analog_value_measure;
        rs485_receive = rx_consigne.test_RS485;
    }

    if(test_start && (rs485_receive == rs485_send + 1)) RS485_success = true;

    if(test_start && RS485_success && (analog_value - analog_value_ref > 50 || analog_value - analog_value_ref > -50)) Analog_success = true;

    if(test_start && RS485_success && (sync_master_counter < 5 && rx_consigne.test_Sync == 10))
    {
        sync_master_counter++;
        if(sync_master_counter == 5) Sync_success = true;

    }
}

//---------------SETUP FUNCTIONS----------------------------------

void setup_hardware()
{
    console_init();

    spin.version.setBoardVersion(TWIST_v_1_1_2);

    twist.setVersion(shield_TWIST_V1_3);
    twist.initAllBuck(); // initialize in buck mode leg1 and leg2
    syncCommunication.initMaster(); // start the synchronisation
}

void setup_software()
{
    data.enableTwistDefaultChannels();

    spin.dac.initConstValue(2);

    task.createCritical(&loop_control_task, control_task_period);
    task.startCritical();

    CommTask_num = task.createBackground(loop_communication_task);
    AppTask_num =  task.createBackground(loop_application_task);

    task.startBackground(CommTask_num);
    task.startBackground(AppTask_num);

    rs485Communication.configure(buffer_tx, buffer_rx, sizeof(consigne_struct), reception_function, 10625000, true); // custom configuration for RS485
}

//---------------LOOP FUNCTIONS----------------------------------

void loop_communication_task()
{
    while (1)
    {
        received_serial_char = console_getchar();
        switch (received_serial_char)
        {
        case 'h':
            //----------SERIAL INTERFACE MENU-----------------------
            printk(" ________________________________________\n");
            printk("|     ------- MENU ---------             |\n");
            printk("|     press i : idle mode                |\n");
            printk("|     press p : power mode               |\n");
            printk("|________________________________________|\n\n");
            //------------------------------------------------------
            break;
        case 't':
            printk("%u:", RS485_success);
            printk("%u:", Sync_success);
            printk("%u:", Analog_success);
            printk("%u\n:", Can_success);
            break;
        case 'i':
            printk("idle mode\n");
            mode = IDLEMODE;
            break;
        case 'p':
            printk("power mode\n");
            mode = POWERMODE;
            break;
        case 'l':
            analog_value_ref += 100;
            break;
        case 'm':
            analog_value_ref -= 100;
            break;
        default:
            break;
        }
    }
}

void loop_application_task()
{
    while (1)
    {

        if (mode == IDLEMODE)
        {
            spin.led.turnOff();
        }
        else if (mode == POWERMODE)
        {
            spin.led.turnOn();
        }
        k_msleep(100);
    }
}

void loop_control_task()
{

    spin.dac.setConstValue(2, 1, analog_value_ref);

    if (mode == IDLEMODE)
    {
        if (pwm_enable == true)
        {
            tx_consigne.id_and_status = (1 << 6) + 0;
            rs485_send = 0;
            tx_consigne.test_RS485 = rs485_send ;
            rs485Communication.startTransmission();
        }
        pwm_enable = false;
    }
    else if (mode == POWERMODE)
    {
        pwm_enable = true;

        /* Voltage reference */
        rs485_send++;

        /* writting rs485 value */
        tx_consigne.test_RS485 = rs485_send;

        tx_consigne.id_and_status = (1 << 6) + 1;

        rs485Communication.startTransmission();

        counter_time++;
        if (counter_time > 50) test_start =  true;

    }

}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */

int main(void)
{
    setup_hardware();
    setup_software();

    return 0;
}