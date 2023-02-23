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

//-------------OWNTECH DRIVERS-------------------
#include "HardwareConfiguration.h"
#include "DataAcquisition.h"
#include "Scheduling.h"

//------------ZEPHYR DRIVERS----------------------
#include "zephyr.h"
#include "console/console.h"

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_background_task();   // Code to be executed in the background task
void loop_critical_task();     // Code to be executed in real time in the critical task

//--------------USER VARIABLES DECLARATIONS----------------------

static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;

static float meas_data; // temp storage meas value (ctrl task)

float32_t duty_cycle = 0.3;

/* Variables used for recording */
typedef struct Record_master {
    float32_t I1_low_value;
    float32_t I2_low_value;
    float32_t V1_low_value;
    float32_t V2_low_value;
    float32_t Vhigh_value;
    float32_t Ihigh_value;
} record_t;

record_t record_array[RECORD_SIZE];

static uint32_t counter = 0;

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

//---------------SETUP FUNCTIONS----------------------------------

void setup_hardware()
{
    hwConfig.configureAdcDefaultAllMeasurements();
    console_init();
    hwConfig.setBoardVersion(TWIST_v_1_1_2);
    hwConfig.initInterleavedBuckMode(); // initialize in buck mode
}

void setup_software()
{
    AppTask_num = scheduling.defineAsynchronousTask(loop_application_task);
    CommTask_num = scheduling.defineAsynchronousTask(loop_communication_task);
    scheduling.defineUninterruptibleSynchronousTask(&loop_control_task, control_task_period);

    scheduling.startAsynchronousTask(AppTask_num);
    scheduling.startAsynchronousTask(CommTask_num);
    scheduling.startUninterruptibleSynchronousTask();
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
            printk("|     press s : serial mode              |\n");
            printk("|     press p : power mode               |\n");
            printk("|     press u : duty cycle UP            |\n");
            printk("|     press d : duty cycle DOWN          |\n");
            printk("|________________________________________|\n\n");
            //------------------------------------------------------
            break;
        case 'i':
            printk("idle mode\n");
            mode = IDLEMODE;
            break;
        case 'p':
            printk("power mode\n");
            mode = POWERMODE;
            counter = 0;
            break;
        case 'u':
            duty_cycle += 0.01;
            break;
        case 'd':
            duty_cycle -= 0.01;
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
            hwConfig.setLedOff();
        }
        else if (mode == POWERMODE)
        {
            hwConfig.setLedOn();
            printk("%f:", V_high);
            printk("%f:", I_high);
            printk("%f:", I1_low_value);
            printk("%f:", I2_low_value);
            printk("%f:", V1_low_value);
            printk("%f\n", V2_low_value);
        }
        k_msleep(100);
    }
}

void loop_control_task()
{
    meas_data = dataAcquisition.getV1Low();
    if (meas_data != -10000)
        V1_low_value = meas_data;

    meas_data = dataAcquisition.getV2Low();
    if (meas_data != -10000)
        V2_low_value = meas_data;

    meas_data = dataAcquisition.getI1Low();
    if (meas_data != -10000)
        I1_low_value = meas_data;

    meas_data = dataAcquisition.getI2Low();
    if (meas_data != -10000)
        I2_low_value = meas_data;

    meas_data = dataAcquisition.getVHigh();
    if (meas_data != -10000)
        V_high = meas_data;

    meas_data = dataAcquisition.getIHigh();
    if (meas_data != -10000)
        I_high = meas_data;

    if (mode == IDLEMODE)
    {
        pwm_enable = false;
        hwConfig.setInterleavedOff();
    }
    else if (mode == POWERMODE)
    {

        hwConfig.setInterleavedDutyCycle(duty_cycle);
        if (!pwm_enable)
        {
            pwm_enable = true;
            hwConfig.setInterleavedOn();
        }

        record_array[counter].I1_low_value = I1_low_value;
        record_array[counter].I2_low_value = I2_low_value;
        record_array[counter].V1_low_value = V1_low_value;
        record_array[counter].V2_low_value = V2_low_value;
        record_array[counter].Ihigh_value = I_high;
        record_array[counter].Vhigh_value = V_high;

        if(counter < RECORD_SIZE) counter++;
    }
}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */
int main(void)
{
    setup_routine();

    return 0;
}
