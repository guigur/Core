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

#define RECORD_SIZE 128 // Number of point to record

#define LEG1_CAPA_DGND PC6
#define LEG2_CAPA_DGND PB7


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

static float32_t delta_V1;
static float32_t V1_max = 0.0;
static float32_t V1_min = 0.0;

static float32_t delta_V2;
static float32_t V2_max = 0.0;
static float32_t V2_min = 0.0;


static float32_t acquisition_moment = 0.06;

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
    float32_t V1_low_value_no_cap;
    float32_t V2_low_value_no_cap;
} record_t;

record_t record_array[RECORD_SIZE];

static uint32_t counter = 0;
static uint32_t print_counter = 0;

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    STEPMODE_1,
    STEPMODE_2,
    POWERMODE
};

uint8_t mode = IDLEMODE;

//---------------SETUP FUNCTIONS----------------------------------

void setup_hardware()
{
    hwConfig.configureAdcDefaultAllMeasurements();
    console_init();
    hwConfig.setBoardVersion(SPIN_v_0_9);
    hwConfig.initInterleavedBuckMode(); // initialize in buck mode

    gpio.configurePin(LEG1_CAPA_DGND, OUTPUT);
    gpio.configurePin(LEG2_CAPA_DGND, OUTPUT);

    float32_t GV1 = 0.044301359147286994;
    float32_t OV1 = -89.8291125470221;
    float32_t GV2 = 0.043891466731813246;
    float32_t OV2 = -89.01321095039089;
    float32_t GVH = 0.029777494874229947;
    float32_t OVH = 0.12805533844297656;

    float32_t GI1 = 0.005510045850270965;
    float32_t OI1 = -11.298753103344417;
    float32_t GI2 = 0.005569903739753797;
    float32_t OI2 = -11.47851441455354;
    float32_t GIH = 0.0052774398156665;
    float32_t OIH = -10.864400298536168;

    dataAcquisition.setV1LowParameters(GV1, OV1);
    dataAcquisition.setV2LowParameters(GV2, OV2);
    dataAcquisition.setVHighParameters(GVH, OVH);

    dataAcquisition.setI1LowParameters(GI1, OI1);
    dataAcquisition.setI2LowParameters(GI2, OI2);
    dataAcquisition.setIHighParameters(GIH, OIH);

    gpio.setPin(LEG1_CAPA_DGND);
    gpio.setPin(LEG2_CAPA_DGND);
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
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
    case 'h':
        //----------SERIAL INTERFACE MENU-----------------------
        // printk(" ________________________________________\n");
        // printk("|     ------- MENU ---------             |\n");
        // printk("|     press i : idle mode                |\n");
        // printk("|     press s : serial mode              |\n");
        // printk("|     press p : power mode               |\n");
        // printk("|     press u : duty cycle UP            |\n");
        // printk("|     press d : duty cycle DOWN          |\n");
        // printk("|________________________________________|\n\n");
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
    case 'a':
        printk("step mode 1\n");
        mode = STEPMODE_1;
        counter = 0;
        gpio.resetPin(LEG1_CAPA_DGND);
        gpio.setPin(LEG2_CAPA_DGND);
        break;
    case 's':
        printk("step mode 2\n");
        mode = STEPMODE_2;
        counter = 0;
        gpio.setPin(LEG1_CAPA_DGND);
        gpio.resetPin(LEG2_CAPA_DGND);
        break;
    case 'w':
        gpio.setPin(LEG1_CAPA_DGND);
        gpio.setPin(LEG2_CAPA_DGND);
        break;
    case 'x':
        gpio.resetPin(LEG1_CAPA_DGND);
        gpio.resetPin(LEG2_CAPA_DGND);
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

void loop_application_task()
{
    if (mode == IDLEMODE)
    {
        hwConfig.setLedOff();
    }
    else if (mode == POWERMODE)
    {
        hwConfig.setLedOn();
    }
    else if (mode == STEPMODE_1 || mode == STEPMODE_2)
    {
        hwConfig.setLedToggle();
        if(counter >= RECORD_SIZE){
            printk("%f:", record_array[print_counter].I1_low_value);
            printk("%f:", record_array[print_counter].I2_low_value);
            printk("%f:", record_array[print_counter].Ihigh_value);
            printk("%f:", record_array[print_counter].V1_low_value);
            printk("%f:", record_array[print_counter].V2_low_value);
            printk("%f:", record_array[print_counter].Vhigh_value);
            printk("%d:", print_counter);
            print_counter++;
            if(print_counter > 30) print_counter = 0;
        }
    }


    // acquisition_moment = acquisition_moment + 0.01;
    // if(acquisition_moment > 0.99) acquisition_moment = 0.02;
    // hwConfig.setHrtimAdcTrigInterleaved(acquisition_moment);
    // printk("%f:", acquisition_moment);

    printk("%f:", duty_cycle);
    printk("%f:", V1_max);
    printk("%f:", V2_max);
    printk("%f:", I_high);
    printk("%f:", I1_low_value);
    printk("%f:", I2_low_value);
    printk("%f:", V_high);
    printk("%f:", V1_low_value);
    printk("%f\n", V2_low_value);

    scheduling.suspendCurrentTaskMs(100);
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

    // if(V1_low_value<V1_min) V1_min = V1_low_value;
    // if(V1_low_value>V1_max) V1_max = V1_low_value;
    // delta_V1 = V1_max - V1_min;

    // if(V2_low_value<V2_min) V2_min = V2_low_value;
    // if(V2_low_value>V2_max) V2_max = V2_low_value;
    // delta_V2 = V2_max - V2_min;


    if (mode == IDLEMODE)
    {
        pwm_enable = false;
        hwConfig.setLeg1Off();
        hwConfig.setLeg2Off();

    }
    else if (mode == POWERMODE ||  mode == STEPMODE_1 || mode == STEPMODE_2)
    {

        // hwConfig.setInterleavedDutyCycle(duty_cycle);
        if(mode == STEPMODE_1) hwConfig.setLeg1DutyCycle(duty_cycle);
        if(mode == STEPMODE_2) hwConfig.setLeg2DutyCycle(duty_cycle);
        if (!pwm_enable)
        {
            pwm_enable = true;
            if(mode == STEPMODE_1) hwConfig.setLeg1On();
            if(mode == STEPMODE_2) hwConfig.setLeg2On();
            // hwConfig.setInterleavedOn();
            counter = 0;
            V1_max  = 0;
            V2_max  = 0;
        }

        if(mode == STEPMODE_1){
            if(counter<RECORD_SIZE/2) gpio.resetPin(LEG1_CAPA_DGND);
            record_array[counter].I1_low_value = I1_low_value;
            record_array[counter].V1_low_value = V1_low_value;
        }
        if(mode == STEPMODE_2) {
            record_array[counter].V2_low_value = V2_low_value;
            record_array[counter].I2_low_value = I2_low_value;
        }

        record_array[counter].Ihigh_value = I_high;
        record_array[counter].Vhigh_value = V_high;

        if(V1_low_value>V1_max) V1_max = V1_low_value;
        if(V2_low_value>V2_max) V2_max = V2_low_value;

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
