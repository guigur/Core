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
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "DataAPI.h"
//------------ZEPHYR DRIVERS----------------------
#include "zephyr/zephyr.h"
#include "zephyr/console/console.h"

#define RECORD_SIZE 128 // Number of point to record

#define LEG1_CAPA_DGND PC6
#define LEG2_CAPA_DGND PB7


//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_application_task();   // Code to be executed in the background task
void loop_communication_task();   // Code to be executed in the background task
void loop_control_task();     // Code to be executed in real time in the critical task

//--------------USER VARIABLES DECLARATIONS----------------------

static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;
uint8_t received_char;
char bufferstr[255]={};

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

int8_t AppTask_num, CommTask_num;

static float32_t acquisition_moment = 0.06;

static float meas_data; // temp storage meas value (ctrl task)

float32_t duty_cycle = 0.3;
float32_t starting_duty_cycle = 0.1;

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

typedef enum
{
    IDLE, SERIAL,
    DUTY, BUCK, BOOST, CAPACITOR, CALIBRATE, DUTY_RESET
} tester_states_t;

uint8_t mode = IDLE;

typedef struct {
    char cmd[16];
    tester_states_t mode;
    void (*func)();
} cmdToState_t;

uint8_t num_defaul_commands = 2;

cmdToState_t default_commands[] = {{"_i", IDLE, NULL}, {"_s", SERIAL, NULL}};
cmdToState_t power_commands[] = {{"_d", DUTY, NULL}, {"_b", BUCK, NULL}, {"_t", BOOST, NULL}, {"_c", CAPACITOR, NULL},{"_k", CALIBRATE, NULL},{"_r", DUTY_RESET, NULL} };

void defaultHandler()
{
    for(uint8_t i = 0; i < num_defaul_commands; i++) //iterates the default commands
    {
        if (strncmp(bufferstr, default_commands[i].cmd, strlen(default_commands[i].cmd)) == 0)
        {
            mode = default_commands[i].mode;
            return;
        }
    }
    printk("unknown default command %s\n", bufferstr);
}


void console_read_line()
{
    uint8_t i;
    for (i = 0; received_char != '\n'; i++)
    {
        received_char = console_getchar();
        if (received_char == '\n')
        {
            if (bufferstr[i-1] == '\r')
            {
                bufferstr[i-1] = '\0';
            }
            else
            {
                bufferstr[i] = '\0';
            }
        }
        else
        {
            bufferstr[i] = received_char;
        }
    }
}


// void powerHandler()
// {
//     if (strncmp(bufferstr, "_TFP_PER", strlen("_TFP_PER")) == 0)
//     {
//         size_t prefixLength = strlen("_TFP_PER");
//         uint16_t parameterValue = getAndConvertParameter(bufferstr + prefixLength, prefixLength);
//         if (parameterValue <= UINT16_MAX)
//         {
//             lcdProgressBar(16, parameterValue);
//         }
//     }
//     else if (strncmp(bufferstr, "_SFP_PER", strlen("_SFP_PER")) == 0)
//     {
//         size_t prefixLength = strlen("_SFP_PER");
//         uint16_t parameterValue = getAndConvertParameter(bufferstr + prefixLength, prefixLength);
//         if (parameterValue <= UINT16_MAX)
//         {
//             lcdProgressBar(16, parameterValue);
//         }
//     }
// }

// uint16_t getAndConvertParameter(const char *inputString, size_t prefixLength)
// {
//     // Find the position of the space character
//     char *spacePosition = strchr(inputString, ':');

//     // Check if space is found and extract the parameter
//     if (spacePosition != NULL)
//     {
//         // Calculate the length of the parameter substring
//         size_t parameterLength = strlen(spacePosition + 1);

//         // Allocate memory for the parameter substring
//         char parameter[parameterLength + 1];

//         // Copy the parameter substring
//         strncpy(parameter, spacePosition + 1, parameterLength);
//         parameter[parameterLength] = '\0'; // Null-terminate the string

//         // Convert the parameter to an integer
//         char *endptr;
//         long int intValue = strtol(parameter, &endptr, 10);

//         // Check if the conversion was successful
//         if (*endptr == '\0') {
//             // Check if the integer fits within uint16_t range
//             if (intValue >= 0 && intValue <= UINT16_MAX)
//             {
//                 return (uint16_t)intValue;
//             }
//             else
//             {
//                 printk("The parameter is not within the valid uint16_t range.\n");
//             }
//         }
//         else
//         {
//             printk("The parameter is not a valid integer.\n");
//         }
//     }
//     else
//     {
//         printk("No parameter found after the space.\n");
//     }

//     // Return an invalid value (you can handle this case as needed in your program)
//     return UINT16_MAX + 1;
// }




//---------------SETUP FUNCTIONS----------------------------------

void setup_routine()
{
    data.enableTwistDefaultChannels();
    spin.version.setBoardVersion(SPIN_v_0_9);
    twist.initAllBuck(); // initialize in buck mode

    spin.gpio.configurePin(LEG1_CAPA_DGND, OUTPUT);
    spin.gpio.configurePin(LEG2_CAPA_DGND, OUTPUT);

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

    data.setParameters(V1_LOW, GV1, OV1);
    data.setParameters(V2_LOW, GV2, OV2);
    data.setParameters(V_HIGH, GVH, OVH);

    data.setParameters(I1_LOW, GI1, OI1);
    data.setParameters(I2_LOW, GI2, OI2);
    data.setParameters(I_HIGH, GIH, OIH);

    spin.gpio.setPin(LEG1_CAPA_DGND);
    spin.gpio.setPin(LEG2_CAPA_DGND);

    AppTask_num = task.createBackground(loop_application_task);
    CommTask_num = task.createBackground(loop_communication_task);
    task.createCritical(&loop_control_task, control_task_period);

    task.startBackground(AppTask_num);
    task.startBackground(CommTask_num);
    task.startCritical();
}

//---------------LOOP FUNCTIONS----------------------------------

void loop_communication_task()
{
    received_char = console_getchar();
    switch (received_char)
    {
    case 'd':
        console_read_line();
        // printk("buffer str = %s\n", bufferstr);
        defaultHandler();
        spin.led.turnOn();
        break;
    // case 'p':
    //     console_read_line();
    //     powerHandler();
    //     // counter = 0;
    //     break;
    // case 'a':
    //     printk("step mode 1\n");
    //     mode = STEPMODE_1;
    //     counter = 0;
    //     spin.gpio.resetPin(LEG1_CAPA_DGND);
    //     spin.gpio.setPin(LEG2_CAPA_DGND);
    //     break;
    // case 's':
    //     printk("step mode 2\n");
    //     mode = STEPMODE_2;
    //     counter = 0;
    //     spin.gpio.setPin(LEG1_CAPA_DGND);
    //     spin.gpio.resetPin(LEG2_CAPA_DGND);
    //     break;
    // case 'w':
    //     spin.gpio.setPin(LEG1_CAPA_DGND);
    //     spin.gpio.setPin(LEG2_CAPA_DGND);
    //     break;
    // case 'x':
    //     spin.gpio.resetPin(LEG1_CAPA_DGND);
    //     spin.gpio.resetPin(LEG2_CAPA_DGND);
    //     break;
    // case 'u':
    //     duty_cycle += 0.01;
    //     break;
    // case 'd':
    //     duty_cycle -= 0.01;
    //     break;
    default:
        break;
    }
}

void loop_application_task()
{
    switch(mode)
    {
        case IDLE:
            spin.led.turnOff();
            printk("IDLE mode \n");
            break;
        case SERIAL:
            spin.led.turnOn();
            printk("SERIAL mode \n");
            break;
        case DUTY_RESET:
            printk("DUTY RESET mode \n");
            duty_cycle = starting_duty_cycle;
            break;
        default:
            break;
        // else if (mode == STEPMODE_1 || mode == STEPMODE_2)
        // {
            // spin.led.toggle();
            // if(counter >= RECORD_SIZE){
            //     printk("%f:", record_array[print_counter].I1_low_value);
            //     printk("%f:", record_array[print_counter].I2_low_value);
            //     printk("%f:", record_array[print_counter].Ihigh_value);
            //     printk("%f:", record_array[print_counter].V1_low_value);
            //     printk("%f:", record_array[print_counter].V2_low_value);
            //     printk("%f:", record_array[print_counter].Vhigh_value);
            //     printk("%d:", print_counter);
            //     print_counter++;
            //     if(print_counter > 30) print_counter = 0;
            // }
        // }
    }

    // acquisition_moment = acquisition_moment + 0.01;
    // if(acquisition_moment > 0.99) acquisition_moment = 0.02;
    // spin.setHrtimAdcTrigInterleaved(acquisition_moment);
    // printk("%f:", acquisition_moment);

    // printk("%f:", duty_cycle);
    // printk("%f:", V1_max);
    // printk("%f:", V2_max);
    // printk("%f:", I_high);
    // printk("%f:", I1_low_value);
    // printk("%f:", I2_low_value);
    // printk("%f:", V_high);
    // printk("%f:", V1_low_value);
    // printk("%f\n", V2_low_value);

    task.suspendBackgroundMs(100);
}

void loop_control_task()
{
    meas_data = data.getLatest(V1_LOW);
    if (meas_data != NO_VALUE)
        V1_low_value = meas_data;

    meas_data = data.getLatest(V2_LOW);
    if (meas_data != NO_VALUE)
        V2_low_value = meas_data;

    meas_data = data.getLatest(V_HIGH);
    if (meas_data != NO_VALUE)
        I1_low_value = meas_data;

    meas_data = data.getLatest(I1_LOW);
    if (meas_data != NO_VALUE)
        I2_low_value = meas_data;

    meas_data = data.getLatest(I2_LOW);
    if (meas_data != NO_VALUE)
        V_high = meas_data;

    meas_data = data.getLatest(I_HIGH);
    if (meas_data != NO_VALUE)
        I_high = meas_data;


    switch(mode){
        case IDLE:
            pwm_enable = false;
            twist.stopLeg(LEG1);
            twist.stopLeg(LEG2);
            break;
    }

    // }
    // else if (mode == POWERMODE ||  mode == STEPMODE_1 || mode == STEPMODE_2)
    // {

    //     // spin.setInterleavedDutyCycle(duty_cycle);
    //     if(mode == STEPMODE_1) twist.setLegDutyCycle(LEG1,duty_cycle);
    //     if(mode == STEPMODE_2) twist.setLegDutyCycle(LEG2,duty_cycle);
    //     if (!pwm_enable)
    //     {
    //         pwm_enable = true;
    //         if(mode == STEPMODE_1) twist.startLeg(LEG1);
    //         if(mode == STEPMODE_2) twist.startLeg(LEG2);
    //         // spin.setInterleavedOn();
    //         counter = 0;
    //         V1_max  = 0;
    //         V2_max  = 0;
    //     }

    //     if(mode == STEPMODE_1){
    //         if(counter<RECORD_SIZE/2) spin.gpio.resetPin(LEG1_CAPA_DGND);
    //         record_array[counter].I1_low_value = I1_low_value;
    //         record_array[counter].V1_low_value = V1_low_value;
    //     }
    //     if(mode == STEPMODE_2) {
    //         record_array[counter].V2_low_value = V2_low_value;
    //         record_array[counter].I2_low_value = I2_low_value;
    //     }

    //     record_array[counter].Ihigh_value = I_high;
    //     record_array[counter].Vhigh_value = V_high;

    //     if(V1_low_value>V1_max) V1_max = V1_low_value;
    //     if(V2_low_value>V2_max) V2_max = V2_low_value;

    //     if(counter < RECORD_SIZE) counter++;
    // }
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
