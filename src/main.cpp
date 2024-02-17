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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define RECORD_SIZE 128 // Number of point to record

#define LEG1_CAPA_DGND PC6
#define LEG2_CAPA_DGND PB7

#define BOOL_SETTING_OFF 0
#define BOOL_SETTING_ON 1

#define LEG_1 0
#define LEG_2 1



//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_application_task();   // Code to be executed in the background task
void loop_communication_task();   // Code to be executed in the background task
void loop_control_task();     // Code to be executed in real time in the critical task

//----------------SERIAL PROTOCOL HANDLER FUNCTIONS---------------------
void console_read_line();
void defaultHandler();
void powerHandler();
void powerLegSettingsHandler();

void boolSettingsHandler(uint8_t power_leg, uint8_t setting_position);
void dutyHandler(uint8_t power_leg, uint8_t setting_position);
void referenceHandler(uint8_t power_leg, uint8_t setting_position);

//--------------USER VARIABLES DECLARATIONS----------------------



static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;
uint8_t received_char;
char bufferstr[255]={};
bool print_done = false;


/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high_value;
static float32_t V_high_value;

static float32_t delta_V1;
static float32_t V1_max = 0.0;
static float32_t V1_min = 0.0;

static float32_t delta_V2;
static float32_t V2_max = 0.0;
static float32_t V2_min = 0.0;

int8_t AppTask_num, CommTask_num;

static float32_t acquisition_moment = 0.06;

static float meas_data; // temp storage meas value (ctrl task)

// float32_t duty_cycle = 0.3;
float32_t starting_duty_cycle = 0.1;

// float32_t *tracking_variable=NULL;
float32_t reference_value = 0.0;

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

typedef enum
{
    IDLE, POWER_ON, POWER_OFF
} tester_states_t;

uint8_t mode = IDLE;

// Define a struct to hold the tracking variables and their names
typedef struct {
    const char *name;
    float32_t *address;
} TrackingVariables;

// Declare the tracking variable struct
TrackingVariables tracking_vars[] = {
    {"V1", &V1_low_value},
    {"V2", &V2_low_value},
    {"VH", &V_high_value},
    {"I1", &I1_low_value},
    {"I2", &I2_low_value},
    {"IH", &I_high_value}
};
uint8_t num_tracking_vars = 6;

// Define a struct to hold the settings of each power leg
typedef struct {
    // bool leg_on    - to define if the leg is ON or OFF
    // bool capa_on   - to define if the capacitor of the leg is ON or OFF
    // bool driver_on - to define if the driver of the leg is ON or OFF
    // bool buck_mode - to define is the leg is in buck mode or boost mode
    bool settings[4];
    float32_t *tracking_variable;
    float32_t reference_value;
    float32_t duty_cycle;
} PowerLegSettings;

PowerLegSettings power_leg_settings[] = {
    //   LEG_OFF      ,   CAPA_OFF      , DRIVER_OFF      ,  BUCK_MODE_ON
    {{BOOL_SETTING_OFF, BOOL_SETTING_OFF, BOOL_SETTING_OFF,  BOOL_SETTING_ON},  &V1_low_value, reference_value, starting_duty_cycle},
    {{BOOL_SETTING_OFF, BOOL_SETTING_OFF, BOOL_SETTING_OFF,  BOOL_SETTING_ON},  &V2_low_value, reference_value, starting_duty_cycle}
};

typedef struct {
    char cmd[16];
    void (*func)(uint8_t power_leg, uint8_t setting_position);
} cmdToSettings_t;

cmdToSettings_t power_settings[] = {
    {"_l", boolSettingsHandler},
    {"_c", boolSettingsHandler},
    {"_v", boolSettingsHandler},
    {"_b", boolSettingsHandler},
    {"_r", referenceHandler},
    {"_d", dutyHandler},
};

uint8_t num_power_settings =  sizeof(power_settings)/sizeof(power_settings[0]);


typedef struct {
    char cmd[16];
    tester_states_t mode;
    void (*func)();
} cmdToState_t;

cmdToState_t default_commands[] = {
    {"_i", IDLE, NULL},
    {"_f", POWER_OFF, NULL},
    {"_o", POWER_ON, NULL},
};

uint8_t num_default_commands = sizeof(default_commands)/sizeof(default_commands[0]);



//---------------------------------------------------------------


void console_read_line()
{
    for (uint8_t i = 0; received_char != '\n'; i++)
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


void dutyHandler(uint8_t power_leg, uint8_t setting_position) {
    // Check if the bufferstr starts with "_d_"
    if (strncmp(bufferstr, "_LEG1_d_", 8) == 0 || strncmp(bufferstr, "_LEG2_d_", 8) == 0) {
        // Extract the duty cycle value from the protocol message
        float32_t duty_value = atof(bufferstr + 9);

        // Check if the duty cycle value is within the valid range (0-100)
        if (duty_value >= 0.0 && duty_value <= 1.0) {
            // Update the duty cycle variable
            power_leg_settings[power_leg].duty_cycle = duty_value;
        } else {
            printk("Invalid duty cycle value: %.5f\n", duty_value);
        }
    } else {
        printk("Invalid protocol format: %s\n", bufferstr);
    }
}


void referenceHandler(uint8_t power_leg, uint8_t setting_position){
    const char *underscore1 = strchr(bufferstr + 6, '_');
    if (underscore1 != NULL) {
        // Find the position of the next underscore after the first underscore
        const char *underscore2 = strchr(underscore1 + 1, '_');
        if (underscore2 != NULL) {
            // Extract the variable name between the underscores
            char variable[3];
            strncpy(variable, underscore1 + 1, underscore2 - underscore1 - 1);
            variable[underscore2 - underscore1 - 1] = '\0';

            // Extract the value after the second underscore
            reference_value = atof(underscore2 + 1);

            printk("Variable: %s\n", variable);
            printk("Value: %.5f\n", reference_value);

            // Finds the tracking variable and updates the address of the tracking_variable
            for (uint8_t i = 0; i < num_tracking_vars; i++) {
                if (strcmp(variable, tracking_vars[i].name) == 0) {
                    power_leg_settings[power_leg].tracking_variable = tracking_vars[i].address;
                    break;
                }
            }
        }
    }

}


void boolSettingsHandler(uint8_t power_leg, uint8_t setting_position)
{
    // Check if the command is for turning the leg on
    if (strncmp(bufferstr + 7, "_on", 3) == 0) {
        power_leg_settings[power_leg].settings[setting_position] = BOOL_SETTING_ON;
        // printk("The leg is %d and the setting is %d", power_leg, setting_position);
        // printk("Result %d", power_leg_settings[power_leg].settings[setting_position]);
    }
    // Check if the command is for turning the leg off
    else if (strncmp(bufferstr + 7, "_off", 4) == 0) {
        power_leg_settings[power_leg].settings[setting_position] = BOOL_SETTING_OFF;
        // printk("The leg is %d and the setting is %d", power_leg, setting_position);
        // printk("Result %d", power_leg_settings[power_leg].settings[setting_position]);
    }
    else {
        // Unknown command
        printk("Unknown power command for LEG%d leg\n", power_leg + 1);
    }
}


void defaultHandler()
{
    for(uint8_t i = 0; i < num_default_commands; i++) //iterates the default commands
    {
        if (strncmp(bufferstr, default_commands[i].cmd, strlen(default_commands[i].cmd)) == 0)
        {
            mode = default_commands[i].mode;
            print_done = false; //authorizes printing the current state once
            return;
        }
    }
    printk("unknown default command %s\n", bufferstr);
}


// // Function to parse the power communications
// void powerHandler() {
//     for(uint8_t i = 0; i < num_power_settings; i++) //iterates the default commands
//     {
//         if (strncmp(bufferstr, power_settings[i].cmd, strlen(power_settings[i].cmd)) == 0)
//         {
//             if (power_settings[i].func != NULL)
//             {
//                 power_settings[i].func(); //pointer to a function to do additional stuff if needed
//             }
//             return;
//         }
//     }
//     printk("unknown power command %s\n", bufferstr);
// }


// Function to parse the power leg settings commands
void powerLegSettingsHandler() {

    // Determine the power leg based on the received message
    uint8_t power_leg = LEG_1; // Default to LEG_1
    if (strncmp(bufferstr, "_LEG2_", strlen("_LEG2_")) == 0) {
        power_leg = LEG_2;
    } else if (strncmp(bufferstr, "_LEG1_", strlen("_LEG1_")) != 0) {
        printk("Unknown leg identifier\n");
        return;
    }

    // COMMAND EXTRACTION
    // Find the position of the second underscore after the leg identifier
    const char *underscore2 = strchr(bufferstr + 5, '_');
    if (underscore2 == NULL) {
        printk("Invalid command format\n");
        return;
    }
    // Extract the command part after the leg identifier
    char command[3];
    strncpy(command, underscore2, 2);
    command[2] = '\0';

    // FIND THE HANDLER OF THE SPECIFIC SETTING COMMAND
    for(uint8_t i = 0; i < num_power_settings; i++) //iterates the default commands
    {
        if (strncmp(command, power_settings[i].cmd, strlen(power_settings[i].cmd)) == 0)
        {
            if (power_settings[i].func != NULL)
            {
                power_settings[i].func(power_leg, i); //pointer to the handler function associated with the command
            }
            return;
        }
    }
    printk("unknown power command %s\n", bufferstr);
}


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
        printk("buffer str = %s\n", bufferstr);
        defaultHandler();
        // spin.led.turnOn();
        break;
    // case 'p':
    //     console_read_line();
    //     printk("buffer str = %s\n", bufferstr);
    //     powerHandler();
    //     // counter = 0;
    //     break;
    case 's':
        console_read_line();
        printk("buffer str = %s\n", bufferstr);
        powerLegSettingsHandler();
        // counter = 0;
        break;
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
            if(!print_done) {
                printk("IDLE \n");
                print_done = true;
            }
            break;
        case POWER_OFF:
            spin.led.turnOn();
            if(!print_done) {
                printk("POWER OFF \n");
                print_done = true;
            }
            break;
        case POWER_ON:
            if(!print_done) {
                printk("POWER ON \n");
                print_done = true;
            }
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
        V_high_value = meas_data;

    meas_data = data.getLatest(I_HIGH);
    if (meas_data != NO_VALUE)
        I_high_value = meas_data;


    switch(mode){
        case IDLE:
            pwm_enable = false;
            twist.stopLeg(LEG1);
            twist.stopLeg(LEG2);
            break;
        // case BUCK:
        // case BOOST:
        //     // if (!pwm_enable)
        //     // {
        //     //     pwm_enable = true;
        //     //     if(mode == STEPMODE_1) twist.startLeg(LEG1);
        //     //     if(mode == STEPMODE_2) twist.startLeg(LEG2);
        //     //     // spin.setInterleavedOn();
        //     //     counter = 0;
        //     //     V1_max  = 0;
        //     //     V2_max  = 0;
        //     // }
        //     // if(mode == STEPMODE_1) twist.setLegDutyCycle(LEG1,duty_cycle);
        //     // if(mode == STEPMODE_2) twist.setLegDutyCycle(LEG2,duty_cycle);
        //     break;
        // case CAPACITOR:

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
        default:
            break;
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
