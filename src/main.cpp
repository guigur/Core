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
#include "Rs485Communication.h"
#include "SyncCommunication.h"


#include "opalib_control_pid.h"

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
#define LEG1_DRIVER_SWITCH PC12
#define LEG2_DRIVER_SWITCH PB13

#define BOOL_SETTING_OFF 0
#define BOOL_SETTING_ON 1

#define BOOL_LEG 0
#define BOOL_CAPA 1
#define BOOL_DRIVER 2
#define BOOL_BUCK 3
#define BOOL_BOOST 4

#define CAPA_SWITCH_INDEX 0
#define DRIVER_SWITCH_INDEX 1

#define GET_ID(x) ((x >> 6) & 0x3)        // retrieve identifiant
#define GET_STATUS(x) (x & 1) // check the status (IDLE MODE or POWER MODE)

// #define LEG1 0
// #define LEG2 1



//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_application_task();   // Code to be executed in the background task
void loop_communication_task();   // Code to be executed in the background task
void loop_control_task();     // Code to be executed in real time in the critical task

//----------------SERIAL PROTOCOL HANDLER FUNCTIONS---------------------
void console_read_line();

void defaultHandler();
void powerLegSettingsHandler();

void boolSettingsHandler(uint8_t power_leg, uint8_t setting_position);
void dutyHandler(uint8_t power_leg, uint8_t setting_position);
void referenceHandler(uint8_t power_leg, uint8_t setting_position);
void calibrationHandler();



//--------------USER VARIABLES DECLARATIONS----------------------



static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable_leg_1 = false;            //[bool] state of the PWM (ctrl task)
static bool pwm_enable_leg_2 = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;
uint8_t received_char;
char bufferstr[255]={};
bool print_done = false;

/* PID coefficient for a 8.6ms step response*/
static float32_t kp = 0.000215;
static float32_t ki = 2.86;
static float32_t kd = 0.0;

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
    channel_t channel_reference;
} TrackingVariables;

// Declare the tracking variable struct
TrackingVariables tracking_vars[] = {
    {"V1", &V1_low_value, V1_LOW},
    {"V2", &V2_low_value, V2_LOW},
    {"VH", &V_high_value, V_HIGH},
    {"I1", &I1_low_value, I1_LOW},
    {"I2", &I2_low_value, I2_LOW},
    {"IH", &I_high_value, I_HIGH}
};
uint8_t num_tracking_vars = 6;

// Define a struct to hold the settings of each power leg
// settings ----
// bool leg_on    - to define if the leg is ON or OFF
// bool capa_on   - to define if the capacitor of the leg is ON or OFF
// bool driver_on - to define if the driver of the leg is ON or OFF
// bool buck_mode - to define is the leg is in buck mode
// bool boost_mode - to define is the leg is in boost mode
// switches ----
// pin_t CAPA_SWITCH    - holds the pin of the capa
// pint_t DRIVER_SWITCH - holds the pin of the driver
typedef struct {
    bool settings[5];
    pin_t switches[2];
    float32_t *tracking_variable;
    const char *tracking_var_name;
    float32_t reference_value;
    float32_t duty_cycle;
} PowerLegSettings;

PowerLegSettings power_leg_settings[] = {
    //   LEG_OFF      ,   CAPA_OFF      , DRIVER_OFF      ,  BUCK_MODE_ON,  BOOST_MODE_ON
    {{BOOL_SETTING_OFF, BOOL_SETTING_OFF, BOOL_SETTING_OFF,  BOOL_SETTING_OFF,  BOOL_SETTING_OFF}, {LEG1_CAPA_DGND, LEG1_DRIVER_SWITCH},  &V1_low_value, "V1", reference_value, starting_duty_cycle},
    {{BOOL_SETTING_OFF, BOOL_SETTING_OFF, BOOL_SETTING_OFF,  BOOL_SETTING_OFF,  BOOL_SETTING_OFF}, {LEG2_CAPA_DGND, LEG2_DRIVER_SWITCH},  &V2_low_value, "V2",reference_value, starting_duty_cycle}
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
    {"_t", boolSettingsHandler},
    {"_r", referenceHandler},
    {"_d", dutyHandler},
};

uint8_t num_power_settings =  sizeof(power_settings)/sizeof(power_settings[0]);


typedef struct {
    char cmd[16];
    tester_states_t mode;
} cmdToState_t;

cmdToState_t default_commands[] = {
    {"_i", IDLE},
    {"_f", POWER_OFF},
    {"_o", POWER_ON},
};

uint8_t num_default_commands = sizeof(default_commands)/sizeof(default_commands[0]);

// -------------COMMUNICATION TEST PARAMETERS------------------------------------

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


/* Sync test parameters*/
static uint8_t ctrl_slave_counter = 0;

//---------------------COMM TEST -----------------------------------------------------------
void reception_function(void)
{
    if (GET_ID(rx_consigne.id_and_status) == 1)
    {
        status = rx_consigne.id_and_status;

        tx_consigne = rx_consigne;
        tx_consigne.test_RS485 = rx_consigne.test_RS485 + 1;
        tx_consigne.test_Sync = ctrl_slave_counter;

        tx_consigne.analog_value_measure = analog_value;

        tx_consigne.id_and_status = tx_consigne.id_and_status & ~(1 << 6);
        tx_consigne.id_and_status = tx_consigne.id_and_status | (1 << 7);

        rs485Communication.startTransmission();
    }
}


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
                    power_leg_settings[power_leg].tracking_var_name = tracking_vars[i].name;
                    power_leg_settings[power_leg].reference_value = reference_value;
                    break;
                }
            }
        }
    }

}



void calibrationHandler() {
    const char *underscore1 = strchr(bufferstr + 1, '_');
    if (underscore1 != NULL) {
        // Find the position of the next underscore after the first underscore
        const char *underscore2 = strchr(underscore1 + 1, '_');
        if (underscore2 != NULL) {
            // Extract the variable name between the underscores
            char variable[3];
            strncpy(variable, bufferstr + 1, underscore1 - (bufferstr + 1));
            variable[underscore1 - (bufferstr + 1)] = '\0';

            // Find the position of the next underscore after the second underscore
            const char *underscore3 = strchr(underscore2 + 1, '_');
            if (underscore3 != NULL) {
                // Extract the gain and offset values after the second underscore
                float32_t gain = atof(underscore1 + 3); // Skip 'g_' and parse gain
                float32_t offset = atof(underscore3 + 3); // Skip 'o_' and parse offset

                // Print the parsed values
                printk("Variable: %s\n", variable);
                printk("Gain: %.8f\n", gain);
                printk("Offset: %.8f\n", offset);

                // Find the tracking variable and update its gain and offset
                for (uint8_t i = 0; i < num_tracking_vars; i++) {
                    if (strcmp(variable, tracking_vars[i].name) == 0) {
                        data.setParameters(tracking_vars[i].channel_reference, gain, offset);
                        printk("channel: %s\n", tracking_vars[i].name);
                        printk("channel: %d\n", tracking_vars[i].channel_reference);
                        break;
                    }
                }
                printk("Variable not found: %s\n", variable);
            }
        }
    }
}

void boolSettingsHandler(uint8_t power_leg, uint8_t setting_position)
{
    if (strncmp(bufferstr + 7, "_on", 3) == 0) {
        power_leg_settings[power_leg].settings[setting_position] = BOOL_SETTING_ON;
        if (setting_position == BOOL_CAPA)  spin.gpio.resetPin(power_leg_settings[power_leg].switches[CAPA_SWITCH_INDEX]);  //turns the capacitor switch ON
        if (setting_position == BOOL_DRIVER)  spin.gpio.resetPin(power_leg_settings[power_leg].switches[DRIVER_SWITCH_INDEX]);  //turns the capacitor switch ON
    } else if (strncmp(bufferstr + 7, "_off", 4) == 0) {
        power_leg_settings[power_leg].settings[setting_position] = BOOL_SETTING_OFF;
        if (setting_position == BOOL_CAPA)  spin.gpio.setPin(power_leg_settings[power_leg].switches[CAPA_SWITCH_INDEX]);  //turns the capacitor switch ON
        if (setting_position == BOOL_DRIVER)  spin.gpio.setPin(power_leg_settings[power_leg].switches[DRIVER_SWITCH_INDEX]);  //turns the capacitor switch ON
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


// Function to parse the power leg settings commands
void powerLegSettingsHandler() {

    // Determine the power leg based on the received message
    leg_t power_leg = LEG1; // Default to LEG1
    if (strncmp(bufferstr, "_LEG2_", strlen("_LEG2_")) == 0) {
        power_leg = LEG2;
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
    twist.initLegBuck(LEG1);
    twist.initLegBuck(LEG2);
    syncCommunication.initSlave(); // start the synchronisation

    spin.gpio.configurePin(LEG1_CAPA_DGND, OUTPUT);
    spin.gpio.configurePin(LEG2_CAPA_DGND, OUTPUT);
    spin.gpio.configurePin(LEG1_DRIVER_SWITCH, OUTPUT);
    spin.gpio.configurePin(LEG2_DRIVER_SWITCH, OUTPUT);


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

    opalib_control_init_leg1_pid(kp, ki, kd, control_task_period);
    opalib_control_init_leg2_pid(kp, ki, kd, control_task_period);

    task.startBackground(AppTask_num);
    task.startBackground(CommTask_num);
    task.startCritical();

    rs485Communication.configure(buffer_tx, buffer_rx, sizeof(consigne_struct), reception_function, 10625000, true); // custom configuration for RS485

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
    case 's':
        console_read_line();
        printk("buffer str = %s\n", bufferstr);
        powerLegSettingsHandler();
        // counter = 0;
        break;
    case 'k':
        console_read_line();
        printk("buffer str = %s\n", bufferstr);
        calibrationHandler();
        break;
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
            spin.led.toggle();
            if(!print_done) {
                printk("POWER OFF \n");
                print_done = true;
            }
            printk("[%d,%d,%d,%d,%d]:", power_leg_settings[LEG1].settings[0], power_leg_settings[LEG1].settings[1], power_leg_settings[LEG1].settings[2], power_leg_settings[LEG1].settings[3], power_leg_settings[LEG1].settings[4]);
            printk("%f:", power_leg_settings[LEG1].duty_cycle);
            printk("%f:", power_leg_settings[LEG1].reference_value);
            printk("%s:", power_leg_settings[LEG1].tracking_var_name);
            printk("%f:", tracking_vars[LEG1].address[0]);
            printk("[%d,%d,%d,%d,%d]:", power_leg_settings[LEG2].settings[0], power_leg_settings[LEG2].settings[1], power_leg_settings[LEG2].settings[2], power_leg_settings[LEG2].settings[3], power_leg_settings[LEG2].settings[4]);
            printk("%f:", power_leg_settings[LEG2].duty_cycle);
            printk("%f:", power_leg_settings[LEG2].reference_value);
            printk("%s:", power_leg_settings[LEG2].tracking_var_name);
            printk("%f:", tracking_vars[LEG2].address[0]);
            printk("\n");
            break;
        case POWER_ON:
            spin.led.turnOn();
            if(!print_done) {
                printk("POWER ON \n");
                print_done = true;
            }
            printk("%f:", V1_max);
            printk("%f:", V2_max);
            printk("%f:", V1_low_value);
            printk("%f:", I1_low_value);
            printk("%f:", I2_low_value);
            printk("%f",  V2_low_value);
            printk("%f:", V_high_value);
            printk("%f:", I_high_value);
            printk("\n");

            break;
        default:
            break;
    }

    // acquisition_moment = acquisition_moment + 0.01;
    // if(acquisition_moment > 0.99) acquisition_moment = 0.02;
    // spin.setHrtimAdcTrigInterleaved(acquisition_moment);
    // printk("%f:", acquisition_moment);

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
        case POWER_OFF:
            twist.stopLeg(LEG1);
            twist.stopLeg(LEG2);
            pwm_enable_leg_1 = false;
            pwm_enable_leg_2 = false;
            counter = 0;
            V1_max  = 0;
            V2_max  = 0;
            break;

        case POWER_ON:

            if(!pwm_enable_leg_1 && power_leg_settings[LEG1].settings[BOOL_LEG]) {twist.startLeg(LEG1); pwm_enable_leg_1 = true;}
            if(!pwm_enable_leg_2 && power_leg_settings[LEG2].settings[BOOL_LEG]) {twist.startLeg(LEG2); pwm_enable_leg_2 = true;}

            //calls the pid calculation if the converter in either in mode buck or boost
            if(power_leg_settings[LEG1].settings[BOOL_BUCK] || power_leg_settings[LEG1].settings[BOOL_BOOST])
                power_leg_settings[LEG1].duty_cycle = opalib_control_leg1_pid_calculation(power_leg_settings[LEG1].reference_value , tracking_vars[LEG1].address[0]);

            if(power_leg_settings[LEG2].settings[BOOL_BUCK] || power_leg_settings[LEG2].settings[BOOL_BOOST])
                power_leg_settings[LEG2].duty_cycle = opalib_control_leg1_pid_calculation(power_leg_settings[LEG2].reference_value , tracking_vars[LEG2].address[0]);

            if(power_leg_settings[LEG1].settings[BOOL_LEG]){
                if(power_leg_settings[LEG1].settings[BOOL_BOOST]){
                    twist.setLegDutyCycle(LEG1, (1-power_leg_settings[LEG1].duty_cycle) ); //inverses the convention of the leg in case of changing from buck to boost
                } else {
                    twist.setLegDutyCycle(LEG1, power_leg_settings[LEG1].duty_cycle ); //uses the normal convention by default
                }
            }

            if(power_leg_settings[LEG2].settings[BOOL_LEG]){
                if(power_leg_settings[LEG2].settings[BOOL_BOOST]){
                    twist.setLegDutyCycle(LEG2, (1-power_leg_settings[LEG2].duty_cycle) ); //inverses the convention of the leg in case of changing from buck to boost
                }else{
                    twist.setLegDutyCycle(LEG2, power_leg_settings[LEG2].duty_cycle); //uses the normal convention by default
                }
            }

            if(V1_low_value>V1_max) V1_max = V1_low_value;  //gets the maximum V1 voltage value. This is used for the capacitor test
            if(V2_low_value>V2_max) V2_max = V2_low_value;  //gets the maximum V2 voltage value. This is used for the capacitor test

            break;
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
