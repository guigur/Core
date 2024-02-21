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
#include "test_bench_comm_protocol.h"

#include "opalib_control_pid.h"


#define RECORD_SIZE 128 // Number of point to record


//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_application_task();   // Code to be executed in the background task
void loop_communication_task();   // Code to be executed in the background task
void loop_control_task();     // Code to be executed in real time in the critical task


//--------------USER VARIABLES DECLARATIONS----------------------



static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable_leg_1 = false;            //[bool] state of the PWM (ctrl task)
static bool pwm_enable_leg_2 = false;            //[bool] state of the PWM (ctrl task)


/* PID coefficient for a 8.6ms step response*/
static float32_t kp = 0.000215;
static float32_t ki = 2.86;
static float32_t kd = 0.0;

/* Measure variables */

float32_t V1_low_value;
float32_t V2_low_value;
float32_t I1_low_value;
float32_t I2_low_value;
float32_t I_high_value;
float32_t V_high_value;

 float32_t delta_V1;
 float32_t V1_max = 0.0;
 float32_t V1_min = 0.0;
 float32_t delta_V2;
 float32_t V2_max = 0.0;
 float32_t V2_min = 0.0;

int8_t AppTask_num, CommTask_num;

static float32_t acquisition_moment = 0.06;

static float meas_data; // temp storage meas value (ctrl task)

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


//---------------SETUP FUNCTIONS----------------------------------

void setup_routine()
{
    data.enableTwistDefaultChannels();
    spin.version.setBoardVersion(SPIN_v_0_9);
    twist.initLegBuck(LEG1);
    twist.initLegBuck(LEG2);

    syncCommunication.initSlave(); // start the synchronisation
    data.enableAcquisition(2, 35); // enable the analog measurement
    data.triggerAcquisition(2);     // starts the analog measurement
    canCommunication.setCanNodeAddr(CAN_SLAVE_ADDR);
    canCommunication.setBroadcastPeriod(10);
    canCommunication.setControlPeriod(10);


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

    rs485Communication.configure(buffer_tx, buffer_rx, sizeof(ConsigneStruct_t), slave_reception_function, 10625000, true); // custom configuration for RS485

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
            printk("[%d,%d,%d,%d,%d]:", power_leg_settings[LEG1].settings[0],
                                        power_leg_settings[LEG1].settings[1],
                                        power_leg_settings[LEG1].settings[2],
                                        power_leg_settings[LEG1].settings[3],
                                        power_leg_settings[LEG1].settings[4]);
            printk("%f:", power_leg_settings[LEG1].duty_cycle);
            printk("%f:", power_leg_settings[LEG1].reference_value);
            printk("%s:", power_leg_settings[LEG1].tracking_var_name);
            printk("%f:", tracking_vars[LEG1].address[0]);
            printk("[%d,%d,%d,%d,%d]:", power_leg_settings[LEG2].settings[0],
                                        power_leg_settings[LEG2].settings[1],
                                        power_leg_settings[LEG2].settings[2],
                                        power_leg_settings[LEG2].settings[3],
                                        power_leg_settings[LEG2].settings[4]);
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
            printk("%f:", power_leg_settings[LEG1].duty_cycle);
            printk("%f:", V1_low_value);
            printk("%f:", V1_low_value);
            printk("%f:", I1_low_value);
            printk("%f:", V1_max);
            printk("%f:", power_leg_settings[LEG2].duty_cycle);
            printk("%f:", I2_low_value);
            printk("%f",  V2_low_value);
            printk("%f:", V2_max);
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


    /* Analog communication value */
    data.triggerAcquisition(2);
    analog_value = data.getLatest(2, 35);

    ctrl_slave_counter++; //counter for the slave function

    can_test_ctrl_enable = canCommunication.getCtrlEnable();
    can_test_reference_value = canCommunication.getCtrlReference();


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
