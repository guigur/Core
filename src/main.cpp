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
#include "test_bench_comm_protocol.h"

//----------- USER INCLUDE ----------------------

//-------------LOOP FUNCTIONS DECLARATION----------------------
void loop_communication_task(); // code to be executed in the slow communication task
int8_t CommTask_num;            // Communication Task number
void loop_application_task();   // code to be executed in the fast application task
int8_t AppTask_num;             // Application Task number
void loop_control_task();       // code to be executed in real-time at 20kHz

//--------------USER VARIABLES DECLARATIONS----------------------

static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable_leg_1 = false;            //[bool] state of the PWM (ctrl task)
static bool pwm_enable_leg_2 = false;            //[bool] state of the PWM (ctrl task)

// uint8_t received_serial_char;

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


//---------------------------------------------------------------

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

    canCommunication.setCanNodeAddr(CAN_MASTER_ADDR);
    canCommunication.setBroadcastPeriod(10);
    canCommunication.setControlPeriod(10);

    spin.dac.initConstValue(2);

    task.createCritical(&loop_control_task, control_task_period);
    task.startCritical();

    CommTask_num = task.createBackground(loop_communication_task);
    AppTask_num =  task.createBackground(loop_application_task);

    task.startBackground(CommTask_num);
    task.startBackground(AppTask_num);

    rs485Communication.configure(buffer_tx, buffer_rx, sizeof(ConsigneStruct_t), master_reception_function, 10625000, true); // custom configuration for RS485
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
            printk("{%u,%u,%u,%u}:", RS485_success,
                                     Sync_success,
                                     Analog_success,
                                     Can_success);
            printk("[%d,%d,%d,%d,%d]:", power_leg_settings[LEG1].settings[0],
                                        power_leg_settings[LEG1].settings[1],
                                        power_leg_settings[LEG1].settings[2],
                                        power_leg_settings[LEG1].settings[3],
                                        power_leg_settings[LEG1].settings[4]);
            printk("%f:", power_leg_settings[LEG1].duty_cycle);
            printk("%f:", power_leg_settings[LEG1].reference_value);
            printk("%s:", power_leg_settings[LEG1].tracking_var_name);
            printk("%f:", tracking_vars[LEG1].address[0]);
            printk("\n");
            break;
        case POWER_ON:
            spin.led.turnOn();
            if(!print_done) {
                printk("POWER ON \n");
                print_done = true;
            }
            printk("%f:", power_leg_settings[LEG1].duty_cycle);
            printk("%f:", power_leg_settings[LEG2].duty_cycle);
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
    spin.dac.setConstValue(2, 1, analog_value_ref);
    canCommunication.setCtrlEnable(true);
    canCommunication.setCtrlReference(CAN_Bus_receive_ref);


    switch(mode){
        case IDLE:
            twist.stopLeg(LEG1);
            twist.stopLeg(LEG2);
            pwm_enable_leg_1 = false;
            pwm_enable_leg_2 = false;

            break;

        case POWER_OFF:

            twist.stopLeg(LEG1);
            twist.stopLeg(LEG2);
            pwm_enable_leg_1 = false;
            pwm_enable_leg_2 = false;

            /* Voltage reference */
            rs485_send++;

            /* writting rs485 value */
            tx_consigne.test_RS485 = rs485_send;

            rs485Communication.startTransmission();

            counter_time++;
            if (counter_time > 50) test_start =  true;

            break;

        case POWER_ON:


            if(!pwm_enable_leg_1 && power_leg_settings[LEG1].settings[BOOL_LEG]) {twist.startLeg(LEG1); pwm_enable_leg_1 = true;}
            if(!pwm_enable_leg_2 && power_leg_settings[LEG2].settings[BOOL_LEG]) {twist.startLeg(LEG2); pwm_enable_leg_2 = true;}


            if(power_leg_settings[LEG1].settings[BOOL_LEG]){
                    twist.setLegDutyCycle(LEG1, power_leg_settings[LEG1].duty_cycle ); //uses the normal convention by default
            }

            if(power_leg_settings[LEG2].settings[BOOL_LEG]){
                    twist.setLegDutyCycle(LEG2, power_leg_settings[LEG2].duty_cycle); //uses the normal convention by default
            }

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
    setup_hardware();
    setup_software();

    return 0;
}