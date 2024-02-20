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
 * @brief  This file is a communication library for the power test bench of the twist board converter
 *
 * @author Luiz Villa <luiz.villa@laas.fr>
 */


#ifndef TEST_BENCH_COMM_PROTOCOL_H
#define TEST_BENCH_COMM_PROTOCOL_H

//-------------OWNTECH DRIVERS-------------------
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "DataAPI.h"
#include "Rs485Communication.h"
#include "SyncCommunication.h"


#include "zephyr/console/console.h"
#include "zephyr/zephyr.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>


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

extern uint8_t received_serial_char;
extern uint8_t received_char;
extern char bufferstr[255];

typedef enum
{
    IDLE, POWER_ON, POWER_OFF
} tester_states_t;

extern tester_states_t mode;

/**
 * @brief Structure representing tracking variables and their information.
 *
 * This structure holds the name, memory address, and channel reference of tracking variables.
 */
typedef struct {
    const char *name;           /**< Name of the tracking variable */
    float32_t *address;         /**< Memory address of the tracking variable */
    channel_t channel_reference; /**< Channel reference of the tracking variable */
} TrackingVariables;
/**
 * @brief Structure representing the settings of a power leg.
 *
 * This structure holds various settings related to a power leg, including boolean settings, switches,
 * tracking variable information, reference value, and duty cycle.
 */
typedef struct {
    bool settings[5];               /**< Array of boolean settings */
    pin_t switches[2];              /**< Array of switches */
    float32_t *tracking_variable;   /**< Pointer to the tracking variable */
    const char *tracking_var_name;  /**< Name of the tracking variable */
    float32_t reference_value;      /**< Reference value */
    float32_t duty_cycle;           /**< Duty cycle value */
} PowerLegSettings;

/**
 * @brief Structure representing a command mapping to settings.
 *
 * This structure maps a command string to a corresponding function pointer that handles the settings for a power leg.
 */
typedef struct {
    char cmd[16];                               /**< Command string */
    void (*func)(uint8_t power_leg, uint8_t setting_position); /**< Function pointer to handle the settings */
} cmdToSettings_t;


/**
 * @brief Structure representing a command mapping to state.
 *
 * This structure maps a command string to a tester state.
 */
typedef struct {
    char cmd[16];             /**< Command string */
    tester_states_t mode;    /**< Tester state */
} cmdToState_t;


/**
 * @brief Structure representing various measurements and statuses.
 *
 * This structure holds variables for testing RS485, Sync, analog measurements, and status information.
 */
typedef struct {
    uint8_t test_RS485;             /**< Variable for testing RS485 */
    uint8_t test_Sync;              /**< Variable for testing Sync */
    uint16_t analog_value_measure;  /**< Analog measurement */
    uint8_t id_and_status;          /**< Status information */
} ConsigneStruct_t;

extern TrackingVariables tracking_vars[6];
extern PowerLegSettings power_leg_settings[2];
extern cmdToSettings_t power_settings[7];
extern cmdToState_t default_commands[3];

extern tester_states_t mode;
extern uint8_t num_tracking_vars;
extern uint8_t num_power_settings;
extern uint8_t num_default_commands;

extern ConsigneStruct_t tx_consigne;
extern ConsigneStruct_t rx_consigne;

extern uint8_t* buffer_tx;
extern uint8_t* buffer_rx;


extern uint8_t status;
extern uint32_t counter_time;

/* RS485 test parameters*/
extern uint8_t rs485_send;
extern uint8_t rs485_receive;

/* Sync test parameters*/
extern uint8_t sync_master_counter;

/* BOOL value for testing */
extern bool test_start     ; // start the test after a certain period of time
extern bool RS485_success  ;
extern bool Sync_success   ;
extern bool Analog_success ;
extern bool Can_success    ;


/* analog test parameters*/
extern uint16_t analog_value;
extern uint16_t analog_value_ref;

/* Sync test parameters*/
extern uint8_t ctrl_slave_counter;

extern bool print_done;

extern float32_t reference_value;

//----------------SERIAL PROTOCOL HANDLER FUNCTIONS---------------------

/**
 * @brief Reads a line from the console input.
 *
 * This function reads characters from the console input until a newline character ('\n') is encountered.
 * It stores the characters in the global bufferstr variable.
 */
void console_read_line();

/**
 * @brief Handles default commands.
 *
 * This function handles default commands by matching the received command with predefined default commands and executing corresponding actions.
 *
 */
void defaultHandler();

/**
 * @brief Handles power leg settings commands.
 *
 * This function determines the power leg based on the received message and delegates the command handling to specific setting handlers.
 * The command format is expected to be "_LEGX_<setting>_XXXXX", where X represents the specific setting value.
 *
 */
void powerLegSettingsHandler();

/**
 * @brief Handles boolean settings for a power leg.
 *
 * This function extracts boolean settings from the received command and updates the corresponding power leg settings.
 * The command format is expected to be "_LEGX_l_on" or "_LEGX_l_off", where "on" turns the setting on and "off" turns it off.
 *
 * @param power_leg The index of the power leg.
 * @param setting_position The position of the boolean setting in the power leg settings array.
 */
void boolSettingsHandler(uint8_t power_leg, uint8_t setting_position);

/**
 * @brief Handles duty cycle settings for a power leg.
 *
 * This function extracts the duty cycle value from the received command and updates the corresponding power leg settings.
 * The duty cycle value is expected to be in the format "_LEGX_d_XXXXX", where XXXXX represents the duty cycle value.
 *
 * @param power_leg The index of the power leg.
 * @param setting_position The position of the duty cycle setting in the power leg settings array.
 */
void dutyHandler(uint8_t power_leg, uint8_t setting_position);

/**
 * @brief Handles reference value settings for a power leg.
 *
 * This function extracts the reference value from the received command and updates the corresponding power leg settings.
 * The reference value is expected to be in the format "_LEGX_r_XXXXX", where XXXXX represents the reference value.
 *
 * @param power_leg The index of the power leg.
 * @param setting_position The position of the reference value setting in the power leg settings array.
 */
void referenceHandler(uint8_t power_leg, uint8_t setting_position);

/**
 * @brief Handles calibration settings.
 *
 * This function extracts the variable name, gain, and offset from the received command and updates the calibration parameters.
 * The command format is expected to be "_VX_g_XX.XXXXX_o_XX.XXXXX", where VX represents the variable name, XX.XXXXX represents the gain,
 * and XX.XXXXX represents the offset.
 */
void calibrationHandler();

/**
 * @brief Handles slave reception.
 *
 * This function receives data from the slave device, processes it, and prepares a response.
 * It updates various variables and starts transmission over the RS485 communication interface.
 */
void slave_reception_function(void);

/**
 * @brief Handles master reception.
 *
 * This function receives data from the master device, processes it, and updates relevant variables.
 * It checks for various conditions to determine the success of data reception and synchronization.
 */
void master_reception_function(void);

#endif  //TEST_BENCH_COMM_PROTOCOL_H
