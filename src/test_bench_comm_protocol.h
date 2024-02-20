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

// Define a struct to hold the tracking variables and their names
typedef struct {
    const char *name;
    float32_t *address;
    channel_t channel_reference;
} TrackingVariables;

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

typedef struct {
    char cmd[16];
    void (*func)(uint8_t power_leg, uint8_t setting_position);
} cmdToSettings_t;


typedef struct {
    char cmd[16];
    tester_states_t mode;
} cmdToState_t;


typedef struct {
    uint8_t buf_vab[3];    // Contains Voltage DATA A and Voltage DATA B
    uint8_t test_RS485;    // variable for testing RS485
    uint8_t test_Sync;    // variable for testing Sync
    uint16_t analog_value_measure; // Contains analog measure
    uint8_t id_and_status; // Contains status
} ConsigneStruct_t ;


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

/* analog test parameters*/
extern uint16_t analog_value;
extern uint16_t analog_value_ref;

/* Sync test parameters*/
extern uint8_t ctrl_slave_counter;

extern bool print_done;

extern float32_t reference_value;

//----------------SERIAL PROTOCOL HANDLER FUNCTIONS---------------------
void console_read_line();

void defaultHandler();

void powerLegSettingsHandler();

void boolSettingsHandler(uint8_t power_leg, uint8_t setting_position);

void dutyHandler(uint8_t power_leg, uint8_t setting_position);

void referenceHandler(uint8_t power_leg, uint8_t setting_position);

void calibrationHandler();

void reception_function();


#endif  //TEST_BENCH_COMM_PROTOCOL_H
