#ifndef _SIMDRV_RCATS_MSGS_H_
#define _SIMDRV_RCATS_MSGS_H_
/*
definitions for RCTN communication.

The intent is to use this file on both sides of the fence to cut down on typos and make changes easier down the line.
*/

const char RCTN_DELIM = '|';     //This character delimits fields in RCTN messages
const char RCTN_TERM = '\n';     //This character marks the end of all RCTN messages

const char* RCTN_CMD_FPGA = "FPGA";

const char* RCTN_CMD_SET = "SET";
const char* RCTN_CMD_GET = "GET";

const char* RCTN_RESULT_SUCCESS = "OK";
const char* RCTN_RESULT_INVALID_CMD = "INVALID_CMD";      //RCTN didn't understand message type
const char* RCTN_RESULT_INVALID_PARAM = "INVALID_PARAM";  //RCTN didn't understand a parameter in the message
const char* RCTN_RESULT_INVALID_VALUE = "INVALID_VALUE";  //RCTN knows the parameter, but the requested value is not valid
const char* RCTN_RESULT_ACTION_FAILED = "ACTION_FAILED";  //RCTN understood the message, but failed when attempting to execute it

#endif // _SIMDRV_RCATS_MSGS_H_

