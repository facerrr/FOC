#ifndef REPORT_H_
#define REPORT_H_
#include "usart.h"
#include "motor.h"
#include "usbd_cdc_if.h"
void upload_control_parameter(void);
void report_state_variable(void);

unsigned int floatToInt32(float data0);
uint32_t float2hex( float HEX );
uint32_t int2hex( int value );
void SynStatePM(void);
void SynControlPM(void);
void logInfo(uint8_t infoID);
#endif
