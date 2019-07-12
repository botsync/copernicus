bool LOWBATTERY_FLAG
bool CHARGING_FLAG
bool MOTOR_ALARM_FLAG
bool BMS_ALARM_FLAG
bool HIGH_TEMP_FLAG
bool COMM_ERROR_FLAG

uint16 BMS_SOC
uint16 BMS_SOH
uint16 BMS_Voltage
int16 BMS_Current
uint16 BMS_voltageDiff
uint16 BMS_tempDiff
uint16 BMS_BCUMode
uint16 BMS_Alarm

uint16[2] M_State
uint16[2] M_Alarm
uint16[2] M_Power
uint16[2] M_Voltage
uint16[2] M_Current
uint8[2] M_Comm

float32 Board_Temp
float32 PowerConn_Temp
float32 PreCharge_Temp
float32 MotorConn_Temp


