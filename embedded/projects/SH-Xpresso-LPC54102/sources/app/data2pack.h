#ifndef __DATA2PACK_H__
#define __DATA2PACK_H__
#include "osp-api.h"

void OSPOut_Bool(ASensorType_t sensorType,
		Android_BooleanResultData_t  *pData);
void OSPOut_triaxis(ASensorType_t sensorType,
		Android_TriAxisPreciseData_t *pData);

int OSPSensor_ctrl_GetSensorState(ASensorType_t sen);
void OSPSensor_ctrl_SensorDisable(ASensorType_t sen);
void OSPSensor_ctrl_SensorEnable(ASensorType_t sen);
void OSPSensor_ctrl_init(void);


#endif
