#ifndef __DATA2PACK_H__
#define __DATA2PACK_H__
#include "osp-api.h"

static void OSPOut_Bool(ASensorType_t sensorType,
			Android_BooleanResultData_t  *pData);
static void OSPOut_triaxis(ASensorType_t sensorType,
			Android_TriAxisPreciseData_t *pData);

#endif
