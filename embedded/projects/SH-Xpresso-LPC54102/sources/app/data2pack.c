/* Open Sensor Platform Project
 * https://github.com/sensorplatforms/open-sensor-platform
 *
 * Copyright (C) 2015 Audience Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *	  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*-------------------------------------------------------------------------*\
 |	I N C L U D E   F I L E S
\*-------------------------------------------------------------------------*/
#include "common.h"
#include "hostinterface.h"
#include "osp-sensors.h"
#include <string.h>
#include "SensorPackets.h"
#include "BlockMemory.h"
#include "Queue.h"

/*-------------------------------------------------------------------------*\
 |	E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*\
 |	P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*\
 |	S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*\
 |	F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*\
 |	P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*\
 |	P R I V A T E	 F U N C T I O N S
\*-------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*\
 |	P U B L I C	 F U N C T I O N S
\*------------------------------------------------------------------------*/
/***************************************************************************
 * @fn	  OSPOut_Bool
 *		  Send boolean data to output driver.
 *
 ***************************************************************************/

void OSPOut_Bool(ASensorType_t sensorType,
			Android_BooleanResultData_t  *pData)
{
	static Buffer_t	*pHifPacket = NULL;
	uint8_t		*pPayload;
	int16_t		status;
	StepDetector_t	stepDetectorData;
	SignificantMotion_t motionData;

	// Do not send data if host did not activate this sensor type
	if (GetSensorState(sensorType) == 0)
		return;

	/* Allocate packet buffer, packetize and place in queue */
	if (pHifPacket == NULL)
        	pHifPacket = Driver_OUT.GetBuffer();

	if (pHifPacket == NULL) {
		D0_printf("OOPS! Couldn't alloc packet [%s] for %d\r\n", __FUNCTION__, sensorType);
		return;
	}

	pPayload = M_GetBufferDataStart(pHifPacket);

	/* Process sensor and format into packet */
	switch (sensorType) {
	case SENSOR_STEP_DETECTOR:
		stepDetectorData.TimeStamp.TS64 = pData->TimeStamp;
		stepDetectorData.StepDetected   = pData->data;

		/* Format Packet */
		pHifPacket->Header.Length = FormatStepDetectorPkt(pPayload, &stepDetectorData, sensorType);
		break;

	case SENSOR_SIGNIFICANT_MOTION:
		motionData.TimeStamp.TS64 = pData->TimeStamp;
		motionData.MotionDetected = pData->data;

		/* Format Packet */
		pHifPacket->Header.Length = FormatSignificantMotionPktFixP(pPayload, &motionData, sensorType);
		break;

	default:
		D0_printf("Unhandled sensor [%d] in %s!\r\n", sensorType, __FUNCTION__);
		return;
	}

	/* Enqueue packet in HIF queue */
	if (pHifPacket->Header.Length > 0) {
		Driver_OUT.PutBuffer(pHifPacket, pHifPacket->Header.Length);
		pHifPacket = NULL;
	} else {
		D0_printf("Packetization error [%d] for sensor %d\r\n", pHifPacket->Header.Length, sensorType);
	}
}

/***************************************************************************
 * @fn	  OSPOut_triaxis
 *		  Sends 3-axis sensor to output driver.

 * Enqueues data only.
 *
 ***************************************************************************/
void OSPOut_triaxis(ASensorType_t sensorType,
			Android_TriAxisPreciseData_t *pData)
{
	static Buffer_t		*pHifPacket = NULL;
	uint8_t			*pPayload;
	int16_t status;
	UncalibratedFixP_t	UnCalFixPData;
	CalibratedFixP_t	CalFixPData;
	QuaternionFixP_t	QuatFixPData;
	OrientationFixP_t	fixPData;
	ThreeAxisFixP_t		TfixPData;
	StepCounter_t		stepCountData;

	// Do not send data if host did not activate this sensor type
	if (GetSensorState(sensorType) == 0)
		return;

	/* Allocate packet buffer, packetize and place in queue */
	if (pHifPacket != NULL)
		pHifPacket = (Buffer_t *) Driver_OUT.GetBuffer();

	if (pHifPacket == NULL) {
		D0_printf("OOPS! Couldn't alloc packet [%s] for %d\r\n", __FUNCTION__, sensorType);
		return;
	}

	pPayload = M_GetBufferDataStart(pHifPacket);

	/* Process sensor and format into packet */
	switch (sensorType) {
	case AP_PSENSOR_ACCELEROMETER_UNCALIBRATED:
	case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
	case SENSOR_GYROSCOPE_UNCALIBRATED:
		UnCalFixPData.TimeStamp.TS64 = pData->TimeStamp;
		UnCalFixPData.Axis[0]	= pData->X;
		UnCalFixPData.Axis[1]	= pData->Y;
		UnCalFixPData.Axis[2]	= pData->Z;
		UnCalFixPData.Offset[0]	= 0;
		UnCalFixPData.Offset[1]	= 0;
		UnCalFixPData.Offset[2]	= 0;

		pHifPacket->Header.Length = FormatUncalibratedPktFixP(pPayload,
			&UnCalFixPData, META_DATA_UNUSED, sensorType);
		break;
	case SENSOR_ACCELEROMETER:
	case SENSOR_MAGNETIC_FIELD:
	case SENSOR_GYROSCOPE:
		CalFixPData.TimeStamp.TS64 = pData->TimeStamp;
		CalFixPData.Axis[0]	= pData->X;
		CalFixPData.Axis[1]	= pData->Y;
		CalFixPData.Axis[2]	= pData->Z;

		pHifPacket->Header.Length = FormatCalibratedPktFixP(pPayload,
				&CalFixPData, sensorType);
		break;
	case SENSOR_ROTATION_VECTOR:
	case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
	case SENSOR_GAME_ROTATION_VECTOR:
		QuatFixPData.TimeStamp.TS64 = pData->TimeStamp;
		QuatFixPData.Quat[0]	= pData->W;
		QuatFixPData.Quat[1]	= pData->X;
		QuatFixPData.Quat[2]	= pData->Y;
		QuatFixPData.Quat[3]	= pData->Z;

		pHifPacket->Header.Length = FormatQuaternionPktFixP(pPayload,
			&QuatFixPData, sensorType);
		break;
	case SENSOR_GRAVITY:
	case SENSOR_LINEAR_ACCELERATION:
		TfixPData.TimeStamp.TS64 = pData->TimeStamp;
		TfixPData.Axis[0]	= pData->X;
		TfixPData.Axis[1]	= pData->Y;
		TfixPData.Axis[2]	= pData->Z;

		pHifPacket->Header.Length = FormatThreeAxisPktFixP(pPayload,
				&TfixPData, sensorType);
		break;
	case SENSOR_ORIENTATION:
		fixPData.TimeStamp.TS64 = pData->TimeStamp;
		fixPData.Pitch  = pData->X;
		fixPData.Roll   = pData->Y;
		fixPData.Yaw	= pData->Z;
		pHifPacket->Header.Length = FormatOrientationFixP(pPayload,
			&fixPData, sensorType);
		break;
	case SENSOR_STEP_COUNTER:
		stepCountData.TimeStamp.TS64 = pData->TimeStamp;
		stepCountData.NumStepsTotal  = pData->X;
		/* Format Packet */
		pHifPacket->Header.Length = FormatStepCounterPkt(pPayload, &stepCountData, sensorType);
		break;

	case SENSOR_PRESSURE:
		//TODO - Ignore for now!
	default:
		D0_printf("Unhandled sensor [%d] in %s!\r\n", sensorType, __FUNCTION__);
		return;
	}

	/* Enqueue packet in HIF queue */
	if (pHifPacket->Header.Length > 0) {
		Driver_OUT.PutBuffer(pHifPacket, pHifPacket->Header.Length);
		pHifPacket = NULL;
	} else {
		/* Save buffer for next packet */
		D0_printf("Packetization error [%d] for sensor %d\r\n", pHifPacket->Header.Length, sensorType);
	}
}


/****************************************************************************************************
 * @fn	  SendSensorEnableReq
 *		  Sends message to the communication task to handle enable request received
 *
 * @param   [IN]sensor - Android sensor enum
 *
 * @return  none
 *
 ***************************************************************************************************/
void SendSensorEnableReq(ASensorType_t sensor)
{
	MessageBuffer *pSendMsg = NULLP;
	ASF_assert(ASFCreateMessage(MSG_SENSOR_ENABLE, sizeof(MsgGeneric), &pSendMsg) == ASF_OK);
	pSendMsg->msg.msgSensEnable.U.dword = sensor;
	ASF_assert(ASFSendMessage(I2CSLAVE_COMM_TASK_ID, pSendMsg) == ASF_OK);
}


/*****************************************************************************
 * @fn	  SendSensorDisableReq
 *	  Sends message to the communication task to handle disable
 *        request received
 *
 * @param   [IN]sensor - Android sensor enum
 *
 * @return  none
 *
 *****************************************************************************/
void SendSensorDisableReq(ASensorType_t sensor)
{
	MessageBuffer *pSendMsg = NULLP;
	ASF_assert(ASFCreateMessage(MSG_SENSOR_DISABLE, sizeof(MsgGeneric),
			&pSendMsg) == ASF_OK);
	pSendMsg->msg.msgSensDisable.U.dword = sensor;
	ASF_assert(ASFSendMessage(I2CSLAVE_COMM_TASK_ID, pSendMsg) == ASF_OK);
}

void PacketReceive(void *pack, int len)
{
	SensorPacketTypes_t Out;
	ParseHostInterfacePkt(&Out, &rx_buf[1], length-1);
}

/*-------------------------------------------------------------------------------------------------*\
 |	E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
