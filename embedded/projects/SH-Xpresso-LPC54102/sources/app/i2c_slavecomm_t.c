/* Open Sensor Platform Project
 * https://github.com/sensorplatforms/open-sensor-platform
 *
 * Copyright (C) 2015 Audience Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*-------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------*/
#include "common.h"
#ifdef ANDROID_COMM_TASK
#include "hostinterface.h"
#include "osp-sensors.h"
#include <string.h>
#include "SensorPackets.h"
#include "BlockMemory.h"
#include "Queue.h"
#include "Driver_OUT.h"

#define HIF_PACKET_SIZE    M_CalcBufferSize(sizeof(HostIFPackets_t))


void Hostif_Init(void);
void Hostif_StartTx(uint8_t *pBuf, uint16_t size, int magic);
void CHostif_StartTxChained(uint8_t *pBuf, uint16_t size, uint8_t *pBuf_next, uint16_t size_next, int magic);

/*-------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------*/
#define SH_WHO_AM_I                 0x54
#define SH_VERSION0                 0x01
#define SH_VERSION1                 0x23

/* Q Size Common Definitions */
#define QUEUE_LOW_THR                               (0)
#define QUEUE_HIGH_THR                              (1)

/* Sensor Data Queue Size Definition */
/* HIF Queue size indicate number of packet a single Q can hold */
#define HIF_SENSOR_DATA_QUEUE_SIZE                  (256)

/* HIF Sensor Data Packet pool size */
#define HIF_SENSOR_DATA_PACKET_POOL_SIZE            HIF_SENSOR_DATA_QUEUE_SIZE

/*-------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------*/
static SH_RegArea_t SlaveRegMap;
static uint8_t QueueOverFlow = 0;
Queue_t *_HiFNonWakeupQueue = NULL;
static void (*PacketRcv)(void *pkt, int len);

/* Memory Pool for Sensor Data Packet for NonWakeup sensor and Wakeup sensor */
DECLARE_BLOCK_POOL( SensorDataPacketPool, HIF_PACKET_SIZE, HIF_SENSOR_DATA_PACKET_POOL_SIZE );

/*-------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      QHighThresholdCallBack
 *          This Callback called when queue reach to high threshold mark
 *
 * @param  [IN] QType - Type of Queue
 *
 * @return  none
 *
 ***************************************************************************************************/
static void QHighThresholdCallBack( Q_Type_t QType )
{
    switch ( QType )
    {
    case QUEUE_NONWAKEUP_TYPE:
        /* Send indication to host */
        SensorHubAssertInt();
        break;

    default:
        break;
    }
}


/****************************************************************************************************
 * @fn      QEmptyCallBack
 *          Callback for queue when it gets empty
 *
 * @param  [IN] QType - Type of Queue
 *
 * @return  void
 *
 ***************************************************************************************************/
static void QEmptyCallBack( Q_Type_t QType )
{

    /* Mark Q empty */
    switch ( QType )
    {
    case QUEUE_NONWAKEUP_TYPE:
        /* Q is empty so clear Host interrupt pin */
        SensorHubDeAssertInt();
        break;

    default:
        break;
    }
}


/****************************************************************************************************
 * @fn      QInitialize
 *          Initialize Non-Wakeup, Wakeup and Control Response Q
 *
 * @param   none
 *
 * @return  OSP_STATUS_OK or error code
 *
 ***************************************************************************************************/
static int16_t QInitialize( void )
{
    int16_t errCode;

    /* Initialize Data Packet pool */
    //TODO - Opportunity for future optimization here by having different PACKET
    // size for this pool since not all sensors will be wakeup type and most likely
    // the wakeup sensors data will be smaller in size as compared to e.g. Rotation Vector
    InitBlockPool( SensorDataPacketPool, sizeof(SensorDataPacketPool), HIF_PACKET_SIZE );

    /* Create Non wakeup Queue */
    _HiFNonWakeupQueue = QueueCreate( HIF_SENSOR_DATA_QUEUE_SIZE, QUEUE_LOW_THR, QUEUE_HIGH_THR );
    ASF_assert(_HiFNonWakeupQueue != NULL);

    /* Register Call backs for High and Low Thresholds   */
    errCode = QueueRegisterCallBack( _HiFNonWakeupQueue , QUEUE_EMPTY_CB, (fpQueueEvtCallback_t) QEmptyCallBack, (void *)QUEUE_NONWAKEUP_TYPE );
    errCode = QueueRegisterCallBack( _HiFNonWakeupQueue , QUEUE_HIGH_THRESHOLD_CB, (fpQueueEvtCallback_t) QHighThresholdCallBack, (void *)QUEUE_NONWAKEUP_TYPE );

    return errCode;
}


/************************************************************************
 * @fn      DeQueueToBuffer
 *          Dequeues HIF packets from the given queue and copies to the
 *          buffer provided. pBufSz contains the buffer size on entry and
 *          returns the buffer size consumed.
 *
 ************************************************************************/
int16_t DeQueueToBuffer( uint8_t *pBuf, uint32_t *pBufSz, Queue_t *pQ )
{
    int16_t status;
    Buffer_t *pHIFPkt;
    uint32_t bufLen = *pBufSz;

    *pBufSz = 0; //Reset outgoing size consumed

    do {
        status = DeQueue( pQ, &pHIFPkt );
        if (status == OSP_STATUS_OK) {
            if (bufLen >= pHIFPkt->Header.Length) {
                memcpy( (pBuf + *pBufSz), &pHIFPkt->DataStart, pHIFPkt->Header.Length );

                /* Update length */
                *pBufSz += pHIFPkt->Header.Length;
                bufLen -= pHIFPkt->Header.Length;
            }

            /* Free packet memory */
            status = FreeBlock( SensorDataPacketPool, pHIFPkt );
            ASF_assert(status == OSP_STATUS_OK);
        }
    } while((status != OSP_STATUS_QUEUE_EMPTY) && (bufLen >= pHIFPkt->Header.Length));

    return OSP_STATUS_OK;
}


/************************************************************************
 * @fn      SH_Slave_init
 *          Initialize the Sensor Hub I2C Slave register interface
 *
 ************************************************************************/
static void SH_Slave_init(void)
{
    memset(&SlaveRegMap, 0, sizeof(SlaveRegMap));
    SlaveRegMap.version0 = SH_VERSION0;
    SlaveRegMap.version1 = SH_VERSION1;
    SlaveRegMap.whoami   = SH_WHO_AM_I;

    SlaveRegMap.irq_cause = 0;
    SlaveRegMap.read_len  = 0;
    SlaveRegMap.rd_mem[0] = 0;
}

extern volatile int i2cdoneflag;

/*------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*------------------------------------------------------------------------*/
uint8_t cmdlog[512];
int    cmdidx = 0;
uint8_t process_command(uint8_t *rx_buf, uint16_t length)
{
    uint8_t remain  = 0;
    SensorPacketTypes_t Out;
    uint32_t        pack_sz;

    if (length < 1) return 0;
    cmdlog[cmdidx] = rx_buf[0];
    cmdidx++;
    cmdidx %= 512;

    switch (rx_buf[0]) {

    case OSP_DATA_LEN_L:
        CHostif_StartTxChained((uint8_t *)&(SlaveRegMap.read_len2), sizeof(SlaveRegMap.read_len2),
                    (uint8_t *)SlaveRegMap.rd_mem, SlaveRegMap.read_len2, __LINE__);

        break;
    case OSP_DATA_LEN_H:
        CHostif_StartTxChained((uint8_t *)&(SlaveRegMap.read_len2)+1, sizeof(SlaveRegMap.read_len2)-1,
                    (uint8_t *)SlaveRegMap.rd_mem, SlaveRegMap.read_len2, __LINE__);
        break;

    case OSP_INT_LEN:
        if (QueueOverFlow) {
            SlaveRegMap.intlen = OSP_INT_OVER;
        } else {
            SlaveRegMap.intlen = OSP_INT_NONE;
        }
        pack_sz = sizeof(SlaveRegMap.rd_mem);

        /* Note: This logic does not handle situation where a buffer in not fully emptied in previous
           read attempt */
        DeQueueToBuffer( SlaveRegMap.rd_mem, &pack_sz, _HiFNonWakeupQueue );
        if (pack_sz > 0) {
            SlaveRegMap.intlen |= OSP_INT_DRDY;
            SlaveRegMap.intlen |= (pack_sz << 4); /* pack_sz is reset in DeQueueToBuffer() call */
        }

        SlaveRegMap.read_len2 = pack_sz;

        if (pack_sz == 0) {
            Hostif_StartTx((uint8_t *)&(SlaveRegMap.intlen), sizeof(SlaveRegMap.intlen), __LINE__);
        } else {

            CHostif_StartTxChained((uint8_t *)&(SlaveRegMap.intlen), sizeof(SlaveRegMap.intlen),
                (uint8_t *)SlaveRegMap.rd_mem, pack_sz, __LINE__);
        }
        break;

    case OSP_INT_REASON:
        if (QueueOverFlow) {
            SlaveRegMap.irq_cause = OSP_INT_OVER;
        } else {
            SlaveRegMap.irq_cause = OSP_INT_NONE;
        }
        pack_sz = sizeof(SlaveRegMap.rd_mem);

        D0_printf("OSP_INT_REASON\r\n");
        /* Note: This logic does not handle situation where a buffer in not fully emptied in previous
           read attempt */
        DeQueueToBuffer( SlaveRegMap.rd_mem, &pack_sz, _HiFNonWakeupQueue );
        if (pack_sz > 0) {
            SlaveRegMap.irq_cause |= OSP_INT_DRDY;
        }
        SlaveRegMap.read_len = pack_sz; /* pack_sz is reset in DeQueueToBuffer() call */
        SlaveRegMap.read_len2 = pack_sz;
        Hostif_StartTx((uint8_t *)&(SlaveRegMap.irq_cause), sizeof(SlaveRegMap.irq_cause), __LINE__);

        break;

    case OSP_WHOAMI:        /* Who am */
        Hostif_StartTx((uint8_t *)(&SlaveRegMap.whoami), sizeof(SlaveRegMap.whoami), __LINE__);
        break;

    case OSP_VERSION0:
        Hostif_StartTx((uint8_t *)(&SlaveRegMap.version0), sizeof(SlaveRegMap.version0), __LINE__);
        break;

    case OSP_VERSION1:
        Hostif_StartTx((uint8_t *)(&SlaveRegMap.version1), sizeof(SlaveRegMap.version1), __LINE__);
        break;

    case OSP_CONFIG:        /* Reset, not implemented yet */
        /* Need to flush queue somewhere */
        QueueOverFlow = 0;
        break;

    case OSP_DATA_OUT:        /* Read data */
        //D0_printf("OSP_DATA_OUT\r\n");
        if (SlaveRegMap.read_len2 == 0)
            break;
        if (SlaveRegMap.read_len2 < 3 && SlaveRegMap.read_len2 > 0) {
            //__ASM volatile("BKPT #01");
        }
        break;

    case OSP_DATA_IN:        /* Write data */
        /* Host has written a packet */
        /* ParseHostInterfacePkt(&Out, &rx_buf[1], length-1); */
        PacketRcv(&rx_buf[1], length - 1);
        break;

    default:
        if (rx_buf[0] >= 0x20 && rx_buf[0] < 0x50) {
            /* ENABLE */
            SensorEnable((ASensorType_t)(rx_buf[0]-0x20));
            D0_printf("Enable %i\r\n", rx_buf[0] - 0x20);
        } else if (rx_buf[0] >= 0x50 && rx_buf[0] < 0x80) {
            /* DISABLE */
            SensorDisable((ASensorType_t)(rx_buf[0]-0x50));
            D0_printf("Disable %i\r\n", rx_buf[0] - 0x50);
        }
        break;
    }
    return remain;
}

void *OSPOut_driver_GetBuffer(void)
{
    return AllocBlock(SensorDataPacketPool);
}

void OSPOut_driver_PutBuffer(void *buf, int len)
{
    EnQueue(_HiFNonWakeupQueue, buf);
}

void OSPOut_driver_Initialize((void )(*ReceiveCB)(void *buf, int len))
{
    int16_t err;
    /* Initialize packet queue */
    err = QInitialize();
    ASF_assert(err == OSP_STATUS_OK);
    Hostif_Init();

    /* Active high int, set to in active */
    SensorHubDeAssertInt();

    /* Init register area for slave */
    SH_Slave_init();

    PacketRcv = ReceiveCB;
}

/**********************************************************************
 * @fn      I2CCommTask
 *          This tasks primary goal is to serialize the communication
 *        request (sensor results) going over I2C
 *
 * @param   none
 *
 * @return  none
 *
 **********************************************************************/
ASF_TASK void I2CCommTask(ASF_TASK_ARG)
{
    MessageBuffer *rcvMsg = NULLP;

    D0_printf("%s-> I2C Slave ready\r\n", __FUNCTION__);

    while(1) {
        ASFReceiveMessage(I2CSLAVE_COMM_TASK_ID, &rcvMsg );

        switch (rcvMsg->msgId) {
        default:
            D1_printf("I2C:!!!UNHANDLED MESSAGE:%d!!!\r\n", rcvMsg->msgId);
            break;
        }
    }
}

struct _Driver_OUT Driver_OUT = {
	.initialize = OSPOut_driver_Initialize,
	.PutBuffer = OSPOut_driver_PutBuffer,
	.GetBuffer = OSPOut_driver_GetBuffer,
};
#endif //ANDROID_COMM_TASK

/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
