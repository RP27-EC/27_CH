#include "judge_protocol.h"
#include "judge.h"
#include "crc.h"
#include <string.h>

/* One bounded frame retained across DMA idle chunks. Invalid candidates slide
 * by one byte, allowing a valid frame following noise or a bad CRC to recover. */
#define JUDGE_FRAME_CAPACITY 512u
static uint8_t pending[JUDGE_FRAME_CAPACITY];
static uint16_t pending_length;
judge_frame_header_t judge_frame_header;
drv_judge_info_t drv_judge_info = {.frame_header = &judge_frame_header};
void judge_recive(const uint8_t *rxBuf, uint16_t length)
{
    uint16_t i;
    if (rxBuf == NULL) return;
    for (i = 0; i < length; ++i) {
        pending[pending_length++] = rxBuf[i];
        while (pending_length != 0) {
            uint16_t payload_length, frame_length;
            if (pending[0] != 0xa5) goto discard;
            if (pending_length < 5) break;
            if (!Verify_CRC8_Check_Sum(pending, 5)) goto discard;
            payload_length = (uint16_t)(pending[1] | ((uint16_t)pending[2] << 8));
            if (payload_length > JUDGE_FRAME_CAPACITY - 9u) goto discard;
            frame_length = payload_length + 9u;
            if (pending_length < frame_length) break;
            if (!Verify_CRC16_Check_Sum(pending, frame_length)) goto discard;
            judge_frame_header.SOF = pending[0];
            judge_frame_header.data_length = payload_length;
            judge_frame_header.seq = pending[3];
            judge_frame_header.CRC8 = pending[4];
            drv_judge_info.cmd_id = (uint16_t)(pending[5] | ((uint16_t)pending[6] << 8));
            drv_judge_info.frame_tail = (uint16_t)(pending[frame_length-2] | ((uint16_t)pending[frame_length-1] << 8));
            judge_update(drv_judge_info.cmd_id, pending + 7, payload_length);
            pending_length -= frame_length;
            memmove(pending, pending + frame_length, pending_length);
            continue;
        discard:
            --pending_length;
            memmove(pending, pending + 1, pending_length);
        }
    }
}
void USART1_rxDataHandler(const uint8_t *rxBuf, uint16_t length)
{
    judge_recive(rxBuf, length);
}
