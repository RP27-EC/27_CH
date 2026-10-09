"""Exercise the real state sender and CAN driver with three bxCAN mailboxes."""
import unittest

from test_ekf_init import ROOT, run_c


def harness():
    driver = (ROOT / 'Application/DriverLayer/drv_can.c').read_text(encoding='utf-8-sig')
    start = driver.index('HAL_StatusTypeDef CAN_SendData(')
    driver = driver[start:driver.index('/* rxData Handler', start)]
    header = (ROOT / 'Application/ProtocolLayer/board_comm.h').read_text(encoding='utf-8-sig')
    start = header.index('typedef struct {', header.index('} Board_Rx_Info_t;'))
    tx_type = header[start:header.index('} Board_Tx_Info_t;', start) + len('} Board_Tx_Info_t;')]
    sender = (ROOT / 'Application/ProtocolLayer/board_comm.c').read_text(encoding='utf-8-sig')
    start = sender.index('HAL_StatusTypeDef Board_Comm_Send_State(')
    sender = sender[start:sender.index('\n}', start) + 2]
    return r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#define CAN1 ((void*)1)
#define CAN2 ((void*)2)
#define CAN_ID_STD 0
#define CAN_RTR_DATA 0
#define MASTER_RX_STATE_CAN_ID 0x23
#define PITCH_MECH_VALID_MARKER 0xA5
typedef enum {HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct {void *Instance;} CAN_HandleTypeDef;
typedef struct {uint32_t StdId, IDE, RTR, DLC;} CAN_TxHeaderTypeDef;
CAN_HandleTypeDef hcan2={CAN2};
static uint32_t tick, release_at, ipsr, primask, basepri;
static unsigned used, probes, queued, state_frames;
static bool stalled, fail_add;
static uint8_t state_payload[8];
uint32_t __get_IPSR(void){return ipsr;}
uint32_t __get_PRIMASK(void){return primask;}
uint32_t __get_BASEPRI(void){return basepri;}
uint32_t HAL_GetTick(void){return tick++;}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h){
    assert(h==&hcan2);
    assert(++probes<100);
    if(!stalled && used==3 && (int32_t)(tick-release_at)>=0)used--;
    return 3-used;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h,CAN_TxHeaderTypeDef *hdr,uint8_t *data,uint32_t *mb){
    assert(h==&hcan2);
    if(fail_add || used==3)return HAL_ERROR;
    *mb=used++;queued++;
    if(hdr->StdId==0x23){state_frames++;memcpy(state_payload,data,8);}
    return HAL_OK;
}
''' + tx_type + '\nBoard_Tx_Info_t board_tx_info;\n' + driver + '\n' + sender


class StateMailboxPressureTest(unittest.TestCase):
    def test_fourth_frame_waits_for_mailbox_and_keeps_payload(self):
        run_c(harness() + r'''
int main(void){
    uint8_t data[8]={0};float angle;
    release_at=1;
    assert(CAN_SendData(&hcan2,0x21,data)==HAL_OK);
    assert(CAN_SendData(&hcan2,0x22,data)==HAL_OK);
    assert(CAN_SendData(&hcan2,0x01,data)==HAL_OK);
    assert(Board_Comm_Send_State(.42f,true)==HAL_OK);
    assert(queued==4 && state_frames==1);
    assert(board_tx_info.state_tx_fail_count==0);
    memcpy(&angle,state_payload+1,4);
    assert(fabsf(angle-.42f)<1e-6f && state_payload[5]==0xA5);
}
''')

    def test_stalled_bus_returns_failure_with_bounded_wait(self):
        run_c(harness() + r'''
int main(void){
    used=3;stalled=true;
    assert(Board_Comm_Send_State(.2f,true)!=HAL_OK);
    assert(tick<=4 && probes<10 && queued==0);
    assert(board_tx_info.state_tx_fail_count==1);
}
''')

    def test_wait_deadline_survives_tick_wrap(self):
        run_c(harness() + r'''
int main(void){
    used=3;tick=UINT32_MAX;release_at=0;
    assert(Board_Comm_Send_State(.2f,true)==HAL_OK);
    assert(state_frames==1 && board_tx_info.state_tx_fail_count==0);
}
''')

    def test_interrupt_and_masked_contexts_never_wait(self):
        run_c(harness() + r'''
int main(void){
    used=3;stalled=true;
    ipsr=1;assert(Board_Comm_Send_State(.2f,true)!=HAL_OK);ipsr=0;
    primask=1;assert(Board_Comm_Send_State(.2f,true)!=HAL_OK);primask=0;
    basepri=1;assert(Board_Comm_Send_State(.2f,true)!=HAL_OK);basepri=0;
    assert(tick==0 && queued==0 && board_tx_info.state_tx_fail_count==3);
}
''')

    def test_other_hal_failure_is_reported_without_retry_loop(self):
        run_c(harness() + r'''
int main(void){
    fail_add=true;
    assert(Board_Comm_Send_State(.2f,true)!=HAL_OK);
    assert(queued==0 && tick<=1 && board_tx_info.state_tx_fail_count==1);
}
''')


if __name__ == '__main__':
    unittest.main()
