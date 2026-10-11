import pathlib, subprocess, tempfile, unittest
ROOT=pathlib.Path(__file__).resolve().parents[2]
class Referee(unittest.TestCase):
 def test_receive_contract(self):
  h=(ROOT/'Application/ProtocolLayer/judge_protocol.h').read_text(encoding='utf-8')
  self.assertIn('const uint8_t *rxBuf, uint16_t length',h)
 def test_dma_order(self):
  s=(ROOT/"Application/DriverLayer/drv_uart.c").read_text(encoding="utf-8-sig")
  branch=s[s.index("else if (huart == &huart1)"):s.index("/* ====== UART7",s.index("else if (huart == &huart1)"))]
  self.assertLess(branch.index("__HAL_DMA_GET_COUNTER"),branch.index("HAL_UART_DMAStop"))
  self.assertLess(branch.index("HAL_UART_DMAStop"),branch.index("USART1_rxDataHandler"))
  self.assertLess(branch.index("USART1_rxDataHandler"),branch.index("HAL_UART_Receive_DMA"))
  self.assertNotIn("memset",branch)
 def test_real_c(self):
  with tempfile.TemporaryDirectory() as d:
   p=pathlib.Path(d)
   (p/'stm32h7xx_hal.h').write_text('#include <stdint.h>\n#include <stdbool.h>\n#include <string.h>\n#define __packed\nuint32_t HAL_GetTick(void);\n')
   (p/'rp_config.h').write_text('#define DEV_OFFLINE 0\n#define DEV_ONLINE 1\n')
   (p/'main.c').write_text(HARNESS)
   cmd=['gcc','-std=c99','-fpack-struct=1','-Wall','-Wextra','-Werror','-I'+d]
   cmd += ['-I'+str(ROOT/'Application'/x) for x in ['ProtocolLayer','DeviceLayer','AlgorithmLayer']]
   cmd += [str(p/'main.c')]+[str(ROOT/x) for x in ['Application/ProtocolLayer/judge_protocol.c','Application/DeviceLayer/judge.c','Application/AlgorithmLayer/crc.c']]+['-o',str(p/'test.exe')]
   result=subprocess.run(cmd,capture_output=True)
   self.assertEqual(result.returncode,0,result.stderr.decode(errors="replace"))
   subprocess.run([str(p/'test.exe')],check=True)
HARNESS=r"""
#include <assert.h>
#include "judge.h"
#include "crc.h"
static uint32_t tick;
static uint8_t combined[160];
static uint16_t combined_n;
uint32_t HAL_GetTick(void){return tick;}
static void frame(uint16_t id,const uint8_t *payload,uint16_t n,int split){
 uint8_t b[80]={0xa5}; b[1]=n;b[2]=n>>8;Append_CRC8_Check_Sum(b,5);
 b[5]=id;b[6]=id>>8;memcpy(b+7,payload,n);Append_CRC16_Check_Sum(b,n+9);
 if(split == -1){memcpy(combined+combined_n,b,n+9);combined_n+=n+9;return;}
 if(split == -2){b[n+8]^=1;}
 if(split == -3){b[4]^=1;}
 if(split>0){judge_recive(b,split);judge_recive(b+split,n+9-split);}else judge_recive(b,n+9);
}
int main(void){
 uint8_t oversized[5]={0xa5,0xff,0xff,0,0};
 uint8_t limit[13]={1}, modern[14]={0},legacy[16]={0},noise[]={0,0xa5,0xff,0xff,0,0,9};
 My_Judge_Init();Append_CRC8_Check_Sum(oversized,5);judge_recive(oversized,5);judge_recive(NULL,100);judge_recive(oversized,0);limit[10]=80;tick=100;judge_recive(noise,sizeof noise);
 frame(0x0201,limit,13,3);assert(My_Judge.status->limit_received);assert(My_Judge.status->limit_rx_count==1);assert(My_Judge.status->limit_update_ms==100);
 tick=120;modern[8]=20;modern[10]=33;modern[12]=44;frame(0x0202,modern,14,8);
 assert(My_Judge.org_info->power_heat_data.chassis_power_buffer==20);assert(My_Judge.org_info->power_heat_data.shooter_id1_17mm_cooling_heat==33);
 assert(My_Judge.status->buffer_update_ms==120);assert(My_Judge.status->limit_update_ms==100);
 assert(My_Judge.org_info->power_heat_data.shooter_id1_42mm_cooling_heat==44);
 assert(My_Judge.org_info->power_heat_data.shooter_id2_17mm_cooling_heat==0);
 tick=140;frame(0x0202,legacy,16,0);assert(My_Judge.status->buffer_rx_count==2);assert(My_Judge.org_info->power_heat_data.chassis_power_buffer==0);
 limit[10]=0;tick=150;frame(0x0201,limit,13,0);frame(0x0201,limit,12,0);assert(My_Judge.status->limit_rx_count==1);assert(My_Judge.org_info->game_robot_status.chassis_power_limit==80);
 frame(0x0202,modern,14,-2);frame(0x0202,modern,14,-3);frame(0x0202,modern,13,0);
 assert(My_Judge.status->buffer_rx_count==2);
 limit[10]=90;frame(0x0201,limit,13,-1);frame(0x0202,modern,14,-1);
 tick=200;judge_recive(combined,combined_n);assert(My_Judge.status->limit_rx_count==2);assert(My_Judge.status->buffer_rx_count==3);
 for(int split=1;split<23;split++)frame(0x0202,modern,14,split);
 assert(My_Judge.status->buffer_rx_count==25);
 for(int i=0;i<70000;i++)My_Judge_Realtime_Task(&My_Judge);
 assert(My_Judge.status->offline_cnt==My_Judge.status->offline_cnt_max);assert(My_Judge.status->limit_update_ms==200);assert(My_Judge.status->buffer_update_ms==200);
 return 0;
}
"""
if __name__=='__main__': unittest.main()
