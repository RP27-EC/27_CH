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
  self.run_c(HARNESS)
 def test_status_17byte_is_accepted(self):
  self.run_c(HARNESS.split('int main(void){')[0]+r'''
int main(void){
 uint8_t status[17]={1};
 My_Judge_Init();status[10]=80;status[16]=7;tick=100;
 frame(0x0201,status,17,3);
 assert(My_Judge.status->limit_received);
 assert(My_Judge.status->limit_rx_count==1);
 assert(My_Judge.org_info->game_robot_status.chassis_power_limit==80);
 return 0;
}
''')
 def test_status_layout_switch_preserves_fields(self):
  self.run_c(HARNESS.split('int main(void){')[0]+r'''
int main(void){
 /* 不同位置放不同值，防止把热量或 float 的字节误读成功率/输出标志。 */
 uint8_t status[17]={101,3,0x34,0x12,0x78,0x56,0x21,0x43,0xe8,3,0x2c,1,0,0,0xc8,0x41,5};
 uint8_t short_status[13]={1};
 My_Judge_Init();tick=100;frame(0x0201,status,17,0);
 assert(sizeof(ext_game_robot_status_t)==17);
 assert(My_Judge.org_info->game_robot_status.robot_id==101);
 assert(My_Judge.org_info->game_robot_status.current_HP==0x1234);
 assert(My_Judge.org_info->game_robot_status.shooter_barrel_heat_limit==1000);
 assert(My_Judge.org_info->game_robot_status.chassis_power_limit==300);
 assert(My_Judge.org_info->game_robot_status.bullet_speed_limit==25.0f);
 assert(My_Judge.org_info->game_robot_status.power_management_gimbal_output==1);
 assert(My_Judge.org_info->game_robot_status.power_management_chassis_output==0);
 assert(My_Judge.org_info->game_robot_status.power_management_shooter_output==1);
 My_Judge_Update(&My_Judge);assert(My_Judge.info->chassis_power_limit==300);
 short_status[10]=80;short_status[12]=2;tick=120;frame(0x0201,short_status,13,4);
 assert(My_Judge.org_info->game_robot_status.chassis_power_limit==80);
 assert(My_Judge.org_info->game_robot_status.bullet_speed_limit==0.0f);
 assert(My_Judge.org_info->game_robot_status.power_management_gimbal_output==0);
 assert(My_Judge.org_info->game_robot_status.power_management_chassis_output==1);
 assert(My_Judge.org_info->game_robot_status.power_management_shooter_output==0);
 assert(My_Judge.status->robot_state_length==13);
 assert(My_Judge.status->robot_state_limit_raw==80);
 assert(My_Judge.status->robot_state_result==JUDGE_ROBOT_STATE_ACCEPTED);
 assert(My_Judge.status->limit_update_ms==120);
 frame(0x0201,status,17,0);assert(My_Judge.org_info->game_robot_status.bullet_speed_limit==25.0f);
 assert(My_Judge.status->robot_state_rx_count==3);assert(My_Judge.status->limit_rx_count==3);
 return 0;
}
''')
 def test_status_rejections_preserve_last_valid_limit(self):
  self.run_c(HARNESS.split('int main(void){')[0]+r'''
int main(void){
 uint8_t status[27]={1},buffer[14]={0};uint16_t bad_lengths[]={0,1,10,11,12,14,16,18,27};
 My_Judge_Init();status[10]=90;tick=100;frame(0x0201,status,17,0);tick=200;
 for(unsigned i=0;i<sizeof(bad_lengths)/sizeof(bad_lengths[0]);i++){
  frame(0x0201,status,bad_lengths[i],0);
  assert(My_Judge.status->robot_state_length==bad_lengths[i]);
  assert(My_Judge.status->robot_state_result==JUDGE_ROBOT_STATE_BAD_LENGTH);
  assert(My_Judge.status->robot_state_limit_raw==0);
 }
 assert(My_Judge.status->robot_state_rx_count==10);
 status[10]=0;frame(0x0201,status,17,0);assert(My_Judge.status->robot_state_result==JUDGE_ROBOT_STATE_ZERO_LIMIT);
 frame(0x0201,status,13,0);assert(My_Judge.status->robot_state_result==JUDGE_ROBOT_STATE_ZERO_LIMIT);
 assert(My_Judge.status->robot_state_rx_count==12);assert(My_Judge.status->robot_state_update_ms==200);
 assert(My_Judge.status->limit_received);assert(My_Judge.status->limit_rx_count==1);
 assert(My_Judge.status->limit_update_ms==100);assert(My_Judge.org_info->game_robot_status.chassis_power_limit==90);
 status[10]=100;tick=300;frame(0x0201,status,17,-2);frame(0x0201,status,17,-3);
 assert(My_Judge.status->robot_state_rx_count==12);assert(My_Judge.status->limit_update_ms==100);
 frame(0x0202,buffer,14,0);assert(My_Judge.status->buffer_received);
 assert(My_Judge.status->robot_state_update_ms==200);
 for(int i=0;i<2000;i++)My_Judge_Realtime_Task(&My_Judge);
 assert(My_Judge.status->robot_state_rx_count==12);
 My_Judge_Init();assert(My_Judge.status->robot_state_rx_count==0);
 assert(My_Judge.status->robot_state_update_ms==0);assert(My_Judge.status->robot_state_length==0);
 assert(My_Judge.status->robot_state_limit_raw==0);assert(My_Judge.status->robot_state_result==JUDGE_ROBOT_STATE_NONE);
 return 0;
}
''')
 def test_status_17byte_dma_splits_and_concatenated_frames(self):
  self.run_c(HARNESS.split('int main(void){')[0]+r'''
int main(void){
 uint8_t status[17]={1},short_status[13]={1},buffer[14]={0},noise[]={0,1,0xa5,0xff,0xff,0,0};
 My_Judge_Init();status[10]=100;short_status[10]=80;tick=100;
 for(int split=1;split<26;split++)frame(0x0201,status,17,split);
 assert(My_Judge.status->limit_rx_count==25);assert(My_Judge.status->robot_state_rx_count==25);
 frame(0x0201,status,17,-1);
 for(uint16_t i=0;i<combined_n;i++)judge_recive(combined+i,1);
 assert(My_Judge.status->limit_rx_count==26);
 combined_n=0;frame(0x0201,short_status,13,-1);frame(0x0202,buffer,14,-1);frame(0x0201,status,17,-1);
 tick=200;judge_recive(noise,sizeof noise);judge_recive(combined,combined_n);
 assert(My_Judge.status->limit_rx_count==28);assert(My_Judge.status->buffer_rx_count==1);
 assert(My_Judge.org_info->game_robot_status.chassis_power_limit==100);
 assert(My_Judge.status->robot_state_length==17);assert(My_Judge.status->limit_update_ms==200);
 return 0;
}
''')
 def run_c(self,harness):
  with tempfile.TemporaryDirectory() as d:
   p=pathlib.Path(d)
   (p/'stm32h7xx_hal.h').write_text('#include <stdint.h>\n#include <stdbool.h>\n#include <string.h>\n#define __packed\nuint32_t HAL_GetTick(void);\n')
   (p/'rp_config.h').write_text('#define DEV_OFFLINE 0\n#define DEV_ONLINE 1\n')
   (p/'main.c').write_text(harness)
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
