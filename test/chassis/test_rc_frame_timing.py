"""Exercise actual DBUS receive callback timing with the hardware parser stubbed."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


class RcFrameTimingTest(unittest.TestCase):
    def test_valid_frame_intervals_reconnect_and_counter_rollover(self):
        source = (ROOT/'Application/ProtocolLayer/rc_protocol.c').read_text(encoding='utf-8-sig')
        start = source.index('void USART5_rxDataHandler(')
        callback = source[start:source.index('\n}', start)+2]
        harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#undef errno
enum {NONE_ERR, DEV_DATA_ERR};
static uint32_t tick;
uint32_t HAL_GetTick(void){return tick;}
typedef struct {int offline_cnt;uint16_t frame_seq;uint32_t frame_update_ms,frame_interval_ms;bool frame_received;} Info;
typedef struct Sensor Sensor;
struct Sensor {Info *info;int errno;void(*update)(Sensor*,uint8_t*);void(*check)(Sensor*);};
static Info info;
static int calls,filtered;
static void update(Sensor *s,uint8_t *b){(void)s;(void)b;calls++;}
static void check(Sensor *s){(void)s;}
static Sensor rc_sensor={&info,NONE_ERR,update,check};
static void rc_interrupt_update(Sensor *s){(void)s;filtered++;}
''' + callback + r'''
int main(void){
 uint8_t frame[18]={0};
 /* 首帧恰在 tick=0，也应被记录为已收到，下一帧的间隔不能再丢掉。 */
 tick=0;USART5_rxDataHandler(frame);assert(info.frame_received);assert(info.frame_interval_ms==0);
 tick=14;USART5_rxDataHandler(frame);assert(info.frame_interval_ms==14);assert(info.frame_update_ms==14);
 tick=42;USART5_rxDataHandler(frame);assert(info.frame_interval_ms==28);
 USART5_rxDataHandler(frame);assert(info.frame_interval_ms==0);
 rc_sensor.errno=DEV_DATA_ERR;tick=56;USART5_rxDataHandler(frame);assert(!info.frame_received);
 assert(info.frame_interval_ms==0);assert(info.frame_update_ms==42);
 rc_sensor.errno=NONE_ERR;tick=70;USART5_rxDataHandler(frame);assert(info.frame_received);assert(info.frame_interval_ms==0);
 tick=84;USART5_rxDataHandler(frame);assert(info.frame_interval_ms==14);
 tick=1000;USART5_rxDataHandler(frame);assert(info.frame_interval_ms==916);
 info.frame_seq=65535;info.frame_update_ms=UINT32_MAX-5;tick=8;
 USART5_rxDataHandler(frame);assert(info.frame_seq==0);assert(info.frame_interval_ms==14);
 assert(calls==9 && filtered==9);assert(info.offline_cnt==0);
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            src=pathlib.Path(directory)/'timing.c'
            exe=pathlib.Path(directory)/'timing.exe'
            src.write_text(harness,encoding='utf-8')
            compiled=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(src),'-o',str(exe)],capture_output=True,text=True)
            self.assertEqual(compiled.returncode,0,compiled.stderr)
            result=subprocess.run([str(exe)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)


if __name__ == '__main__':
    unittest.main()
