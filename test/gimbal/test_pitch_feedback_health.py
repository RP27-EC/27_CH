"""Exercise real state-frame decoding and watchdog with host stubs."""
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


class PitchFeedbackHealthTest(unittest.TestCase):
    def test_valid_invalid_timeout_and_recovery_are_distinguishable(self):
        header = (ROOT / 'Application/ProtocolLayer/communicate.h').read_text(encoding='utf-8-sig')
        source = (ROOT / 'Application/ProtocolLayer/communicate.c').read_text(encoding='utf-8-sig')
        functions = []
        for signature in ('void Communicate_Init(void)', 'void Board_Rx_State(const uint8_t *rxBuf)',
                          'void Communicate_Check_Timeout(void)'):
            start = source.index(signature)
            functions.append(source[start:source.index('\n}', start)+2])
        c = r'''
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
''' + re.sub(r'^#include[^\n]*', '', header, flags=re.M) + r'''
Board_Rx_Info_t board_rx_info;
Board_Tx_Info_t board_tx_info;
''' + '\n'.join(functions) + r'''
static void receive(uint8_t marker,float angle){
    uint8_t buf[8]={0};buf[5]=marker;memcpy(buf+1,&angle,4);Board_Rx_State(buf);
}
int main(void){
    Communicate_Init();
    CHECK(!board_rx_info.is_pitch_mech_online);
    receive(0xA5,.2f);
    CHECK(board_rx_info.is_pitch_mech_online && board_rx_info.is_state_online);
    CHECK(board_rx_info.state_rx_count==1 && board_rx_info.state_last_marker==0xA5);
    for(int i=0;i<500;i++){
        Communicate_Check_Timeout();receive(0xA5,.2f);
        CHECK(board_rx_info.is_pitch_mech_online);
    }
    CHECK(board_rx_info.state_invalid_count==0 && board_rx_info.state_timeout_count==0);
    receive(0,.2f);
    CHECK(!board_rx_info.is_pitch_mech_online && board_rx_info.is_state_online);
    CHECK(board_rx_info.pitch_mech_invalid_reason==1 && board_rx_info.state_invalid_count==1);
    receive(0xA5,NAN);
    CHECK(!board_rx_info.is_pitch_mech_online && board_rx_info.pitch_mech_invalid_reason==2);
    receive(0xA5,4.0f);
    CHECK(!board_rx_info.is_pitch_mech_online && board_rx_info.state_invalid_count==3);
    receive(0xA5,-.2f);
    CHECK(board_rx_info.is_pitch_mech_online && board_rx_info.pitch_mech_invalid_reason==0);
    for(int i=0;i<1000;i++)Communicate_Check_Timeout();
    CHECK(!board_rx_info.is_pitch_mech_online && !board_rx_info.is_state_online);
    CHECK(board_rx_info.pitch_mech_invalid_reason==3 && board_rx_info.state_timeout_count==1);
    receive(0xA5,.25f);
    CHECK(board_rx_info.is_pitch_mech_online && board_rx_info.pitch_mech_invalid_reason==0);
    CHECK(board_rx_info.pitch_mech_angle==.25f);
    for(int i=0;i<1000;i++)Communicate_Check_Timeout();
    CHECK(board_rx_info.state_timeout_count==2);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            src = pathlib.Path(directory) / 'health.c'
            exe = pathlib.Path(directory) / 'health.exe'
            src.write_text(c, encoding='utf-8')
            subprocess.run(['gcc', '-std=c99', str(src), '-lm', '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
