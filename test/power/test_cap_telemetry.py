"""Check real capacitor RX timestamps and heartbeat without enabling capacitor TX."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


class CapTelemetryTest(unittest.TestCase):
    def test_receive_time_and_sequence_survive_timeout_and_tx_stays_disabled(self):
        with tempfile.TemporaryDirectory() as folder:
            out = pathlib.Path(folder)
            (out / 'stm32h7xx_hal.h').write_text('#include <stdint.h>\nuint32_t HAL_GetTick(void);\n')
            (out / 'main.h').write_text('#include "stm32h7xx_hal.h"\n')
            (out / 'rp_config.h').write_text('#define DEV_ONLINE 0\n#define DEV_OFFLINE 1\n#define CAP_OFFLINE_CNT_MAX 300\n')
            (out / 'drv_can.h').write_text('')
            (out / 'cap_test.c').write_text(r'''
#include <assert.h>
#include <string.h>
#include "cap.h"
static uint32_t tick;
uint32_t HAL_GetTick(void){return tick;}
int main(void){
    cap.init(&cap);assert(!cap.status->power_received);
    assert(cap.status->status==DEV_OFFLINE);
    cap_rx_info_t wire={0};wire.now_chassis_power=85;wire.now_cap_V=0;
    tick=123;cap.rx(&cap,(uint8_t *)&wire);
    assert(cap.info->chassis_power==85 && cap.status->power_rx_count==1);
    assert(cap.status->power_update_ms==123 && cap.status->power_received);
    for(int i=0;i<70000;i++)cap.heartbeat(&cap);
    assert(cap.status->status==DEV_OFFLINE && cap.status->offline_cnt==300);
    assert(cap.status->power_update_ms==123 && cap.status->power_rx_count==1);
    wire.now_chassis_power=0;tick=1000;cap.rx(&cap,(uint8_t *)&wire);
    assert(cap.info->chassis_power==0 && cap.status->power_update_ms==1000);
    assert(cap.status->power_rx_count==2 && cap.status->status==DEV_ONLINE);
    cap_update(NULL,(uint8_t *)&wire);cap_update(&cap,NULL);
    assert(cap.status->power_rx_count==2);
    /* Linking without CAN_SendData succeeds only if the TX path stays inert. */
    cap_send_2E();return 0;
}
''', encoding='utf-8')
            command = ['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-I' + folder]
            command += ['-I' + str(ROOT / 'Application' / layer) for layer in ['DeviceLayer', 'ProtocolLayer']]
            command += [str(out / 'cap_test.c'), str(ROOT / 'Application/DeviceLayer/cap.c'),
                        str(ROOT / 'Application/ProtocolLayer/cap_protocol.c'), '-o', str(out / 'cap.exe')]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([str(out / 'cap.exe')], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
