"""Exercise real USART1 callbacks with a DMA abort/completion interrupt model."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


def function(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


class UartReceiveTest(unittest.TestCase):
    def test_idle_abort_cannot_reenter_parser_and_completion_restarts_receive(self):
        source = (ROOT / 'Application/DriverLayer/drv_uart.c').read_text(encoding='utf-8-sig')
        callbacks = '\n'.join(function(source, s) for s in [
            'void HAL_UART_RxCpltCallback(',
            'void HAL_UART_ErrorCallback(',
            'static void uart_rx_idle_callback(UART_HandleTypeDef* huart)\n{'])
        harness = r'''
#include <assert.h>
#include <stdint.h>
#define USART1_RX_BUF_LEN 400
#define USART10_RX_BUF_LEN 100
#define USART7_RX_BUF_LEN 100
#define USART8_RX_BUF_LEN 100
#define USART9_RX_BUF_LEN 100
#define DMA_SxCR_EN 1U
#define DMA_IT_TC 2U
#define DMA_IT_HT 4U
#define DMA_IT_TE 8U
#define DMA_IT_DME 16U
#define DMA_IT_FE 32U
#define UART5_RC_USE_HAL_TOIDLE_DMA 0
#define UART_CLEAR_OREF 1U
#define UART_CLEAR_NEF 2U
#define UART_CLEAR_PEF 4U
#define UART_CLEAR_FEF 8U
#define UART_CLEAR_IDLEF 16U
typedef struct {uint32_t CR,NDTR;} DMA_Stream_TypeDef;
typedef struct {DMA_Stream_TypeDef *Instance;} DMA_HandleTypeDef;
typedef struct {DMA_HandleTypeDef *hdmarx;} UART_HandleTypeDef;
static DMA_Stream_TypeDef stream;
static DMA_HandleTypeDef dma={&stream};
static UART_HandleTypeDef huart1={&dma},huart10,huart7,huart8,huart9;
static uint8_t usart1_dma_rxbuf[400],usart10_dma_rxbuf[100],usart7_dma_rxbuf[100],usart8_dma_rxbuf[100],usart9_dma_rxbuf[100];
static int parsed,restarted,rx_busy=1,dma_busy=1;static uint16_t parsed_length;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
static void disable_dma(DMA_HandleTypeDef *handle){
    handle->Instance->CR &= ~DMA_SxCR_EN;
    /* STM32 HAL documents that clearing EN may generate TC. */
    if(handle->Instance->CR & DMA_IT_TC) HAL_UART_RxCpltCallback(&huart1);
    /* Avoid hanging the test if the unsafe callback has just restarted DMA. */
    handle->Instance->CR &= ~DMA_SxCR_EN;
}
#define __HAL_DMA_DISABLE(h) disable_dma(h)
#define __HAL_DMA_ENABLE(h) ((h)->Instance->CR |= DMA_SxCR_EN)
#define __HAL_DMA_DISABLE_IT(h,m) ((h)->Instance->CR &= ~(m))
#define __HAL_DMA_GET_COUNTER(h) ((h)->Instance->NDTR)
#define __HAL_UART_CLEAR_IDLEFLAG(h) ((void)(h))
#define __HAL_UART_CLEAR_FLAG(h,m) ((void)(h),(void)(m))
static void HAL_UART_DMAStop(UART_HandleTypeDef *h){
    if(rx_busy){h->hdmarx->Instance->CR=0;dma_busy=0;rx_busy=0;}
}
void HAL_UART_AbortReceive(UART_HandleTypeDef *h){h->hdmarx->Instance->CR=0;dma_busy=rx_busy=0;}
static void HAL_UART_Receive_DMA(UART_HandleTypeDef *h,uint8_t *buf,uint16_t n){
    (void)buf;if(dma_busy && !rx_busy)return;
    restarted++;dma_busy=rx_busy=1;h->hdmarx->Instance->NDTR=n;
    h->hdmarx->Instance->CR=DMA_SxCR_EN|DMA_IT_TC|DMA_IT_HT;
}
static void USART1_rxDataHandler(const uint8_t *b,uint16_t n){(void)b;parsed++;parsed_length=n;}
#define USART10_rxDataHandler(b) ((void)(b))
#define USART7_rxDataHandler(b) ((void)(b))
#define USART8_rxDataHandler(b) ((void)(b))
#define USART9_rxDataHandler(b) ((void)(b))
'''
        body = r'''
int main(void){
    stream.CR=DMA_SxCR_EN|DMA_IT_TC|DMA_IT_HT;stream.NDTR=377;
    uart_rx_idle_callback(&huart1);
    assert(parsed==1 && parsed_length==23 && restarted==1);
    stream.CR=0;stream.NDTR=0;HAL_UART_RxCpltCallback(&huart1);
    assert(parsed==2 && parsed_length==400 && restarted==2);
    HAL_UART_ErrorCallback(&huart1);
    assert(parsed==2 && restarted==3 && stream.NDTR==400);
    /* DME ends UART RX but leaves its DMA handle BUSY. */
    rx_busy=0;dma_busy=1;HAL_UART_ErrorCallback(&huart1);
    assert(parsed==2 && restarted==4 && rx_busy && dma_busy);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as folder:
            src = pathlib.Path(folder) / 'driver.c'
            exe = pathlib.Path(folder) / 'driver.exe'
            src.write_text(harness + callbacks + body, encoding='utf-8')
            result = subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                                     str(src), '-o', str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
