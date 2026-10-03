/**
  ******************************************************************************
  * @file    drv_flash.c
  * @brief   Flash存储器底层驱动 - 提供STM32内部Flash的读写和擦除功能
  *          本文件实现了对STM32F4系列芯片内部Flash的底层操作，包括：
  *          1. Flash数据读取（直接内存映射读取）
  *          2. Flash数据写入（支持字节/半字/字/双字四种粒度）
  *          3. Flash扇区擦除（写入前必须先擦除对应扇区）
  *          常用于掉电保存参数、存储校准数据、记录运行日志等场景。
  ******************************************************************************
  * @attention
  *   重要注意事项：
  *   1. 写入或擦除Flash时会暂时禁止中断，耗时较长
  *   2. 不要擦除程序代码所在的扇区，否则会导致程序无法运行
  *   3. STM32F407的Flash扇区0~11，扇区大小不一致（16KB/64KB/128KB）
  *   4. 写入前必须先擦除对应扇区，因为Flash只能从1写为0，不能从0写为1
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "drv_flash.h"
#include "string.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  从Flash读取数据
  * @param  addr: Flash中的起始地址（必须是有效的Flash地址）
  * @param  buf:  用于存储读取数据的目标缓冲区指针
  * @param  len:  要读取的字节数
  * @retval 无
  * @note   Flash在读取时等同于普通内存，可以直接通过地址指针访问
  *         读取操作不会对Flash内容造成任何影响
  */
void Flash_ReadData(uint32_t addr, uint32_t *buf, uint16_t len)
{
	/* 使用memcpy将Flash地址处的数据复制到目标缓冲区 */
	/* 将Flash地址强制转换为void*指针，兼容memcpy的参数类型 */
	memcpy(buf, (void*)addr, len);
}

/**
  * @brief  以字节(Byte)为单位向Flash写入数据
  * @param  addr: Flash中的目标起始地址
  * @param  data: 源数据缓冲区指针
  * @param  num:  要写入的字节数
  * @retval 无
  * @note   写入粒度：1字节（8位）
  *         注意: 写入前目标扇区必须已经擦除（全为0xFF）
  */
void Flash_WriteByteData(uint32_t addr, uint8_t *data, uint16_t num)
{
	HAL_FLASH_Unlock();  /* 解锁Flash，允许写入操作 */
	/* 逐字节写入Flash */
	for(uint16_t i = 0; i < num; i++)
	{
		/* addr+i: 目标地址，data[i]: 要写入的字节数据 */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr + i, data[i]);
	}
	HAL_FLASH_Lock();  /* 重新锁定Flash，防止意外写入 */
}

/**
  * @brief  以半字(HalfWord)为单位向Flash写入数据
  * @param  addr: Flash中的目标起始地址（建议2字节对齐）
  * @param  data: 源数据缓冲区指针（uint16_t类型数组）
  * @param  num:  要写入的半字数量
  * @retval 无
  * @note   写入粒度：2字节（16位）
  *         每次写入2字节，地址偏移为 2*i
  */
void Flash_WriteHalfWordData(uint32_t addr, uint16_t *data, uint16_t num)
{
	HAL_FLASH_Unlock();
	for(uint16_t i = 0; i < num; i++)
	{
		/* 半字写入: 地址每次偏移2字节 */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + 2 * i, data[i]);
	}
	HAL_FLASH_Lock();
}

/**
  * @brief  以字(Word)为单位向Flash写入数据
  * @param  addr: Flash中的目标起始地址（建议4字节对齐）
  * @param  data: 源数据缓冲区指针（uint32_t类型数组）
  * @param  num:  要写入的字数量
  * @retval 无
  * @note   写入粒度：4字节（32位）
  *         每次写入4字节，地址偏移为 4*i
  */
void Flash_WriteWordData(uint32_t addr, uint32_t *data, uint16_t num)
{
	HAL_FLASH_Unlock();
	for(uint16_t i = 0; i < num; i++)
	{
		/* 字写入: 地址每次偏移4字节 */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4 * i, data[i]);
	}
	HAL_FLASH_Lock();
}

/**
  * @brief  以双字(DoubleWord)为单位向Flash写入数据
  * @param  addr: Flash中的目标起始地址（建议8字节对齐）
  * @param  data: 源数据缓冲区指针（uint64_t类型数组）
  * @param  num:  要写入的双字数量
  * @retval 无
  * @note   写入粒度：8字节（64位）
  *         STM32F4系列支持双字写入，每次写入8字节
  */
void Flash_WriteDoubleWordData(uint32_t addr, uint64_t *data, uint16_t num)
{
	HAL_FLASH_Unlock();
	for(uint16_t i = 0; i < num; i++)
	{
		/* 双字写入: 地址每次偏移8字节 */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + 8 * i, data[i]);
	}
	HAL_FLASH_Lock();
}

/**
  * @brief  擦除指定的Flash扇区
  * @param  SectorNum: 要擦除的扇区编号，取值范围0~11（STM32F407）
  * @retval uint32_t: 扇区错误码
  *         - 0: 擦除成功
  *         - 非0: 发生错误的扇区编号
  * @note   重要:
  *         1. 写入Flash前必须先擦除对应扇区
  *         2. 擦除操作会将整个扇区的数据清零（全部变为0xFF）
  *         3. 不要擦除程序代码所在的扇区！
  *         4. 扇区大小: 扇区0=16KB, 扇区1=16KB, 扇区2=16KB, 扇区3=16KB,
  *            扇区4=64KB, 扇区5~11=128KB
  *         5. 电压范围设置为FLASH_VOLTAGE_RANGE_3(2.7~3.6V)
  */
uint32_t Flash_EraseSector(uint32_t SectorNum)
{
	FLASH_EraseInitTypeDef FLASH_Erase;  /* 擦除操作配置结构体 */
	uint32_t sectorError = 0;            /* 错误码，擦除出错时存储出错扇区号 */

	HAL_FLASH_Unlock();  /* 解锁Flash */

	/* 配置擦除参数 */
	FLASH_Erase.TypeErase = FLASH_TYPEERASE_SECTORS;  /* 按扇区擦除模式 */
	FLASH_Erase.Banks = FLASH_BANK_1;                  /* 使用Bank1 */
	FLASH_Erase.Sector = SectorNum;                     /* 目标扇区编号 */
	FLASH_Erase.NbSectors = 1;                          /* 擦除1个扇区 */
	FLASH_Erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;   /* 电压范围2.7~3.6V */

	/* 执行扇区擦除操作 */
	HAL_FLASHEx_Erase(&FLASH_Erase, &sectorError);

	HAL_FLASH_Lock();  /* 重新锁定Flash */

	return sectorError;  /* 返回错误码，0表示成功 */
}

/**
  * @brief  擦除Flash扇区11（便捷函数）
  * @retval uint32_t: 扇区错误码，0表示成功
  * @note   扇区11是STM32F407中最后一个128KB扇区，常用于存储用户数据
  *         该扇区位于Flash最末尾，通常不会被程序代码占用
  */
uint32_t Flash_EraseSector11(void)
{
	uint32_t sectorError;

	sectorError = Flash_EraseSector(11);  /* 调用通用擦除函数擦除扇区11 */

	return sectorError;
}

