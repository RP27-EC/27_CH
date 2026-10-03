/**
 ******************************************************************************
 * @file    drv_flash.c
 * @brief   Flash存储器底层驱动文件
 ******************************************************************************
 * @attention
 *
 * 该文件实现了STM32内部Flash存储器的读写操作，包括：
 * - 按字节、半字、字三种粒度写入Flash
 * - 读取Flash中的数据
 * - 擦除指定扇区
 *
 * Flash存储器在机器人系统中通常用于：
 * - 存储PID参数等需要掉电保存的配置
 * - 存储电机零点校准数据
 * - 存储系统运行日志或故障记录
 *
 * 注意：Flash写入前必须先擦除对应扇区，擦除操作会将扇区内所有数据置为0xFF
 *
 * Copyright 2024 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
/* 包含Flash驱动头文件、HAL兼容层头文件和字符串操作头文件 */
#include "drv_flash.h"
#include "stm32_hal_legacy.h"
#include "string.h"

/* Private macro -------------------------------------------------------------*/
/* 私有宏定义（当前未使用） */

/* Private function prototypes -----------------------------------------------*/
/* 私有函数前置声明（当前未使用） */

/* Private typedef -----------------------------------------------------------*/
/* 私有类型定义（当前未使用） */

/* Private variables ---------------------------------------------------------*/
/* 私有变量（当前未使用） */

/* Exported variables --------------------------------------------------------*/
/* 导出变量（当前未使用） */

/* Private functions ---------------------------------------------------------*/
/* 私有函数（当前未使用） */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  从Flash读取数据
 * @param  addr: Flash中被读取数据的起始地址（必须是有效的Flash地址）
 * @param  buf:  存储读取数据的目标缓冲区指针
 * @param  len:  读取数据的长度，以字节为单位
 * @retval 无
 * @note   通过memcpy直接从Flash地址读取数据到内存缓冲区
 *         Flash地址空间可以直接像内存一样读取，无需特殊操作
 */
void Flash_ReadData(uint32_t addr, uint32_t *buf, uint16_t len)
{
  /* 使用memcpy将Flash地址处的数据复制到目标缓冲区 */
  /* 将Flash地址强制转换为void*指针，memcpy按字节逐个复制 */
  memcpy(buf, (void*)addr, len);
}

/**
 * @brief  向Flash写入数据（按字节为单位）
 * @param  addr: Flash中写入数据的起始地址（必须是有效的Flash地址）
 * @param  data: 要写入的数据源缓冲区指针
 * @param  num:  写入数据的数量（字节个数）
 * @retval 无
 * @note   写入前必须先调用Flash_EraseSector擦除对应扇区
 *         写入过程：解锁Flash -> 逐字节编程 -> 锁定Flash
 *         每次写入一个字节（8位）
 */
void Flash_WriteByteData(uint32_t addr, uint8_t *data, uint16_t num)
{
  /* 解锁Flash，允许写入操作 */
  HAL_FLASH_Unlock();

  /* 逐字节写入数据 */
  for(uint16_t i = 0; i < num; i++)
  {
    /* 以字节为单位编程Flash，地址按1字节递增 */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr + i, data[i]);
  }

  /* 锁定Flash，禁止写入操作，保护数据安全 */
  HAL_FLASH_Lock();
}

/**
 * @brief  向Flash写入数据（按半字为单位）
 * @param  addr: Flash中写入数据的起始地址（必须是2字节对齐的地址）
 * @param  data: 要写入的数据源缓冲区指针
 * @param  num:  写入数据的数量（半字个数，每个半字为16位/2字节）
 * @retval 无
 * @note   半字为16位（2字节），写入地址必须2字节对齐
 *         地址偏移量为2*i（每个半字占2字节）
 */
void Flash_WriteHalfWordData(uint32_t addr, uint16_t *data, uint16_t num)
{
  /* 解锁Flash */
  HAL_FLASH_Unlock();

  /* 逐半字写入数据 */
  for(uint16_t i = 0; i < num; i++)
  {
    /* 以半字为单位编程Flash，地址偏移为2*i（2字节对齐） */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + 2 * i, data[i]);
  }

  /* 锁定Flash */
  HAL_FLASH_Lock();
}

/**
 * @brief  向Flash写入数据（按字为单位）
 * @param  addr: Flash中写入数据的起始地址（必须是4字节对齐的地址）
 * @param  data: 要写入的数据源缓冲区指针
 * @param  num:  写入数据的数量（字个数，每个字为32位/4字节）
 * @retval 无
 * @note   字为32位（4字节），写入地址必须4字节对齐
 *         地址偏移量为4*i（每个字占4字节）
 */
void Flash_WriteWordData(uint32_t addr, uint32_t *data, uint16_t num)
{
  /* 解锁Flash */
  HAL_FLASH_Unlock();

  /* 逐字写入数据 */
  for(uint16_t i = 0; i < num; i++)
  {
    /* 以字为单位编程Flash，地址偏移为4*i（4字节对齐） */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4 * i, data[i]);
  }

  /* 锁定Flash */
  HAL_FLASH_Lock();
}

///**
//  * @brief  向Flash写入数据（按双字为单位）
//  * @param  addr: Flash中写入数据的起始地址（必须是8字节对齐的地址）
//  * @param  data: 要写入的数据源缓冲区指针
//  * @param  num:  写入数据的数量（双字个数，每个双字为64位/8字节）
//  * @note   双字为64位（8字节），写入地址必须8字节对齐
//  *         该函数目前被注释掉，适用于支持双字编程的STM32系列
//  */
//void Flash_WriteDoubleWordData(uint32_t addr,uint64_t *data,uint16_t num)
//{
//	HAL_FLASH_Unlock();
//	for(uint16_t i=0;i<num;i++)
//	{
//		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr+8*i, data[i]);
//	}
//	HAL_FLASH_Lock();
//}

/**
 * @brief  擦除Flash指定扇区
 * @param  SectorNum: 需要擦除的目标扇区编号，取值范围为0至11
 * @retval uint32_t:  擦除错误状态
 *         - 0: 擦除成功
 *         - 其他值: 发生擦除错误的扇区编号
 * @note   擦除操作会将目标扇区内的所有数据恢复为默认值（0xFF）
 *         在写入Flash之前必须先擦除对应的扇区
 *         电压范围设置为FLASH_VOLTAGE_RANGE_3（2.7V~3.6V）
 *         警告：不要擦除程序代码所在的扇区，否则会导致系统崩溃
 */
uint32_t Flash_EraseSector(uint32_t SectorNum)
{
  FLASH_EraseInitTypeDef FLASH_Erase;  /* Flash擦除配置结构体 */
  uint32_t sectorError = 0;            /* 擦除错误状态，初始化为0（成功） */

  /* 解锁Flash，允许擦除操作 */
  HAL_FLASH_Unlock();

  /* 配置擦除参数 */
  FLASH_Erase.TypeErase = FLASH_TYPEERASE_SECTORS;  /* 擦除类型：按扇区擦除 */
  FLASH_Erase.Banks = FLASH_BANK_1;                  /* 指定Flash Bank 1 */
  FLASH_Erase.Sector = SectorNum;                     /* 设置要擦除的扇区编号 */
  FLASH_Erase.NbSectors = 1;                          /* 擦除扇区数量：1个 */
  FLASH_Erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;  /* 电压范围：2.7V~3.6V */

  /* 执行扇区擦除操作 */
  HAL_FLASHEx_Erase(&FLASH_Erase, &sectorError);

  /* 锁定Flash，保护数据安全 */
  HAL_FLASH_Lock();

  /* 返回错误状态，0表示成功 */
  return sectorError;
}

/**
 * @brief  擦除扇区11（便捷函数）
 * @param  无
 * @retval uint32_t: 擦除错误状态
 * @note   专门用于擦除第11号扇区
 *         在本项目中，扇区11通常被用作参数存储区
 *         调用Flash_EraseSector(11)实现具体擦除操作
 */
uint32_t Flash_EraseSector11(void)
{
  uint32_t sectorError;

  /* 调用通用扇区擦除函数，擦除第11号扇区 */
  sectorError = Flash_EraseSector(11);

  return sectorError;
}
