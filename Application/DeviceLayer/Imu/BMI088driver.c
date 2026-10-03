/**
  ******************************************************************************
  * @file           : BMI088driver.c
  * @brief          : BMI088六轴IMU传感器驱动文件
  * @details        : 实现BMI088加速度计和陀螺仪的初始化、寄存器读写、
  *                   数据采集等功能。支持SPI和IIC两种通信方式。
  *                   BMI088是一款高精度六轴惯性测量单元，集成三轴加速度计
  *                   和三轴陀螺仪，常用于机器人姿态检测。
  ******************************************************************************
  */

#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"

/**
  * @brief  加速度计灵敏度系数（当前配置为±3g量程）
  * @note   灵敏度 = 量程 / ADC最大值，3g量程对应约0.000897 g/LSB
  *          根据实际量程需求可修改为其他值（如BMI088_ACCEL_6G_SEN等）
  */
fp32 BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;

/**
  * @brief  陀螺仪灵敏度系数（当前配置为±2000°/s量程）
  * @note   灵敏度 = 量程 / ADC最大值，2000°/s量程对应约0.061°/s/LSB
  *          根据实际量程需求可修改为其他值（如BMI088_GYRO_500_SEN等）
  */
fp32 BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;



/**
  * ==================== SPI通信方式下的寄存器读写宏 ====================
  * 通过宏定义封装SPI读写操作，自动处理片选信号的拉低/拉高
  * BMI088加速度计和陀螺仪分别有独立的片选引脚
  */
#if defined(BMI088_USE_SPI)

/**
  * @brief  加速度计写单个寄存器宏
  * @param  reg: 寄存器地址
  * @param  data: 要写入的数据
  * @note   流程：拉低片选 -> 写寄存器 -> 拉高片选
  */
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }

/**
  * @brief  加速度计读单个寄存器宏
  * @param  reg: 寄存器地址
  * @param  data: 读取到的数据存入此变量
  * @note   BMI088 SPI读操作需要：先发寄存器地址(OR 0x80)，再发空字节，最后读取数据
  *         0x80为读标志位，0x55为SPI通信中的哑字节（dummy byte）
  */
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }

/**
  * @brief  加速度计连续读取多个寄存器宏
  * @param  reg: 起始寄存器地址
  * @param  data: 存储读取数据的缓冲区
  * @param  len: 读取的字节长度
  */
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

/**
  * @brief  陀螺仪写单个寄存器宏
  */
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }

/**
  * @brief  陀螺仪读单个寄存器宏
  */
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }

/**
  * @brief  陀螺仪连续读取多个寄存器宏
  */
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

/* 底层SPI读写函数前向声明 */
static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

/**
  * ==================== IIC通信方式（预留） ====================
  * 当使用IIC通信时，在此分支实现对应的读写函数
  */
#elif defined(BMI088_USE_IIC)


#endif

/**
  * @brief  加速度计寄存器配置表
  * @details 二维数组，每行包含3个元素：[寄存器地址, 配置值, 错误码]
  *          用于初始化时按顺序写入配置并验证，若验证失败返回对应错误码
  *
  * 配置项说明：
  * [0] PWR_CTRL    - 使能加速度计
  * [1] PWR_CONF    - 设置工作模式为正常模式
  * [2] ACC_CONF    - 配置为正常模式、800Hz输出数据率
  * [3] ACC_RANGE   - 设置量程为±3g
  * [4] INT1_IO_CTRL - 配置INT1引脚为推挽输出、低电平有效
  * [5] INT_MAP_DATA - 将数据就绪中断映射到INT1引脚
  */
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
};

/**
  * @brief  陀螺仪寄存器配置表
  * @details 二维数组，每行包含3个元素：[寄存器地址, 配置值, 错误码]
  *
  * 配置项说明：
  * [0] RANGE       - 设置量程为±2000°/s
  * [1] BANDWIDTH   - 设置带宽为1000Hz（滤波器带宽116Hz）
  * [2] LPM1        - 设置正常工作模式（非低功耗）
  * [3] CTRL        - 使能数据就绪中断（DRDY）
  * [4] INT3_INT4_IO_CONF - 配置INT3引脚为推挽输出、低电平有效
  * [5] INT3_INT4_IO_MAP  - 将DRDY中断映射到INT3引脚
  */
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};

/**
  * @brief  BMI088整体初始化函数
  * @retval 错误码，BMI088_NO_ERROR表示初始化成功
  * @details 初始化流程：GPIO和SPI初始化 -> 加速度计初始化 -> 陀螺仪初始化
  *          任一子模块初始化失败将返回对应的错误码
  */
uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;

    /* 初始化GPIO和SPI通信接口 */
    BMI088_GPIO_init();
    BMI088_com_init();

    /* 初始化加速度计和陀螺仪，使用位或操作合并错误码 */
    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    return error;
}

/**
  * @brief  加速度计初始化函数
  * @retval BMI088_NO_ERROR: 初始化成功
  *         BMI088_NO_SENSOR: 芯片ID不匹配，未检测到传感器
  *         其他值: 对应寄存器配置验证失败的错误码
  * @details 初始化步骤：
  *          1. 检查SPI通信是否正常（读取芯片ID）
  *          2. 软件复位传感器
  *          3. 再次读取芯片ID验证"who am I"
  *          4. 逐个写入配置寄存器并回读验证
  */
bool_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    /* 第一步：检查SPI通信是否正常（读取芯片ID进行测试） */
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* 第二步：加速度计软件复位 */
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    /* 第三步：复位后再次检查通信是否正常 */
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* 第四步：校验芯片ID，确认是BMI088加速度计 */
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* 第五步：逐个写入配置寄存器，写入后回读验证是否正确 */
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
        /* 写入配置值到对应寄存器 */
        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* 回读寄存器值进行验证 */
        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* 如果回读值与写入值不一致，返回对应的错误码 */
        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

/**
  * @brief  陀螺仪初始化函数
  * @retval BMI088_NO_ERROR: 初始化成功
  *         BMI088_NO_SENSOR: 芯片ID不匹配，未检测到传感器
  *         其他值: 对应寄存器配置验证失败的错误码
  * @details 初始化步骤与加速度计类似：
  *          1. 检查SPI通信是否正常
  *          2. 软件复位传感器
  *          3. 校验芯片ID
  *          4. 逐个写入配置寄存器并回读验证
  */
bool_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    /* 检查SPI通信是否正常 */
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* 陀螺仪软件复位 */
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    /* 复位后再次检查通信是否正常 */
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* 校验芯片ID，确认是BMI088陀螺仪 */
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* 逐个写入配置寄存器，写入后回读验证 */
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        /* 写入配置值 */
        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* 回读验证 */
        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* 验证失败则返回错误码 */
        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}





/**
  * @brief  读取BMI088加速度计、陀螺仪和温度数据
  * @param  gyro[3]:    输出参数，存储三轴角速度数据（单位：°/s）
  * @param  accel[3]:   输出参数，存储三轴加速度数据（单位：m/s²）
  * @param  temperate:  输出参数，存储芯片温度数据（单位：°C）
  * @details 数据读取流程：
  *          1. 从加速度计读取6字节原始数据（X/Y/Z各2字节，小端序）
  *          2. 将原始16位整数乘以灵敏度系数转换为物理量
  *          3. 从陀螺仪读取8字节数据（包含1字节芯片ID + 6字节角速度 + 1字节校验）
  *          4. 校验芯片ID后解析角速度数据
  *          5. 读取温度寄存器并转换为摄氏度
  */
void BMI088_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};  /* 数据缓冲区 */
    int16_t bmi088_raw_temp;  /* 临时变量，存储16位原始数据 */

    /* ==================== 读取加速度计数据 ==================== */
    /* 从ACCEL_XOUT_L寄存器开始连续读取6字节（X_L, X_H, Y_L, Y_H, Z_L, Z_H） */
    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    /* 将小端格式的两个字节组合成16位有符号整数，再乘以灵敏度得到加速度值 */
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];  /* X轴：高字节<<8 | 低字节 */
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;         /* 乘以灵敏度转换为物理量 */
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];  /* Y轴 */
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];  /* Z轴 */
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    /* ==================== 读取陀螺仪数据 ==================== */
    /* 从GYRO_CHIP_ID开始读取8字节，包含ID和角速度数据 */
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);

    /* 校验芯片ID，确保读取的数据有效 */
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        /* 陀螺仪数据从buf[2]开始，buf[0]为ID，buf[1]为状态 */
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];  /* X轴角速度 */
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];  /* Y轴角速度 */
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];  /* Z轴角速度 */
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }

    /* ==================== 读取温度数据 ==================== */
    /* 温度数据为12位，分布在TEMP_M和TEMP_L两个寄存器中 */
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    /* 组合高3位和低8位，得到12位原始温度值 */
    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    /* 温度值是有符号的12位数据，超过1023需要进行补码转换 */
    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;  /* 转换为负温度值 */
    }

    /* 应用温度系数和偏移量，转换为摄氏度 */
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/**
  * ==================== SPI通信方式下的底层读写函数实现 ====================
  */
#if defined(BMI088_USE_SPI)

/**
  * @brief  SPI写单个寄存器（底层函数）
  * @param  reg: 目标寄存器地址
  * @param  data: 要写入的数据
  * @note   先发送寄存器地址，再发送数据字节
  */
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);    /* 发送寄存器地址 */
    BMI088_read_write_byte(data);   /* 发送要写入的数据 */
}

/**
  * @brief  SPI读单个寄存器（底层函数）
  * @param  reg: 目标寄存器地址
  * @param  return_data: 读取到的数据存入此指针指向的变量
  * @note   BMI088读操作：地址的最高位置1（|0x80），然后发哑字节读取数据
  */
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);  /* 发送寄存器地址（最高位置1表示读操作） */
    *return_data = BMI088_read_write_byte(0x55);  /* 发送哑字节并读取返回数据 */
}

/**
  * @brief  SPI连续读取多个寄存器（底层函数）
  * @param  reg: 起始寄存器地址
  * @param  buf: 存储读取数据的缓冲区指针
  * @param  len: 要读取的字节数
  * @note   连续读取时，BMI088会自动递增寄存器地址
  */
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* 发送起始寄存器地址（最高位置1） */
    BMI088_read_write_byte(reg | 0x80);

    /* 循环读取len个字节数据 */
    while (len != 0)
    {
        *buf = BMI088_read_write_byte(0x55);  /* 每次发送哑字节以接收一个数据字节 */
        buf++;   /* 缓冲区指针后移 */
        len--;   /* 剩余长度递减 */
    }
}

/**
  * ==================== IIC通信方式（预留） ====================
  */
#elif defined(BMI088_USE_IIC)

#endif
