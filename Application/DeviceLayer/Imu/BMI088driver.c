/**
 * @file    BMI088driver.c
 * @brief   BMI088 IMU 传感器驱动层
 *
 * 本文件实现了 Bosch BMI088 六轴 IMU 传感器的底层驱动，包括：
 *   1. 加速度计和陀螺仪的初始化配置
 *   2. SPI/IIC 通信接口封装
 *   3. 原始数据读取和转换
 *   4. 温度数据读取
 *
 * BMI088 是一款高性能六轴惯性测量单元，包含：
 *   - 三轴加速度计（量程可选：3g/6g/12g/24g）
 *   - 三轴陀螺仪（量程可选：125dps~2000dps）
 *
 * 支持 SPI 和 IIC 两种通信方式（通过宏定义选择）
 *
 * @note   本驱动基于 BMI088 官方数据手册编写
 * @version V1.0
 */

/* Includes ------------------------------------------------------------------*/
#include "BMI088driver.h"   /* BMI088 驱动头文件，包含寄存器地址和配置宏 */
#include "BMI088reg.h"      /* BMI088 寄存器定义 */
#include "BMI088Middleware.h" /* 硬件抽象层，包含 SPI 读写和延时函数 */

/* Exported variables --------------------------------------------------------*/
/**
 * @brief 加速度计灵敏度因子
 * 根据配置的量程选择对应的转换因子
 * 默认：BMI088_ACCEL_3G_SEN = 3G 量程，输出单位为 m/s^2
 */
fp32 BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;

/**
 * @brief 陀螺仪灵敏度因子
 * 根据配置的量程选择对应的转换因子
 * 默认：BMI088_GYRO_2000_SEN = 2000dps 量程，输出单位为 rad/s
 */
fp32 BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

/* ==================== SPI 通信方式下的宏定义 ==================== */
#if defined(BMI088_USE_SPI)

/**
 * @brief 加速度计 SPI 写单寄存器宏
 * 流程：拉低 CS -> 发送寄存器地址+数据 -> 拉高 CS
 */
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     /* 拉低加速度计 CS 片选 */ \
        BMI088_write_single_reg((reg), (data));  /* 写入寄存器 */ \
        BMI088_ACCEL_NS_H();                     /* 拉高加速度计 CS 片选 */ \
    }

/**
 * @brief 加速度计 SPI 读单寄存器宏
 * 流程：拉低 CS -> 发送寄存器地址(最高位置1) -> 读取数据 -> 拉高 CS
 * 注意：读操作时寄存器地址最高位需置1（0x80）
 */
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    /* 拉低加速度计 CS 片选 */ \
        BMI088_read_write_byte((reg) | 0x80);   /* 发送寄存器地址（读标志位） */ \
        BMI088_read_write_byte(0x55);           /* 发送 dummy 字节，产生时钟 */ \
        (data) = BMI088_read_write_byte(0x55);  /* 读取数据 */ \
        BMI088_ACCEL_NS_H();                    /* 拉高加速度计 CS 片选 */ \
    }

/**
 * @brief 加速度计 SPI 读多字节寄存器宏
 * 用于连续读取多个寄存器（如 XYZ 三轴数据）
 */
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       /* 拉低加速度计 CS 片选 */ \
        BMI088_read_write_byte((reg) | 0x80);      /* 发送寄存器地址（读标志位） */ \
        BMI088_read_muli_reg(reg, data, len);      /* 连续读取多个字节 */ \
        BMI088_ACCEL_NS_H();                       /* 拉高加速度计 CS 片选 */ \
    }

/**
 * @brief 陀螺仪 SPI 写单寄存器宏
 */
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     /* 拉低陀螺仪 CS 片选 */ \
        BMI088_write_single_reg((reg), (data)); /* 写入寄存器 */ \
        BMI088_GYRO_NS_H();                     /* 拉高陀螺仪 CS 片选 */ \
    }

/**
 * @brief 陀螺仪 SPI 读单寄存器宏
 */
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     /* 拉低陀螺仪 CS 片选 */ \
        BMI088_read_single_reg((reg), &(data)); /* 读取寄存器 */ \
        BMI088_GYRO_NS_H();                     /* 拉高陀螺仪 CS 片选 */ \
    }

/**
 * @brief 陀螺仪 SPI 读多字节寄存器宏
 */
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         /* 拉低陀螺仪 CS 片选 */ \
        BMI088_read_muli_reg((reg), (data), (len)); /* 连续读取多个字节 */ \
        BMI088_GYRO_NS_H();                         /* 拉高陀螺仪 CS 片选 */ \
    }

/* SPI 底层读写函数声明 */
static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)
/* IIC 通信方式下的函数声明（待实现） */
#endif

/**
 * @brief 加速度计寄存器配置表
 * 每行格式：{寄存器地址, 配置值, 错误码}
 * 按顺序写入以下寄存器完成加速度计初始化：
 *   1. PWR_CTRL: 使能加速度计
 *   2. PWR_CONF: 设置为正常工作模式
 *   3. ACC_CONF: 设置数据率和带宽（800Hz）
 *   4. ACC_RANGE: 设置量程（3g）
 *   5. INT1_IO_CTRL: 配置 INT1 中断引脚
 *   6. INT_MAP_DATA: 将数据就绪中断映射到 INT1
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
 * @brief 陀螺仪寄存器配置表
 * 每行格式：{寄存器地址, 配置值, 错误码}
 * 按顺序写入以下寄存器完成陀螺仪初始化：
 *   1. GYRO_RANGE: 设置量程（2000dps）
 *   2. GYRO_BANDWIDTH: 设置带宽（1000Hz ODR, 116Hz BW）
 *   3. GYRO_LPM1: 设置为正常低功耗模式
 *   4. GYRO_CTRL: 使能数据就绪中断
 *   5. INT3_INT4_IO_CONF: 配置 INT3 中断引脚
 *   6. INT3_INT4_IO_MAP: 将数据就绪中断映射到 INT3
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

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  BMI088 传感器总初始化函数
 *         初始化 GPIO、SPI 通信接口，然后分别初始化加速度计和陀螺仪
 * @return 错误码：BMI088_NO_ERROR 表示成功，其他值表示对应的错误
 *
 * @note   初始化顺序：GPIO/SPI -> 加速度计 -> 陀螺仪
 *         如果任一传感器初始化失败，会返回对应的错误码
 */
uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;

    /* 初始化 GPIO 和 SPI 通信接口 */
    BMI088_GPIO_init();   /* 初始化片选引脚等 GPIO */
    BMI088_com_init();    /* 初始化 SPI 通信接口 */

    /* 初始化加速度计（失败时返回错误码） */
    error |= bmi088_accel_init();
    /* 初始化陀螺仪（失败时返回错误码） */
    error |= bmi088_gyro_init();

    return error;  /* 返回综合错误码 */
}

/**
 * @brief  BMI088 加速度计初始化
 *         完成通信检查、软件复位、芯片 ID 验证、寄存器配置
 * @return true: 初始化成功；false: 初始化失败（芯片 ID 不匹配或寄存器配置失败）
 *
 * @note   初始化流程：
 *           1. 读取芯片 ID 检查通信是否正常
 *           2. 软件复位
 *           3. 再次读取芯片 ID 验证复位后通信正常
 *           4. 校验芯片 ID 值
 *           5. 逐个写入配置寄存器并回读验证
 */
bool_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    /* 步骤1：检查 SPI 通信是否正常（读取芯片 ID 两次） */
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);  /* 等待传感器响应 */
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* 步骤2：加速度计软件复位 */
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);  /* 复位后需要较长等待时间 */

    /* 步骤3：复位后检查通信是否恢复正常 */
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* 步骤4：校验芯片 ID（"Who Am I" 检查） */
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;  /* 芯片 ID 不匹配，可能未连接或芯片损坏 */
    }

    /* 步骤5：逐个写入配置寄存器并回读验证 */
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
        /* 写入配置值 */
        BMI088_accel_write_single_reg(
            write_BMI088_accel_reg_data_error[write_reg_num][0],  /* 寄存器地址 */
            write_BMI088_accel_reg_data_error[write_reg_num][1]); /* 配置值 */
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* 回读验证 */
        BMI088_accel_read_single_reg(
            write_BMI088_accel_reg_data_error[write_reg_num][0],  /* 寄存器地址 */
            res);                                                   /* 读回的值 */
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* 比较写入值和读回值，不一致则返回对应错误码 */
        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];  /* 返回错误码 */
        }
    }
    return BMI088_NO_ERROR;  /* 所有寄存器配置成功 */
}

/**
 * @brief  BMI088 陀螺仪初始化
 *         完成通信检查、软件复位、芯片 ID 验证、寄存器配置
 * @return true: 初始化成功；false: 初始化失败
 *
 * @note   初始化流程与加速度计类似，但寄存器地址和配置不同
 */
bool_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    /* 步骤1：检查 SPI 通信是否正常 */
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* 步骤2：陀螺仪软件复位 */
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);  /* 复位后等待 */

    /* 步骤3：复位后检查通信 */
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* 步骤4：校验芯片 ID */
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;  /* 芯片 ID 不匹配 */
    }

    /* 步骤5：逐个写入配置寄存器并回读验证 */
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        /* 写入配置值 */
        BMI088_gyro_write_single_reg(
            write_BMI088_gyro_reg_data_error[write_reg_num][0],  /* 寄存器地址 */
            write_BMI088_gyro_reg_data_error[write_reg_num][1]); /* 配置值 */
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* 回读验证 */
        BMI088_gyro_read_single_reg(
            write_BMI088_gyro_reg_data_error[write_reg_num][0],  /* 寄存器地址 */
            res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* 比较写入值和读回值 */
        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;  /* 所有寄存器配置成功 */
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  读取 BMI088 加速度计、陀螺仪和温度数据
 * @param[out] gyro[3]    陀螺仪三轴数据（rad/s），索引 0/1/2 分别对应 X/Y/Z
 * @param[out] accel[3]   加速度计三轴数据（m/s^2），索引 0/1/2 分别对应 X/Y/Z
 * @param[out] temperate  温度数据（摄氏度）
 *
 * @note   数据读取流程：
 *           1. 连续读取加速度计 XYZ 轴 6 字节数据
 *           2. 高低字节组合为 16 位有符号整数
 *           3. 乘以灵敏度因子转换为物理单位
 *           4. 读取陀螺仪数据（先读芯片 ID 验证通信）
 *           5. 读取温度数据（特殊格式：11位有效数据）
 */
void BMI088_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};  /* 临时缓冲区 */
    int16_t bmi088_raw_temp;  /* 临时变量，用于存储原始 16 位数据 */

    /* ========== 读取加速度计数据 ========== */
    /* 连续读取 6 字节：X_L, X_H, Y_L, Y_H, Z_L, Z_H */
    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    /* X 轴加速度：高低字节组合，乘以灵敏度因子 */
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    /* Y 轴加速度 */
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    /* Z 轴加速度 */
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    /* ========== 读取陀螺仪数据 ========== */
    /* 先读取 8 字节（包含芯片 ID + 6 字节 XYZ 数据） */
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);

    /* 验证芯片 ID（buf[0] 应为陀螺仪芯片 ID） */
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        /* X 轴角速度 */
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;

        /* Y 轴角速度 */
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;

        /* Z 轴角速度 */
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }

    /* ========== 读取温度数据 ========== */
    /* 读取温度寄存器（2 字节，11 位有效数据） */
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    /* 温度数据格式：高 3 位 + 低 8 位，共 11 位有符号数 */
    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    /* 11 位有符号数转换：大于 1023 时需要减去 2048 得到负值 */
    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    /* 温度转换公式：温度 = 原始值 * 灵敏度因子 + 偏移量 */
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/* ==================== SPI 底层读写函数实现 ==================== */
#if defined(BMI088_USE_SPI)

/**
 * @brief  SPI 写单个寄存器
 * @param  reg  寄存器地址
 * @param  data 要写入的数据
 *
 * @note   发送格式：[寄存器地址][数据]
 *         写操作时寄存器地址最高位为 0
 */
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);   /* 发送寄存器地址 */
    BMI088_read_write_byte(data);  /* 发送数据 */
}

/**
 * @brief  SPI 读单个寄存器
 * @param  reg         寄存器地址
 * @param  return_data 读取到的数据指针
 *
 * @note   发送格式：[寄存器地址 | 0x80][dummy] -> 接收数据
 *         读操作时寄存器地址最高位置1（0x80）
 */
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);     /* 发送地址（读标志位） */
    *return_data = BMI088_read_write_byte(0x55);  /* 读取数据（发送 dummy） */
}

/**
 * @brief  SPI 连续读取多个寄存器
 * @param  reg  起始寄存器地址
 * @param  buf  数据接收缓冲区
 * @param  len  读取的字节数
 *
 * @note   发送格式：[寄存器地址 | 0x80][dummy*N] -> 接收 N 字节数据
 *         适用于读取连续的多字节数据（如 XYZ 三轴数据）
 */
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);  /* 发送起始寄存器地址 */

    /* 循环读取多个字节 */
    while (len != 0)
    {
        *buf = BMI088_read_write_byte(0x55);  /* 读取一个字节 */
        buf++;   /* 缓冲区指针递增 */
        len--;   /* 剩余字节数递减 */
    }
}

#elif defined(BMI088_USE_IIC)
/* IIC 通信方式下的底层函数（待实现） */
#endif
