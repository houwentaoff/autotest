/*
 * key.c
 *
 * ch452 driver
 *
 *  Created on: 2013.04.08
 *      Author: mzd
 *      explain:采用中断＋异步通知方式
 *              中断发生时，内核主动通知用户，用户再来读数据
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>

// #include <asm/mach-types.h>
// #include <plat/hardware.h>
// #include <mach/board-am335xevm.h>
// #include <plat/gpio.h>
// #include <plat/irqs.h>
#include <linux/input.h>  

#include <asm/uaccess.h>    /* copy_*_user */
#include "key.h"
#include "../scale-A8-drivers.h"

MODULE_AUTHOR("CDJZ-TECH, SPSY");
MODULE_LICENSE("GPL");

//#define KEY_DEBUG

#ifdef KEY_DEBUG
#define __D(level, fmt, args...)    printk(level "key: Debug " fmt, ##args)
#define __E(level, fmt, args...)    printk(level "key: Error " fmt, ##args)
#else
#define __D(level, fmt, args...)
#define __E(level, fmt, args...)
#endif

#define KEY_MAJOR 0   /* dynamic major by default - 动态分配设备号*/
#define DEVICENAME "scale-key"


/************************CH452 pin operation*******************************************************/
//SCL置值清值
#define CH452_SCL_SET           gpio_set_value(SCALE_KEY_SCL, GPIO_ST_HIGH)
#define CH452_SCL_CLR           gpio_set_value(SCALE_KEY_SCL, GPIO_ST_LOW)
#define CH452_SCL_D_OUT         {}

//SDA置值清值
#define CH452_SDA_SET           gpio_set_value(SCALE_KEY_SDA, GPIO_ST_HIGH)
#define CH452_SDA_CLR           gpio_set_value(SCALE_KEY_SDA, GPIO_ST_LOW)
#define CH452_SDA_IN            gpio_get_value(SCALE_KEY_SDA)
//SDA为输出/输入方向切换
#define CH452_SDA_D_OUT         {gpio_direction_output(SCALE_KEY_SDA, GPIO_ST_HIGH);}
#define CH452_SDA_D_IN          {gpio_direction_input(SCALE_KEY_SDA);}

//中断引脚
#define DISABLE_KEY_INTERRUPT   //disable_irq_nosync(gpio_to_irq(SCALE_KEY_INT))
#define ENABLE_KEY_INTERRUPT    //enable_irq(gpio_to_irq(SCALE_KEY_INT))
#define CLEAR_KEY_INTER_FLAG
//状态引脚
#define GET_CH452_ST            gpio_get_value(SCALE_KEY_ST)


KEY_CODE    KeyCode;
unsigned char  Key_Type;
int key_major;
int irq_nr;
static struct class *key_class;
//定义结构体fasync_struct
struct fasync_struct *key_async;

struct file_operations key_fops =
{

    .owner = THIS_MODULE,
    .open = key_open,
    .read = key_read,
    .write = key_write,
    .unlocked_ioctl = key_ioctl,
    .fasync = key_fasync,
    .release = key_release
};
static struct input_dev *button_dev; 


/*****************************************************************************
函数名称: Ch452_I2c_Start()
函数功能: 启动I2C
入口参数: 无
出口参数: 无
全局变量: 无
备注:
 *******************************************************************************/
void Ch452_I2c_Start(void) // 操作起始
{
    DISABLE_KEY_INTERRUPT;  //禁止键盘中断,防止开始时被CH452中断而进入中断服务程序中的START
    CH452_SDA_D_OUT;        // 设置SDA为输出方向
    CH452_SDA_SET;          //发送起始条件的数据信号
    CH452_SCL_SET;
    CH452_SCL_D_OUT;
    udelay(2);
    CH452_SDA_CLR;          //发送起始信号
    udelay(2);
    CH452_SCL_CLR;          //钳住I2C总线，准备发送或接收数据
    udelay(2);
}

void Ch452_I2c_Stop(void)
{
    CH452_SDA_D_OUT;
    CH452_SDA_CLR;
    udelay(2);
    CH452_SCL_SET;
    udelay(2);
    CH452_SDA_SET;
    udelay(2);
    //CH452_SCL_CLR;
    udelay(2);
    ENABLE_KEY_INTERRUPT;
}

void Ch452_I2c_WrByte(unsigned char u8Data)
{
    unsigned char i;

    CH452_SDA_D_OUT;
    for (i = 0; i != 8; i++)
    {
        if (u8Data & 0x80)
        {
            CH452_SDA_SET;
        }
        else
        {
            CH452_SDA_CLR;
        }
        udelay(1);
        CH452_SCL_SET;

        u8Data <<= 1;
        udelay(2);

        CH452_SCL_CLR;
        udelay(1);
    }

    CH452_SDA_SET;
    udelay(1);

    CH452_SCL_SET;
    udelay(2);

    CH452_SCL_CLR;
    udelay(1);
}

unsigned char Ch452_I2c_RdByte(void)
{
    unsigned char u8Data, i;

    CH452_SDA_D_OUT;
    CH452_SDA_SET;

    udelay(2);
    CH452_SDA_D_IN;

    u8Data = 0;
    for (i = 0; i != 8; i++)
    {
        CH452_SCL_SET;
        udelay(2);

        u8Data <<= 1;
        if (CH452_SDA_IN)
        {
            u8Data++;
        }

        CH452_SCL_CLR;
        udelay(2);
    }

    CH452_SDA_D_OUT;
    CH452_SDA_SET;
    udelay(1);

    CH452_SCL_SET;
    udelay(2);

    CH452_SCL_CLR;
    udelay(1);

    return u8Data;
}

void Ch452_Write(unsigned char addr, unsigned short u16Cmd)
{
    Ch452_I2c_Start();
    Ch452_I2c_WrByte((((u16Cmd >> 7) & 0xFF) & CH452_I2C_MASK) | addr);
    Ch452_I2c_WrByte((unsigned char)u16Cmd);
    Ch452_I2c_Stop();
}

unsigned char Ch452_Read(unsigned char addr)
{
    unsigned char u8KeyCode;

    Ch452_I2c_Start();
    Ch452_I2c_WrByte((((CH452_GET_KEY >> 7) & 0xFF) & CH452_I2C_MASK) | 0x01 | addr);
    u8KeyCode = Ch452_I2c_RdByte();
    Ch452_I2c_Stop();
    return u8KeyCode;
}

unsigned char Ch452_Get_Ver(unsigned char cs)
{
    unsigned char  addr, ch452_ver;

    addr = (cs == CH452_0 ? CH452_I2C_ADDR0 : CH452_I2C_ADDR1);
    Ch452_I2c_Start();
    Ch452_I2c_WrByte((((CH452_VER >> 7) & 0xFF) & CH452_I2C_MASK) | 0x01 | addr);
    ch452_ver = Ch452_I2c_RdByte();
    Ch452_I2c_Stop();

    return ch452_ver;
}

int Ch452_Init(void)
{
    unsigned char i, chip0_ver = 0xFF, chip1_ver = 0xFF;

    Ch452_Write(CH452_I2C_ADDR0, CH452_SYSON2);
    Ch452_Write(CH452_I2C_ADDR1, CH452_SYSON2);

    for (i = 0; i < 5; i++)
    {
        chip0_ver = Ch452_Get_Ver(CH452_0);
        chip1_ver = Ch452_Get_Ver(CH452_1);

        if ((chip0_ver == 0xFF) && (chip1_ver == 0xFF))
        {
            return KEY_INIT_ERROR;
        }
        else if ((chip0_ver != 0xFF) && (chip1_ver != 0xFF))
        {
            Key_Type = KEY_TYPE_DOUBLE;
            break;
        }
        else
        {
            Key_Type = KEY_TYPE_SINGLE;
            break;
        }
        //__D(KERN_NOTICE,"Key_Type = %d\n",Key_Type);
    }
    return KEY_INIT_OK;
}


static ssize_t key_open(struct inode *inode, struct file *filp)
{
    return 0;
}
static int button_open(struct input_dev *dev) 
{
    return 0;
}
static void button_close(struct input_dev *dev)
{
    return ;
}
static int button_flush(struct input_dev *dev, struct file *file)
{
    printk("[event]:==>%s\n", __func__);
    printk("[event]:<==%s\n", __func__);
    return 0;
}
static void button2_event(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
    printk("[event]:==>%s type[0x%x] code[0x%x] value[0x%x]\n",
            __func__, type, code, value);
    printk("[event]:<==%s\n", __func__);
}

static int button_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
    printk("[event]:==>%s type[0x%x] code[0x%x] value[0x%x]\n",
            __func__, type, code, value);
    if (EV_SYN == type && SYN_CONFIG == code)
    {
        KeyCode = value&0xff;
        kill_fasync(&key_async, SIGIO, POLL_IN);
    }
    printk("[event]:<==%s\n", __func__);
    return 0;
}

static ssize_t key_read(struct file *file , char *buf, size_t count, loff_t *f_ops)
{
    int ret;

    ret = copy_to_user(buf, &KeyCode, 1);
    if (ret == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static ssize_t key_write(struct file *file , const char *buf, size_t count, loff_t *f_ops)
{
    return 0;
}

static long key_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    unsigned char  ver;
    int err = 1;
    /*
     * extract the type and number bitfields, and don't decode
     * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
     */
    if (_IOC_TYPE(cmd) != SCALE_MAGIC)
        return -ENOTTY;
    if (_IOC_NR(cmd) > SCALE_IOC_MAXNR)
        return -ENOTTY;

    /*
     * the direction is a bitmask, and VERIFY_WRITE catches R/W
     * transfers. `Type' is user-oriented, while
     * access_ok is kernel-oriented, so the concept of "read" and
     * "write" is reversed
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
        err = access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        err = access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if (!err)
    {
        return -EFAULT;
    }

    switch (cmd)
    {
    case SCALE_KEY_IOC_INIT:
        Ch452_Init();
        break;
    case SCALE_KEY_IOC_ID:
        ver = Ch452_Get_Ver(CH452_0);
        put_user(ver, (int __user *)arg);
        break;
    default:
        return -1;
    }
    return 0;
}

// 实现key_fasync函数，调用函数fasync_helper将fd,filp和定义的结构体传给内核
static ssize_t key_fasync(int fd, struct file *filp, int on)
{
    return fasync_helper(fd, filp, on, &key_async);
}

static ssize_t key_release(struct inode *inode, struct file *filp)
{
    key_fasync(-1, filp, 0);    //文件关闭时将结构体从异步队列中删除
    return 0;
}

uint32_t jiffies_old = 0;
static irqreturn_t key_irq(int a, void * b)
{
    //disable_irq_nosync(gpio_to_irq(SCALE_KEY_INT));
    //printk("[key]: Key_Type[%d]\n", Key_Type);

    if (jiffies - jiffies_old < 10)
    {
        Ch452_Read(CH452_I2C_ADDR1);
        Ch452_Read(CH452_I2C_ADDR0);
        Ch452_Write(CH452_I2C_ADDR1,CH452_RESET);
        Ch452_Write(CH452_I2C_ADDR0,CH452_RESET);
        Ch452_Write(CH452_I2C_ADDR0, CH452_SYSON2);
        Ch452_Write(CH452_I2C_ADDR1, CH452_SYSON2);
        jiffies_old = jiffies;
        return  IRQ_RETVAL(IRQ_HANDLED);
    }
    if (Key_Type == KEY_TYPE_SINGLE)
    {
        KeyCode = Ch452_Read(CH452_I2C_ADDR1);
        KeyCode &= ~0x40;
        //__D(KERN_NOTICE,"KeyCode = %d\n",KeyCode);
    }
    else if (Key_Type == KEY_TYPE_DOUBLE)
    {
        if (GET_CH452_ST == GPIO_ST_LOW)
        {
            KeyCode = Ch452_Read(CH452_I2C_ADDR0);
            KeyCode &= ~0x40;
            KeyCode |= 0x80;
            //__D(KERN_NOTICE,"KeyCode = %d\n",KeyCode);
        }
        else
        {
            KeyCode = Ch452_Read(CH452_I2C_ADDR1);
            KeyCode &= ~0x40;
            //__D(KERN_NOTICE,"KeyCode = %d\n",KeyCode);
        }
        Ch452_Write(CH452_I2C_ADDR1,CH452_RESET);
	    Ch452_Write(CH452_I2C_ADDR0,CH452_RESET);
	    Ch452_Write(CH452_I2C_ADDR0, CH452_SYSON2);
	    Ch452_Write(CH452_I2C_ADDR1, CH452_SYSON2);
    }
    //input_event(button_dev, EV_MSC, MSC_SCAN, KeyCode);
    input_report_key(button_dev, KEY_NUMERIC_1/*KeyCode*/, 1); 
    input_sync(button_dev);
    printk("[key]: KeyCode[%d][0x%x]\n", KeyCode, KeyCode);
    //当设备可读时，调用函数kill_fasync发送信号SIGIO给内核。
    kill_fasync(&key_async, SIGIO, POLL_IN);
    jiffies_old = jiffies;
    
    //enable_irq(gpio_to_irq(SCALE_KEY_INT));    
    return  IRQ_RETVAL(IRQ_HANDLED);
}
static struct input_handle grab;
static struct input_handler handler;

static int key_ch452_init(void)
{
    int error = 0; 
    struct input_dev *input_dev;
    memset(&grab, 0, sizeof(grab));
    memset(&handler, 0, sizeof(struct input_handler));
    
    handler.event = button2_event;
    grab.handler = &handler;
    
    input_dev = input_allocate_device();
    if (!input_dev)
    {
        printk(KERN_ERR"Unable to allocate the input device!!/n");
        return -ENOMEM; 
    }
    button_dev = input_dev;
    set_bit(EV_KEY, button_dev->evbit);
    set_bit(EV_LED, button_dev->evbit);
    set_bit(EV_SND, button_dev->evbit);
    set_bit(EV_SYN, button_dev->evbit);
    set_bit(KEY_SPACE, button_dev->keybit);
    set_bit(KEY_NUMERIC_0, button_dev->keybit);
    set_bit(KEY_NUMERIC_1, button_dev->keybit);
    set_bit(KEY_NUMERIC_2, button_dev->keybit);
    set_bit(KEY_NUMERIC_3, button_dev->keybit);
    set_bit(KEY_NUMERIC_4, button_dev->keybit);
    button_dev->name = "buttons_tyc";
    button_dev->phys = "/dev/input00";
    button_dev->dev.init_name = "input_tyc"; 
    button_dev->id.bustype = BUS_I2C;
    button_dev->id.version = 0x0100;//1.0
    button_dev->open = button_open; 
    button_dev->close = button_close;
    button_dev->flush = button_flush;
    button_dev->event = button_event;//led or sound :operation to device   (EV_SYN+SYN_CONFIG) only?
    //button_dev->grab = &grab;//xxxxxxxx
    
    error = input_register_device(button_dev);  
    if (error)
    {
        printk(KERN_ERR "button.c: Failed to register device\n"); 
        input_free_device(button_dev);
        return error;
    }
    
    gpio_request(SCALE_KEY_SDA, "ch452 sda pin");
    gpio_direction_output(SCALE_KEY_SDA, GPIO_ST_HIGH);

    gpio_request(SCALE_KEY_SCL, "ch452 scl pin");
    gpio_direction_output(SCALE_KEY_SCL, GPIO_ST_HIGH);

    gpio_request(SCALE_KEY_ST, "ch452 st pin");
    gpio_direction_input(SCALE_KEY_ST);

    gpio_request(SCALE_KEY_INT, "ch452 int pin");
    gpio_direction_input(SCALE_KEY_INT);

    if (KEY_INIT_ERROR == Ch452_Init())
    {
        printk("[key]: init ch452 fail!!!\n");
    }

    key_major = register_chrdev(KEY_MAJOR, DEVICENAME, &key_fops);
    key_class = class_create(THIS_MODULE, DEVICENAME);
    device_create(key_class, NULL, MKDEV(key_major, 0), NULL, DEVICENAME);
    if (key_major < 0)
    {
        printk("module key ch452 register fail!\n");
        return -1;
    }
    else
    {
        printk("module key ch452 register success!\n");
    }
    irq_nr = gpio_to_irq(SCALE_KEY_INT);
    irq_set_irq_type(irq_nr, IRQF_TRIGGER_FALLING);

    request_irq(irq_nr, key_irq, 0, DEVICENAME, NULL);
    Ch452_Write(CH452_I2C_ADDR1,CH452_RESET);
    Ch452_Write(CH452_I2C_ADDR0,CH452_RESET);
    Ch452_Write(CH452_I2C_ADDR0, CH452_SYSON2);
    Ch452_Write(CH452_I2C_ADDR1, CH452_SYSON2);
    //enable_irq(irq_nr);
    return 0;
}

static void key_ch452_exit(void)
{
    input_unregister_device(button_dev); 
    free_irq(irq_nr, NULL);
    gpio_free(SCALE_KEY_SDA);
    gpio_free(SCALE_KEY_SCL);
    gpio_free(SCALE_KEY_ST);
    gpio_free(SCALE_KEY_INT);
    unregister_chrdev(key_major, DEVICENAME);
    device_destroy(key_class, MKDEV(key_major, 0));
    class_destroy(key_class);
}

module_init(key_ch452_init);
module_exit(key_ch452_exit);
