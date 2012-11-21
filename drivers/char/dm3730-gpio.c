#include <linux/miscdevice.h> 
#include <linux/delay.h> 
#include <linux/kernel.h> 
#include <linux/module.h> 
#include <linux/init.h> 
#include <linux/mm.h> 
#include <linux/fs.h> 
#include <linux/types.h> 
#include <linux/delay.h> 
#include <linux/moduleparam.h> 
#include <linux/slab.h> 
#include <linux/errno.h> 
#include <linux/ioctl.h> 
#include <linux/cdev.h> 
#include <linux/string.h> 
#include <linux/list.h> 
#include <linux/pci.h> 
#include <linux/gpio.h> 
#include <linux/version.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>

#include <linux/delay.h>

extern int omap_mux_init_gpio(int gpio, int val);

#define DEVICE_NAME "dm3730-gpio" //设备名(/dev/led) 

#define MOTOR_MAGIC 'g'
#define SET_GPIO42_DIR                  _IOW(MOTOR_MAGIC, 2,int)
#define SET_GPIO42_VALUE		_IOW(MOTOR_MAGIC, 3,int) 
#define GET_GPIO42_VALUE		_IOW(MOTOR_MAGIC, 4,int) 

// ioctl 函数的实现 
// 在应用用户层将通过 ioctl 函数向内核传递参数，以控制 LED的输出状态 
static int am1808_gpio_ioctl( 
 struct inode *inode,  
 struct file *file,   
 unsigned int cmd,  
 unsigned long arg) 
{ 
	#if 1
	printk("dm3730-gpio:cmd interger is %d,hex is 0x%08x\n",(int)cmd,(unsigned int)cmd);
	printk("dm3730-gpio:arg interger is %d,hex is 0x%08x\n",(int)arg,(unsigned int)arg);
	#endif
   	switch (cmd)
   	{	
		case SET_GPIO42_VALUE:   // GPIO7[13]
		if(arg == 0)
		{
			gpio_set_value(42,0); 
		}
		else
		{
			gpio_set_value(42,1); 
		}
		break;	
	
		case SET_GPIO42_DIR:  // GPIO7[15]
		if(arg == 0)   // 输出
		{
			//omap_mux_init_gpio(42, 0);
			 gpio_direction_output(42,1);
		}
		else  // 输入
		{
			//omap_mux_init_gpio(42, (1 << 8));
			 gpio_direction_input(42);
		}
		break;

		
		case GET_GPIO42_VALUE:  // GPIO8[9]
 		if(arg != 0)
		{
			if(gpio_get_value(42) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;

		default:
		printk("dm3730-gpio:ioctl cmd error\n");
		return -EINVAL; 
   	} 
   	return 0;       
} 
 
 
//  设备函数操作集，在此只有 ioctl函数，通常还有 read, write, open, close 等，因为本 LED驱动在下面已经
//  注册为 misc 设备，因此也可以不用 open/close  
static struct file_operations dev_fops = { 
 .owner = THIS_MODULE, 
 .ioctl = am1808_gpio_ioctl, 
}; 
  
//  把 LED驱动注册为 MISC 设备 
static struct miscdevice misc = { 
  //动态设备号
  .minor = MISC_DYNAMIC_MINOR,  
  .name = DEVICE_NAME, 
  .fops = &dev_fops, 
}; 
 
 
// 设备初始化 
static int __init dev_init(void) 
{ 
	unsigned long TmpRegVal;
	int status;
	int ret;

	ret = gpio_request(42, "sja1000");
	if(ret)
	{
		printk("request gpio 42 error\n");
	}
	else
	{
		printk("request gpio 42 ok\n");
	}
   	ret = misc_register(&misc); //注册设备 
   	printk (DEVICE_NAME"\tinitialized\n"); //打印初始化信息 
   	return ret; 
} 
 
static void __exit dev_exit(void) 
{ 
	gpio_free(42);
	misc_deregister(&misc); 
} 
 
// 模块初始化，仅当使用 insmod/podprobe 命令加载时有用，
// 如果设备不是通过模块方式加载，此处将不会被调用 
module_init(dev_init); 

// 卸载模块，当该设备通过模块方式加载后，
// 可以通过 rmmod 命令卸载，将调用此函数 
module_exit(dev_exit);

// 版权信息 
MODULE_LICENSE("GPL"); 
// 开发者信息 
MODULE_AUTHOR("Lierda EA nmy"); 
