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

// 以下是要给到app测试的
#define DEVICE_NAME "dm3730-gpio" //设备名(/dev/led) 
#define MOTOR_MAGIC 'g'
#if 0
#define GET_KEY10			_IOW(MOTOR_MAGIC, 1,int) 
#define GET_KEY9			_IOW(MOTOR_MAGIC, 2,int) 
#define GET_KEY8			_IOW(MOTOR_MAGIC, 3,int) 
#define GET_KEY7			_IOW(MOTOR_MAGIC, 4,int) 
#define GET_KEY6			_IOW(MOTOR_MAGIC, 5,int) 
#define GET_KEY5			_IOW(MOTOR_MAGIC, 6,int) 
#define GET_KEY4			_IOW(MOTOR_MAGIC, 7,int) 
#define GET_KEY3			_IOW(MOTOR_MAGIC, 8,int) 
#define GET_KEY2			_IOW(MOTOR_MAGIC, 9,int) 
#define GET_KEY1			_IOW(MOTOR_MAGIC, 10,int) 
#endif
#define GET_1V8_LED_INT			_IOW(MOTOR_MAGIC, 11,int) 
#define SET_BL_EN			_IOW(MOTOR_MAGIC, 12,int) 
#define SET_LED_EN			_IOW(MOTOR_MAGIC, 13,int) 
#define SET_BUZ_EN			_IOW(MOTOR_MAGIC, 14,int) 
#define SET_LED1			_IOW(MOTOR_MAGIC, 15,int) 
#define SET_LED2			_IOW(MOTOR_MAGIC, 16,int) 
#define SET_RELAY1			_IOW(MOTOR_MAGIC, 17,int) 
#define SET_RELAY2			_IOW(MOTOR_MAGIC, 18,int) 
#define SET_RELAY3			_IOW(MOTOR_MAGIC, 19,int) 
#define SET_RELAY4			_IOW(MOTOR_MAGIC, 20,int) 
#define GET_IOIN3			_IOW(MOTOR_MAGIC, 21,int) 
#define GET_IOIN2			_IOW(MOTOR_MAGIC, 22,int) 
#define GET_IOIN1			_IOW(MOTOR_MAGIC, 23,int) 

// gpio setting
#if 0
#define GPIO_KEY10   		24
#define GPIO_KEY9    		43
#define GPIO_KEY8    		26
#define GPIO_KEY7    		27
#define GPIO_KEY6    		28
#define GPIO_KEY5    		29
#define GPIO_KEY4    		136
#define GPIO_KEY3    		137
#define GPIO_KEY2    		138
#define GPIO_KEY1    		139
#endif
#define GPIO_1V8_LED_INT   	173
#define GPIO_BL_EN   		141
#define GPIO_LED_EN  		140
#define GPIO_BUZ_EN  		163
#define GPIO_LED1    		160
#define GPIO_LED2    		161
#define GPIO_RELAY1  		156
#define GPIO_RELAY2  		157
#define GPIO_RELAY3   		158
#define GPIO_RELAY4  		149
#define GPIO_IOIN3   		98
#define GPIO_IOIN2   		164
#define GPIO_IOIN1   		170

unsigned char gpio_input_array[] = {
	/*GPIO_KEY10,
	GPIO_KEY9,
	GPIO_KEY8,
	GPIO_KEY7,
	GPIO_KEY6,
	GPIO_KEY5,
	GPIO_KEY4,
	GPIO_KEY3,
	GPIO_KEY2,
	GPIO_KEY1,*/
	GPIO_1V8_LED_INT,
	GPIO_IOIN3,
	GPIO_IOIN2,
	GPIO_IOIN1
};


unsigned char gpio_output_array[] = {
	GPIO_BL_EN,
	GPIO_LED_EN,
	GPIO_BUZ_EN,
	GPIO_LED1,
	GPIO_LED2,
	GPIO_RELAY1,
	GPIO_RELAY2,
	GPIO_RELAY3,
	GPIO_RELAY4
};

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
		case SET_BL_EN:   
		if(arg == 0)
		{
			gpio_set_value(GPIO_BL_EN,0); 
		}
		else
		{
			gpio_set_value(GPIO_BL_EN,1); 
		}
		break;
		case SET_LED_EN:   
		if(arg == 0)
		{
			gpio_set_value(GPIO_LED_EN,0); 
		}
		else
		{
			gpio_set_value(GPIO_LED_EN,1); 
		}
		break;	
		case SET_BUZ_EN:   
		if(arg == 0)
		{
			gpio_set_value(GPIO_BUZ_EN,0); 
		}
		else
		{
			gpio_set_value(GPIO_BUZ_EN,1); 
		}
		break;	
		case SET_LED1:   
		if(arg == 0)
		{
			gpio_set_value(GPIO_LED1,0); 
		}
		else
		{
			gpio_set_value(GPIO_LED1,1); 
		}
		break;	
		case SET_LED2:   
		if(arg == 0)
		{
			gpio_set_value(GPIO_LED2,0); 
		}
		else
		{
			gpio_set_value(GPIO_LED2,1); 
		}
		break;	
		case SET_RELAY1:   
		if(arg == 0)
		{
			gpio_set_value(GPIO_RELAY1,0); 
		}
		else
		{
			gpio_set_value(GPIO_RELAY1,1); 
		}
		break;	
		case SET_RELAY2:   
		if(arg == 0)
		{
			gpio_set_value(GPIO_RELAY2,0); 
		}
		else
		{
			gpio_set_value(GPIO_RELAY2,1); 
		}
		break;	
		case SET_RELAY3:   
		if(arg == 0)
		{
			gpio_set_value(GPIO_RELAY3,0); 
		}
		else
		{
			gpio_set_value(GPIO_RELAY3,1); 
		}
		break;	
		case SET_RELAY4:   
		if(arg == 0)
		{
			gpio_set_value(GPIO_RELAY4,0); 
		}
		else
		{
			gpio_set_value(GPIO_RELAY4,1); 
		}
		break;	
		
	
		
		#if 0
		case GET_KEY10:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_KEY10) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		case GET_KEY9:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_KEY9) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		case GET_KEY8:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_KEY8) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		case GET_KEY7:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_KEY7) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		case GET_KEY6:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_KEY6) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		case GET_KEY5:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_KEY5) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		case GET_KEY4:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_KEY4) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		case GET_KEY3:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_KEY3) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		case GET_KEY2:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_KEY2) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		case GET_KEY1:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_KEY1) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		#endif
		case GET_1V8_LED_INT:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_1V8_LED_INT) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		case GET_IOIN3:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_IOIN3) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		case GET_IOIN2:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_IOIN2) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		case GET_IOIN1:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_IOIN1) == 0)			
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
	int gpio;
	int i;

	for(i = 0;i < sizeof(gpio_input_array);i++)
	{
		gpio = 	gpio_input_array[i];
		ret = gpio_request(gpio, "GPIO");
		if(ret)
			printk("request input gpio %d error\n",gpio);
		else
			printk("request input gpio %d  ok\n",gpio);
		gpio_direction_input(gpio);
	}

	for(i = 0;i < sizeof(gpio_output_array);i++)
	{
		gpio = 	gpio_output_array[i];
		ret = gpio_request(gpio, "GPIO");
		if(ret)
			printk("request output gpio %d error\n",gpio);
		else
			printk("request output gpio %d  ok\n",gpio);
		gpio_direction_output(gpio,1);
	}

   	ret = misc_register(&misc); //注册设备 
   	printk (DEVICE_NAME"\tinitialized\n"); //打印初始化信息 
   	return ret; 
} 
 
static void __exit dev_exit(void) 
{ 
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
