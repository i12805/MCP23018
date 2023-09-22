#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/gpio.h>                 // Required for the GPIO functions
#include <linux/interrupt.h>            // Required for the IRQ code
#include <linux/kobject.h>    // Using kobjects for the sysfs bindings
#include <linux/kthread.h>
#include <linux/delay.h>

#include "MCP23018.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("IMB");
MODULE_DESCRIPTION("MCP23018 I2C IO expander driver");
MODULE_VERSION("0.2");

#define IRQ_PIN_COUNT (2)
#define TASK_PERIOD (100)    /* in [ms] */
#define BUTTON_DEBOUNCE (50) /* in [ms] */ 

static unsigned int nRST_pin = 6u; 	///< Default
module_param(nRST_pin, uint, S_IRUGO); ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(nRST_pin, " MCP23018 reset pin (default=27)");     ///< parameter description

static unsigned int irqA_pin = 5u; 	///< Default
module_param(irqA_pin, uint, S_IRUGO); ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(irqA_pin, " MCP23018 interrupt A pin (default=4)");     ///< parameter description

static unsigned int irqB_pin = 13u; 	///< Default
module_param(irqB_pin, uint, S_IRUGO); ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(irqB_pin, " MCP23018 interrupt B pin (default=22)");     ///< parameter description


static struct task_struct *rd_butt_matrix_task; /* periodic task for reading button matrix input pins */
static int read_button_matrix(void *arg); /* task function */

static unsigned int gpioIRQ[IRQ_PIN_COUNT] = {5, 13};
static unsigned int irqNumber[IRQ_PIN_COUNT] = {0}; ///< Used to share the IRQ number within this file
static unsigned int irqCount[IRQ_PIN_COUNT] = {0};

/// Function prototype for the custom IRQ handler function -- see below for the implementation
static irq_handler_t MCP23018_IRQA_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);
static irq_handler_t MCP23018_IRQB_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

static irq_handler_t MCP23018_IRQA_threaded_fn(unsigned int irq, void *dev_id, struct pt_regs *regs);
static irq_handler_t MCP23018_IRQB_threaded_fn(unsigned int irq, void *dev_id, struct pt_regs *regs);

/* function pointer array of irq handlers */
static irq_handler_t (*MCP23018_irq_handler[IRQ_PIN_COUNT])(unsigned int, void *, struct pt_regs *) = {MCP23018_IRQA_irq_handler, MCP23018_IRQB_irq_handler};
static irq_handler_t (*MCP23018_threaded_fn[IRQ_PIN_COUNT])(unsigned int, void *, struct pt_regs *) = {MCP23018_IRQA_threaded_fn, MCP23018_IRQB_threaded_fn};

static unsigned int u32INTCAP = 0x12345678u; // hold interrupt capture register of the MCP23018

/** @brief Displays if the LED is on or off */
static ssize_t u32INTCAP_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
	int len = 0;
	len =  sprintf(buf, "%d\n", (u32INTCAP & 0xffffffffu));
	return len;
}

/**  The __ATTR_RO macro defines a read-only attribute. There is no need to identify that the
 *  function is called _show, but it must be present. __ATTR_WO can be  used for a write-only
 *  attribute but only in Linux 3.11.x on.
 */
static struct kobj_attribute intcap_attr = __ATTR_RO(u32INTCAP);     ///< the irq capture reg. kobject attr

/**  The _attrs[] is an array of attributes that is used to create the attribute group below.
 *  The attr property of the kobj_attribute is used to extract the attribute struct
 */
static struct attribute *MCP23018_attrs[] = {
      &intcap_attr.attr,
      NULL,
};

/**  The attribute group uses the attribute array and a name, which is exposed on sysfs */
static struct attribute_group attr_group = {
      .name  = "MCP23018_data",
      .attrs = MCP23018_attrs
};
static struct kobject *MCP23018_kobj;

/* init i2c driver */
#define I2C_BUS_AVAILABLE (1)
#define DEVICE_NAME "MCP23018_I2C_DRIVER"
#define SLAVE_ADDR (0x22)

static struct i2c_adapter *i2c_adapter = NULL;
static struct i2c_client *i2c_client = NULL;

static const struct i2c_device_id MCP23018_id[] = {
	{ DEVICE_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, MCP23018_id);

static int I2C_write(unsigned char *buf, unsigned int len) {
	return i2c_master_send(i2c_client, buf, len);
}

static int I2C_read(unsigned char *buf, unsigned int len) {
	return i2c_master_recv(i2c_client, buf, len);
}

#define DATA_PAGE_SIZE (22) /* max data, that could be read from MCP32018 */
#define I2C_MSG_SIZE (2)
static int i2c_get_reg_data(int reg_addr, size_t len, unsigned char *RD_buff) {
	struct i2c_msg msg[I2C_MSG_SIZE];
	unsigned char cmd[1] = {0x00}; /* actually MCP32018 register address */
	int sent;
	unsigned short datalen;

	if (!RD_buff) return -1;

	if ((reg_addr + len) > DATA_PAGE_SIZE) datalen = (unsigned short)DATA_PAGE_SIZE;
	else if (len == 0) return 0;
	else datalen = (unsigned short)len;

	cmd[0] = (unsigned char)reg_addr;

	msg[0].addr = SLAVE_ADDR;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = cmd;

	msg[1].addr = SLAVE_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].len = datalen; /* data in bytes */
	msg[1].buf = RD_buff;

	sent = i2c_transfer(i2c_client->adapter, msg, I2C_MSG_SIZE);

	return (int)datalen;
}

static int i2c_set_reg_data(int reg_addr, size_t len, unsigned char *WR_buff) {
	struct i2c_msg msg[I2C_MSG_SIZE];
	unsigned char cmd[1] = {0x00}; /* actually MCP32018 register address */
	int sent;
	unsigned short datalen;

	if (!WR_buff) return -1;

	if ((reg_addr + len) > DATA_PAGE_SIZE) datalen = (unsigned short)DATA_PAGE_SIZE;
	else if (len == 0) return 0;
	else datalen = (unsigned short)len;

	cmd[0] = (unsigned char)reg_addr;

	msg[0].addr = SLAVE_ADDR;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = cmd;

	msg[1].addr = SLAVE_ADDR;
	msg[1].flags = 0; /* WRite */
	msg[1].len = datalen; /* data in bytes */
	msg[1].buf = WR_buff;

	sent = i2c_transfer(i2c_client->adapter, msg, I2C_MSG_SIZE);

	return (int)sent;
}

static int MCP23018_probe(struct i2c_client *client) {
	unsigned char RDbuff[DATA_PAGE_SIZE] = {0};
	unsigned int bytes_read = 0u;
	unsigned int i = 0u;
	unsigned char WRdata[2] = {0xffu, 0x00u};

	printk(KERN_INFO "I2C client: Probing ...");
	//bytes_read = I2C_read(buffer, 22u);
	bytes_read = i2c_get_reg_data(0x00, 22u, RDbuff); /* make sure data lenght is aligned with the buffer's size!!! */
	if (bytes_read <= 0) {
		printk(KERN_ALERT "I2C client Error");
	} else {
		printk(KERN_INFO "I2C client: read %d bytes:", bytes_read);
		for(i = 0; i < bytes_read; i++) {
			printk(KERN_INFO "\t0x%02x: 0x%02x", i, RDbuff[i]);
		}
	}
	
	/* config irq  - not a single one!
	 * for the button matrix we do not need interrupt, 'cos
	 * the irq will fire at every column iteration. */

	/* config port B as output */
	bytes_read = i2c_set_reg_data((int)IODIRB, 1, &WRdata[1]);
	printk(KERN_INFO "I2C client: cfg IODIRB (0x%02x) = 0x%02x; bytes sent: %d;", (int)IODIRB, WRdata[1], bytes_read);

	/* set all pull-ups of port B */
	bytes_read = i2c_set_reg_data((int)GPPUB, 1, &WRdata[0]);
	printk(KERN_INFO "I2C client: cfg GPPUB (0x%02x) = 0x%02x; bytes sent: %d;", (int)GPPUB, WRdata[0], bytes_read);

	/* set all output pins of port B to high */
	bytes_read = i2c_set_reg_data((int)GPIOB, 1, &WRdata[0]);
	printk(KERN_INFO "I2C client: cfg GPIOB (0x%02x) = 0x%02x; bytes sent: %d;", (int)GPIOB, WRdata[0], bytes_read);

	/* set input pins inverted */
	bytes_read = i2c_set_reg_data((int)IPOLA, 1, &WRdata[0]);
	printk(KERN_INFO "I2C client: cfg IPOLA (0x%02x) = 0x%02x; bytes sent: %d;", (int)IPOLA, WRdata[0], bytes_read);

	/* read back all registers */
	if (i2c_get_reg_data(0x00, 22, RDbuff) < 0) {
		printk(KERN_INFO "Error reading regs.");
	} else {
		printk(KERN_INFO "MCP23018: Read back registers:");
		for (i = 0; i < 22; i++) printk(KERN_INFO "\t0x%02x = 0x%02x;", i, RDbuff[i]);
	}

	return 0;
}

static int MCP23018_remove(struct i2c_client *client) {

	printk(KERN_INFO "I2C client: Remove.");
	return 0;
}

static struct i2c_driver MCP23018_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		},
	.probe_new      = MCP23018_probe,
	.remove         = MCP23018_remove,
	.id_table       = MCP23018_id,
};

static struct i2c_board_info i2c_board_info = {
	I2C_BOARD_INFO(DEVICE_NAME, SLAVE_ADDR)
};
	 
static int i2c_driver_init(void) {
	int ret = -1;
	
	i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);
	
	if (i2c_adapter) {
		i2c_client = i2c_new_client_device(i2c_adapter, &i2c_board_info);
		
		if (!i2c_client) goto ERR;
		
		i2c_add_driver(&MCP23018_driver);
		i2c_put_adapter(i2c_adapter);
		
		ret = 0;
	}
ERR:
	return ret;
}

static void i2c_driver_exit(void) {
	
	i2c_unregister_device(i2c_client);
	i2c_del_driver(&MCP23018_driver);
	
	return;
}

/* END of i2c driver */

/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point. In this example this
 *  function sets up the GPIOs and the IRQ
 *  @return returns 0 if successful
 */
static int __init MCP23018_init(void){
	int result = 0;
	int aresult[IRQ_PIN_COUNT] = {0};
	int ret = 0;
	int i = 0;
	printk(KERN_INFO "MCP23018: Initializing the MCP23018 LKM.\n");
	printk(KERN_INFO "MCP23018: PAGE_SIZE = %ld", PAGE_SIZE);

	/* ***** init MCP23018 reset pin ***** */
	if (!gpio_is_valid(nRST_pin)) {
		printk(KERN_ALERT "MCP23018: nRST pin (%d) is not valid.\n", nRST_pin);
		ret = -ENODEV;
		goto OUT;
	}
	if (gpio_request(nRST_pin, "nRST_pin") < 0) {
		printk(KERN_ALERT "MCP23018: nRST pin (%d) cannot be requested.\n", nRST_pin);
		ret = -ENODEV;
		goto OUT;
	}
	gpio_direction_output(nRST_pin, 1);

	/* *** init i2c driver *** */
	if(i2c_driver_init()) {
		ret = -1;
		goto CLEAN_GPIO;
	}

	/* *** init interrupts *** */
	gpioIRQ[0] = irqA_pin;
	gpioIRQ[1] = irqB_pin;

	for(i = 0; i < IRQ_PIN_COUNT; i++) {
		// Is the GPIO a valid GPIO number
		if (!gpio_is_valid(gpioIRQ[i])){
			printk(KERN_ALERT "MCP23018: invalid gpioIRQx pin (%d). Index %d.\n", gpioIRQ[i], i);
			ret = -ENODEV;
			goto CLEAN_I2C;
		}

		gpio_request(gpioIRQ[i], "sysfs"); // Set up the gpioButton
		gpio_direction_input(gpioIRQ[i]);  // Set the button GPIO to be an input
		gpio_set_debounce(gpioIRQ[i], BUTTON_DEBOUNCE);  // Debounce the button with a delay
		gpio_export(gpioIRQ[i], false);  // Causes gpio to appear in /sys/class/gpio
					 	 // the bool argument prevents the direction from being changed
		// Perform a quick test to see that the IRQ is working as expected on LKM load
		printk(KERN_INFO "MCP23018: The IRQ %d state is currently: %d.\n", i, gpio_get_value(gpioIRQ[i]));

		// GPIO numbers and IRQ numbers are not the same! This function performs the mapping for us
		irqNumber[i] = gpio_to_irq(gpioIRQ[i]);
		printk(KERN_INFO "MCP23018: The IRQ pin %d is mapped to IRQ: %d.\n", gpioIRQ[i], irqNumber[i]);

		/* check irq handler */
		if (MCP23018_irq_handler[i]) {
#if 0
			aresult[i] = request_irq(irqNumber[i], (irq_handler_t)(MCP23018_irq_handler[i]), IRQF_TRIGGER_FALLING, /* default INT setting on MCP23018 */ "MCP23018_IRQ", /* Used in /proc/interrupts to identify the owner */ NULL /* The *dev_id for shared interrupt lines, NULL is okay */);
#endif
			aresult[i] = request_threaded_irq(irqNumber[i], (irq_handler_t)(MCP23018_irq_handler[i]), (irq_handler_t)(MCP23018_threaded_fn[i]), /* irq flags */ IRQF_TRIGGER_FALLING, /* default INT setting on MCP23018 */ "MCP23018_IRQ", /* Used in /proc/interrupts to identify the owner */ NULL /* The *dev_id for shared interrupt lines, NULL is okay */);

			printk(KERN_INFO "MCP23018: The interrupt request %d result is: %d. ", i, aresult[i]);
			if(aresult[i] != 0) {
				printk(KERN_ALERT "ERR.\n");
				ret = aresult[i];
				goto CLEAN_IRQ;
			} else {
				printk(KERN_INFO "OK.\n");
			}
		}
	} /* end for loop - init irqs */
	/* at this point index i = 2; we need to adjust it to the IRQ index number 
	 * in order to perform correct IRQ cleaning in case of error in init further down
	 * wneh goto CLEAN_IRQ is called */
	i = 1;

	/* create sysfs representation object */
	// create the kobject sysfs entry at /sys -- probably not an ideal location!
	MCP23018_kobj = kobject_create_and_add("MCP23018", kernel_kobj->parent); // kernel_kobj points to /sys/kernel
	if(!MCP23018_kobj){
		printk(KERN_ALERT "MCP23018: failed to create kobject mapping.\n");
		ret = -ENOMEM;
		goto CLEAN_IRQ;
	}
	// add the attributes to /sys/MCP23018 -- for example, /sys/MCP32018/u32INTCAP
	result = sysfs_create_group(MCP23018_kobj, &attr_group);
	if(result) {
		printk(KERN_ALERT "MCP23018: failed to create sysfs group.\n");
		kobject_put(MCP23018_kobj); // clean up -- remove the kobject sysfs entry
		ret = result;
		goto CLEAN_IRQ;
	}
	
	/* add kthread for controlling output port B with MCP23018-22 */
	rd_butt_matrix_task = kthread_run(read_button_matrix, NULL, "MCP32018_RD_thrd");
	if (IS_ERR(rd_butt_matrix_task)) {
		printk(KERN_ALERT "MCP32018 RD butt.mtrx kthread_run failed\n");
		ret = -ENOMEM; /* -ENOMEM is returned by kthread_run in case of error */
		goto CLEAN_KOBJ;
	}
	else
	{
		goto OUT;
	}

CLEAN_KOBJ:
	printk(KERN_INFO "Release MCP32018 kobject.");
	kobject_put(MCP23018_kobj); // clean up -- remove the kobject sysfs entry

CLEAN_IRQ:
	for(; i >= 0; i--) {
		printk(KERN_INFO "Release MCP32018 irq %d resources.", i);
		free_irq(irqNumber[i], NULL); // Free the IRQ number, no *dev_id required in this case
		gpio_unexport(gpioIRQ[i]);    // Unexport the Button GPIO
		gpio_free(gpioIRQ[i]);        // Free the irq GPIOs
	}
CLEAN_I2C:
	printk(KERN_INFO "Release MCO32018 I2C driver.");
	i2c_driver_exit();
CLEAN_GPIO:
	printk(KERN_INFO "Release MCP32018 nRST pin (%d).", nRST_pin);
	gpio_free(nRST_pin);        // Free the MCP23018 reset GPIO
OUT:
	return ret;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required. Used to release the
 *  GPIOs and display cleanup messages.
 */
static void __exit MCP23018_exit(void){
	int i = 0;
	kobject_put(MCP23018_kobj); // clean up -- remove the kobject sysfs entry
	for(i = 0; i < IRQ_PIN_COUNT; i++) {
		printk(KERN_INFO "MCP23018: The IRQ %d state is currently: %d.\n", i, gpio_get_value(gpioIRQ[i]));
		printk(KERN_INFO "MCP23018: The IRQ %d was activated %d times.\n", i, irqCount[i]);
		free_irq(irqNumber[i], NULL); // Free the IRQ number, no *dev_id required in this case
		gpio_unexport(gpioIRQ[i]);    // Unexport the Button GPIO
		gpio_free(gpioIRQ[i]);        // Free the irq GPIOs
	}
		
	i2c_driver_exit();

	gpio_set_value(nRST_pin, 0u); /* put MCP23018 in reset */
	gpio_free(nRST_pin);          /* Free the MCP23018 reset GPIO */
	kthread_stop(rd_butt_matrix_task); /* stop pin matrix reading kthread */

	printk(KERN_INFO "MCP23018: Goodbye from the LKM!\n");
}


/** @brief The GPIO IRQ Handler function
 *  This function is a custom interrupt handler that is attached to the GPIO above. The same interrupt
 *  handler cannot be invoked concurrently as the interrupt line is masked out until the function is complete.
 *  This function is static as it should not be invoked directly from outside of this file.
 *  @param irq    the IRQ number that is associated with the GPIO -- useful for logging.
 *  @param dev_id the *dev_id that is provided -- can be used to identify which device caused the interrupt
 *  Not used in this example as NULL is passed.
 *  @param regs   h/w specific register values -- only really ever used for debugging.
 *  return returns IRQ_HANDLED if successful -- should return IRQ_NONE otherwise.
 */
static irq_handler_t MCP23018_IRQA_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
	irqCount[0]++;
	printk(KERN_INFO "MCP23018: Interrupt IRQA (%d) count = %d.", irq, irqCount[0]);

	return (irq_handler_t) IRQ_WAKE_THREAD; /* restore irqs and shedule threaded function, that can wait */
}

static irq_handler_t MCP23018_IRQB_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
	irqCount[1]++;
	printk(KERN_INFO "MCP23018: Interrupt IRQB (%d) count = %d.", irq, irqCount[1]);

	return (irq_handler_t) IRQ_WAKE_THREAD;
}

static irq_handler_t MCP23018_IRQA_threaded_fn(unsigned int irq, void *dev_id, struct pt_regs *regs) {
	unsigned char buffer[1] = {0};
	unsigned int bytes_read = 0u;
	bytes_read = i2c_get_reg_data(INTCAPA, 1, buffer);
	if (bytes_read > 0) {
		u32INTCAP &= ~(0xFFu);
		u32INTCAP |= (unsigned int)(buffer[0] & 0xFFu);
		printk(KERN_INFO "INTCAPA = 0x%02x.", buffer[0]);
	} else {
		printk(KERN_INFO "kur");
	}
	return (irq_handler_t) IRQ_HANDLED;
}

static irq_handler_t MCP23018_IRQB_threaded_fn(unsigned int irq, void *dev_id, struct pt_regs *regs) {
	unsigned char buffer[1] = {0};
	unsigned int bytes_read = 0u;

	bytes_read = i2c_get_reg_data(INTCAPB, 1, buffer);
	if (bytes_read > 0) {
		u32INTCAP &= ~(0xFF00u);
		u32INTCAP |= (((unsigned int)buffer[0] << 8u) & 0xFF00u);
		printk(KERN_INFO "INTCAPB = 0x%02x.", buffer[0]);
	} else {
		printk(KERN_INFO "kur");
	}
	return (irq_handler_t) IRQ_HANDLED;
}

static int read_button_matrix(void *arg) {
	static unsigned char u8PortB[1] = {0x00u};
	unsigned char u8PortA[1] = {0x00u};
	static int n = 0;
	int status = 0;

	allow_signal(SIGKILL);
	
	while (!kthread_should_stop()) {
		set_current_state(TASK_RUNNING);
		
		/* iterate an output pins */
		u8PortB[0] = (unsigned char)(~(1 << n));
		status = i2c_set_reg_data((int)GPIOB, 1, u8PortB);
		// we  need ~1us for port A pin to change
		udelay(1);
		/* read port A and tranlsate it to output data */
		status = i2c_get_reg_data((int)GPIOA, 1, u8PortA);
		if (status > 0) {
			u32INTCAP &= ~(((unsigned int)0xFFu << (n*8)));
			u32INTCAP |= ((unsigned int)(u8PortA[0]) << (n*8));
		//	printk(KERN_INFO "INTCAPA = 0x%02x.", u8PortA[0]);
		} else {
			printk(KERN_INFO "kur");
		}
		/* we use only 4 lines to fit a uint32! */
		n = (n + 1) % 8;
		
		set_current_state(TASK_INTERRUPTIBLE);
		msleep(TASK_PERIOD);
	}
	
	do_exit(0);
	
	return 0;
}

module_init(MCP23018_init);
module_exit(MCP23018_exit);
