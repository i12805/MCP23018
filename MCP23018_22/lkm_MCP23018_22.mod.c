#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xbd97005b, "module_layout" },
	{ 0x61dc5886, "param_ops_uint" },
	{ 0x223fc350, "kthread_stop" },
	{ 0xba4c293, "gpiod_set_raw_value" },
	{ 0x741da963, "i2c_del_driver" },
	{ 0x987349fe, "i2c_unregister_device" },
	{ 0xfe990052, "gpio_free" },
	{ 0xfbb00adc, "gpiod_unexport" },
	{ 0xc1514a3b, "free_irq" },
	{ 0x977035dc, "wake_up_process" },
	{ 0x31556c9c, "kthread_create_on_node" },
	{ 0x84f7e14e, "kobject_put" },
	{ 0xa1166360, "sysfs_create_group" },
	{ 0x1ab29b47, "kobject_create_and_add" },
	{ 0x67395882, "kernel_kobj" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x4293a604, "gpiod_to_irq" },
	{ 0x8f411754, "gpiod_get_raw_value" },
	{ 0x2e1cbb5d, "gpiod_export" },
	{ 0x22c45e16, "gpiod_set_debounce" },
	{ 0x2595ac62, "gpiod_direction_input" },
	{ 0x3d71b056, "i2c_put_adapter" },
	{ 0x38bfef0a, "i2c_register_driver" },
	{ 0xe8137a53, "i2c_new_client_device" },
	{ 0xdafd094c, "i2c_get_adapter" },
	{ 0x4ccd9305, "gpiod_direction_output_raw" },
	{ 0xe30324cf, "gpio_to_desc" },
	{ 0x47229b5c, "gpio_request" },
	{ 0x952664c5, "do_exit" },
	{ 0xb3f7646e, "kthread_should_stop" },
	{ 0xf9a482f9, "msleep" },
	{ 0x6df1aaf1, "kernel_sigaction" },
	{ 0x8e865d3c, "arm_delay_ops" },
	{ 0x5f754e5a, "memset" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x86332725, "__stack_chk_fail" },
	{ 0x400b004b, "i2c_transfer" },
	{ 0x8f678b07, "__stack_chk_guard" },
	{ 0xc5850110, "printk" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("i2c:MCP23018_22_I2C_DRV");

MODULE_INFO(srcversion, "43140DFE2C4D39F0A83BA5B");
