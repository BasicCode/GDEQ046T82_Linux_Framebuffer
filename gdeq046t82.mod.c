#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

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

#ifdef CONFIG_MITIGATION_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const char ____versions[]
__used __section("__versions") =
	"\x14\x00\x00\x00\x92\xb1\x4b\x1c"
	"_dev_err\0\0\0\0"
	"\x1c\x00\x00\x00\xcb\xf6\xfd\xf0"
	"__stack_chk_fail\0\0\0\0"
	"\x20\x00\x00\x00\xc7\x33\xc3\x5e"
	"__devm_drm_dev_alloc\0\0\0\0"
	"\x18\x00\x00\x00\x74\x29\xe6\xeb"
	"devm_gpiod_get\0\0"
	"\x1c\x00\x00\x00\x58\x84\x1a\xd1"
	"mipi_dbi_spi_init\0\0\0"
	"\x28\x00\x00\x00\x07\xd1\x6d\x94"
	"mipi_dbi_dev_init_with_formats\0\0"
	"\x20\x00\x00\x00\xbb\xbe\x49\x44"
	"drm_mode_config_reset\0\0\0"
	"\x1c\x00\x00\x00\x7d\xbb\xcc\x6f"
	"drm_dev_register\0\0\0\0"
	"\x1c\x00\x00\x00\x75\xf2\xab\xf2"
	"drm_fbdev_dma_setup\0"
	"\x18\x00\x00\x00\x0d\xec\x12\x03"
	"dev_err_probe\0\0\0"
	"\x1c\x00\x00\x00\xb2\x13\xb0\xf8"
	"driver_unregister\0\0\0"
	"\x10\x00\x00\x00\xa6\x50\xba\x15"
	"jiffies\0"
	"\x10\x00\x00\x00\xf9\x82\xa4\xf9"
	"msleep\0\0"
	"\x18\x00\x00\x00\xbd\xba\xf5\xdd"
	"gpiod_get_value\0"
	"\x18\x00\x00\x00\x87\x0e\xf5\xec"
	"drm_dev_enter\0\0\0"
	"\x20\x00\x00\x00\x30\x89\xe1\x4b"
	"drm_fb_dma_get_gem_obj\0\0"
	"\x20\x00\x00\x00\x70\x75\x95\xf7"
	"mipi_dbi_command_buf\0\0\0\0"
	"\x18\x00\x00\x00\xdf\x34\xa0\xe8"
	"drm_dev_exit\0\0\0\0"
	"\x28\x00\x00\x00\xb3\x1c\xa2\x87"
	"__ubsan_handle_out_of_bounds\0\0\0\0"
	"\x1c\x00\x00\x00\xa6\xc0\xe9\x7e"
	"mipi_dbi_hw_reset\0\0\0"
	"\x20\x00\x00\x00\x87\x98\x9b\x49"
	"mipi_dbi_enable_flush\0\0\0"
	"\x30\x00\x00\x00\x37\x40\xb9\x07"
	"drm_gem_dma_prime_import_sg_table_vmap\0\0"
	"\x20\x00\x00\x00\xad\xf6\xcc\x79"
	"drm_gem_dma_dumb_create\0"
	"\x14\x00\x00\x00\xdc\x5b\x69\xb8"
	"noop_llseek\0"
	"\x14\x00\x00\x00\x47\xee\x64\x8b"
	"drm_read\0\0\0\0"
	"\x14\x00\x00\x00\x89\x23\x2c\xae"
	"drm_poll\0\0\0\0"
	"\x14\x00\x00\x00\x09\x2d\x6a\x54"
	"drm_ioctl\0\0\0"
	"\x1c\x00\x00\x00\xc9\x81\xca\x3e"
	"drm_compat_ioctl\0\0\0\0"
	"\x18\x00\x00\x00\x84\x2d\xad\x35"
	"drm_gem_mmap\0\0\0\0"
	"\x14\x00\x00\x00\x14\x2f\xd2\xed"
	"drm_open\0\0\0\0"
	"\x14\x00\x00\x00\xbb\xdc\x8c\x07"
	"drm_release\0"
	"\x24\x00\x00\x00\xcb\xf2\x0b\x3c"
	"mipi_dbi_pipe_mode_valid\0\0\0\0"
	"\x20\x00\x00\x00\xa7\xd8\x37\xc5"
	"mipi_dbi_pipe_disable\0\0\0"
	"\x14\x00\x00\x00\xbb\x6d\xfb\xbd"
	"__fentry__\0\0"
	"\x20\x00\x00\x00\xc5\x32\x30\x4b"
	"__spi_register_driver\0\0\0"
	"\x1c\x00\x00\x00\xca\x39\x82\x5b"
	"__x86_return_thunk\0\0"
	"\x18\x00\x00\x00\x34\xe7\x86\x83"
	"drm_dev_unplug\0\0"
	"\x24\x00\x00\x00\x26\x9e\x82\xa6"
	"drm_atomic_helper_shutdown\0\0"
	"\x24\x00\x00\x00\x35\x50\xd9\xc2"
	"mipi_dbi_command_stackbuf\0\0\0"
	"\x10\x00\x00\x00\x7e\x3a\x2c\x12"
	"_printk\0"
	"\x18\x00\x00\x00\x81\xc8\x24\x1d"
	"___ratelimit\0\0\0\0"
	"\x18\x00\x00\x00\x26\x0c\xae\x84"
	"module_layout\0\0\0"
	"\x00\x00\x00\x00\x00\x00\x00\x00";

MODULE_INFO(depends, "drm_mipi_dbi,drm_dma_helper");

MODULE_ALIAS("of:N*T*Cgooddisplay,gdeq046t82");
MODULE_ALIAS("of:N*T*Cgooddisplay,gdeq046t82C*");

MODULE_INFO(srcversion, "8AE5A0E84E07D30F9673BAE");
