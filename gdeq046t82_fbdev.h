#ifndef GDEQ046T82_H
#define GDEQ046T82_H

// Display controlls
int gdeq046t82_write_command(struct spi_device *spi, u8 cmd);
int gdeq046t82_write_data(struct spi_device *spi, u8 data);
int gdeq046t82_busy_wait(unsigned int timeout_ms);
void gdeq046t82_clear(void);
void gdeq046t82_grayscale_test(void);
void gdeq046t82_draw_out_buffers(int fast_refresh);
void gdeq046t82_write_buffer(int command, u8 *buffer);
void gdeq046t82_set_ram_area(u16 x, u16 y, u16 width, u16 height);
void gdeq046t82_full_update(int fast_refresh);
void gdeq046t82_update_grayscale(void);
void gdeq046t82_power_on(struct spi_device *spi); 
void gdeq046t82_reset(void);
void gdeq046t82_sleep(void);
int gdeq046t82_init_grayscale(struct spi_device *spi);
int gdeq046t82_init_display(struct spi_device *spi);
void gdeq046t82_framebuffer_to_buffer(void);
static void gdeq046t82_free_framebuffer(struct fb_info *info);
static ssize_t gdeq046t82_write(struct fb_info *info, const char __user *buf, size_t count, loff_t *ppos);
static ssize_t gdeq046t82_read(struct fb_info *info, char __user *buf, size_t count, loff_t *ppos);
// IOCTL, mmap, sysfs
static int gdeq046t82_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg);
static int gdeq046t82_mmap(struct fb_info *info, struct vm_area_struct *vma);
static ssize_t gdeq046t82_sysfs_update(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
int gdeq046t82_probe(struct spi_device *spi);
void gdeq046t82_remove(struct spi_device *spi);
void gdeq046t82_shutdown(struct spi_device *spi);

#endif // GDEQ046T82_H
