/**
 * Kernel module driver for the Good Display GDEQ046T82 e-ink
 * display.
 *
 * This display runs over SPI and uses two aditional control
 * signals: BUSY (busy-gpios), DC (dc-gpios), and RESET (res-gpios)
 * which should be defined in the device tree node.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/fb.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/ioctl.h>



#include "gdeq046t82.h"


#define GDEQ046T82_HEIGHT 480
#define GDEQ046T82_WIDTH 800
#define GDEQ046T82_BUSY_TIMEMOUT 10000
#define BPP 24
#define TIMER_INTERVAL 10 * HZ // 10 seconds in jiffies

// full screen update LUT 0~3 gray
const u8 lut_4G[] =
{ // 00 : VSS, 01 : VSH1, 10 : VSL, 11 : VSH2
  0x80, 0x48, 0x4A, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VS L0 (red 0, black 0) white
  //0x0A, 0x48, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VS L1 (red 1, black 0) light grey
  0x0A, 0x48, 0x68, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VS L1 (red 1, black 0) light grey
  // 0x88, 0x48, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VS L2 (red 0, black 1) dark grey
  0x88, 0x48, 0x60, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VS L2 (red 0, black 1) dark grey
  0xA8, 0x48, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VS L3 (red 1, black 1) black
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VS L4 (vcom)
  0x07, 0x1E, 0x1C, 0x02, 0x00, // TP 0 RP0 @50
  0x05, 0x01, 0x05, 0x01, 0x02, // TP 1 RP1
  0x08, 0x01, 0x01, 0x04, 0x04, // TP 2 RP2
  //0x00, 0x02, 0x00, 0x02, 0x02, // TP 3 RP3
  0x00, 0x02, 0x01, 0x02, 0x02, // TP 3 RP3
  0x00, 0x00, 0x00, 0x00, 0x00, // TP 4 RP4
  0x00, 0x00, 0x00, 0x00, 0x00, // TP 5 RP5
  0x00, 0x00, 0x00, 0x00, 0x00, // TP 6 RP6
  0x00, 0x00, 0x00, 0x00, 0x00, // TP 7 RP7
  0x00, 0x00, 0x00, 0x00, 0x00, // TP 8 RP8
  0x00, 0x00, 0x00, 0x00, 0x01, // TP 9 RP9
  0x22, 0x22, 0x22, 0x22, 0x22, // frame rate @100
  0x17, 0x41, 0xA8, 0x32, 0x30, // VGH, VSH1, VSH2, VSL, VCOM
  0x00, 0x00, // Reserve 1, Reserve 2
};

//Frame buffer
static struct fb_info *info;

static struct fb_ops gdeq046t82_fb_ops = {
    .owner = THIS_MODULE,
    .fb_read = gdeq046t82_read,
    .fb_write = gdeq046t82_write,
    .fb_fillrect = sys_fillrect,
    .fb_copyarea = sys_copyarea,
    .fb_imageblit = sys_imageblit,
    .fb_destroy = gdeq046t82_free_framebuffer,
    .fb_ioctl = gdeq046t82_ioctl,
    .fb_mmap = gdeq046t82_mmap
};

//Dummy or double buffer
u8 out_buffer_black[(GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT) / 8];
u8 out_buffer_gray[(GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT) / 8];

static struct spi_device *gdeq046t82_device;
//Control signals
struct gpio_desc *dc_gpio, *busy_gpio, *reset_gpio;

//sysfs device attribute struct
static DEVICE_ATTR(update_display, S_IRUGO | S_IWUSR, NULL, gdeq046t82_sysfs_update);

// Define `ioctl` commands
#define GDEQ046T82_IOCTL_BASE 'W'
#define GDEQ046T82_IOCTL_UPDATE_DISPLAY _IO(GDEQ046T82_IOCTL_BASE, 0)
#define GDEQ046T82_IOCTL_FAST_REFRESH _IO(GDEQ046T82_IOCTL_BASE, 1)
#define GDEQ046T82_IOCTL_FULL_REFRESH _IO(GDEQ046T82_IOCTL_BASE, 2)

/**
 * Display commands
 */
int gdeq046t82_write_command(struct spi_device *spi, u8 cmd)
{
    gpiod_set_value(dc_gpio, 0); // Command mode
    udelay(1);
    int ret = spi_write(spi, &cmd, 1);
    gpiod_set_value(dc_gpio, 1);

    return ret;
} 

int gdeq046t82_write_data(struct spi_device *spi, u8 data)
{
    gpiod_set_value(dc_gpio, 1); // Data mode
    udelay(1);
    int ret = spi_write(spi, &data, 1);

    return ret;
}

/*
 Wait for the busy signal to clear
*/
int gdeq046t82_busy_wait(unsigned int timeout_ms) {
    unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);

    while (time_before(jiffies, timeout)) {
        if (gpiod_get_value(busy_gpio) == 1) {
            //Wait for BUSY to go LOW
            return 0;
        }
        msleep(10); // Poll every 10 milliseconds
    }

    pr_err("gdeq046t82: busy_wait Timeout occurred\n");
    return -ETIMEDOUT;
}
 
/*
 Clear the display to white
*/
void gdeq046t82_clear(void) {
    //Fill the buffer with some value
    for(int i = 0; i < (GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT) / 8; i++) {
        out_buffer_black[i] = 0xFF;
        out_buffer_gray[i] = 0xFF;
    }

    gdeq046t82_write_buffer(0x26, out_buffer_gray); // set previous
    gdeq046t82_write_buffer(0x24, out_buffer_black); // set current
    gdeq046t82_full_update(false); // full refresh
}

/*
  Test pattern to test grayscale function
*/
void gdeq046t82_grayscale_test(void) {
  gdeq046t82_set_ram_area(0, 0, GDEQ046T82_WIDTH, GDEQ046T82_HEIGHT);

  gdeq046t82_write_command(gdeq046t82_device, 0x24);
  for (int i = 0; i < GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT / 8; i++) {
    gdeq046t82_write_data(gdeq046t82_device, 0x00);
  }
 for (int i = 0; i < GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT / 8; i++)  {
    gdeq046t82_write_data(gdeq046t82_device, 0xFF);
  }
 for (int i = 0; i < GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT / 8; i++)  {
    gdeq046t82_write_data(gdeq046t82_device, 0x00);
  }
 for (int i = 0; i < GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT / 8; i++)  {
    gdeq046t82_write_data(gdeq046t82_device, 0xFF);
  }
  gdeq046t82_write_command(gdeq046t82_device, 0x26);
 for (int i = 0; i < GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT / 8; i++)  {
    gdeq046t82_write_data(gdeq046t82_device, 0x00);
  }
 for (int i = 0; i < GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT / 8; i++)  {
    gdeq046t82_write_data(gdeq046t82_device, 0x00);
  }
 for (int i = 0; i < GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT / 8; i++)  {
    gdeq046t82_write_data(gdeq046t82_device, 0xFF);
  }
 for (int i = 0; i < GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT / 8; i++)  {
    gdeq046t82_write_data(gdeq046t82_device, 0xFF);
  }
  gdeq046t82_update_grayscale();
}

/*
 Send the commands to write the internal out_buffers to the dispaly.
 Called after they have been populated by a conversion function for
 example.
*/
void gdeq046t82_draw_out_buffers(int fast_refresh) {
    gdeq046t82_write_buffer(0x26, out_buffer_gray);
    gdeq046t82_write_buffer(0x24, out_buffer_black);
    gdeq046t82_full_update(fast_refresh);
}

/*
  Writes a buffer to the display at position x, y, with a specified
  width and height.
 */
void gdeq046t82_write_buffer(int command, u8 *buffer) {

    gdeq046t82_set_ram_area(0, 0, GDEQ046T82_WIDTH, GDEQ046T82_HEIGHT);
    gdeq046t82_write_command(gdeq046t82_device, command);
    int ret = spi_write(gdeq046t82_device, buffer, (GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT) / 8);

    if (ret < 0) { 
        dev_err(&gdeq046t82_device->dev, "SPI write failed: %d\n", ret); 
    }
}


/*
 * Set the partial ram area in the display controller memory.
 */
void gdeq046t82_set_ram_area(u16 x, u16 y, u16 width, u16 height) {
    gdeq046t82_busy_wait(GDEQ046T82_BUSY_TIMEMOUT); //Wait for the device to become ready

    gdeq046t82_write_command(gdeq046t82_device, 0x11); // set ram entry mode
    gdeq046t82_write_data(gdeq046t82_device, 0x03);    // x increment, y increment
    gdeq046t82_write_command(gdeq046t82_device, 0x44);
    gdeq046t82_write_data(gdeq046t82_device, (x & 0xFF) );
    gdeq046t82_write_data(gdeq046t82_device, ((x >> 8) & 0b00000011 ));
    gdeq046t82_write_data(gdeq046t82_device, ((x + width - 1) & 0xFF));
    gdeq046t82_write_data(gdeq046t82_device, ((x + width - 1) >> 8 & 0b00000011));
    gdeq046t82_write_command(gdeq046t82_device, 0x45);
    gdeq046t82_write_data(gdeq046t82_device, (y & 0xFF) );
    gdeq046t82_write_data(gdeq046t82_device, ((y >> 8) & 0b00000011 ));
    gdeq046t82_write_data(gdeq046t82_device, ((y + height - 1) & 0xFF));
    gdeq046t82_write_data(gdeq046t82_device, ((y + height - 1) >> 8 & 0b00000011));
    gdeq046t82_write_command(gdeq046t82_device, 0x4e);
    gdeq046t82_write_data(gdeq046t82_device, (x & 0xFF) );
    gdeq046t82_write_data(gdeq046t82_device, ((x >> 8) & 0b00000011 ));
    gdeq046t82_write_command(gdeq046t82_device, 0x4f);
    gdeq046t82_write_data(gdeq046t82_device, (y & 0xFF) );
    gdeq046t82_write_data(gdeq046t82_device, ((y >> 8) & 0b00000011 ));
}

void gdeq046t82_full_update(int fast_refresh) {
    //Just in case some muppet tries to call the update function too quickly
    gdeq046t82_busy_wait(GDEQ046T82_BUSY_TIMEMOUT);

    gdeq046t82_write_command(gdeq046t82_device, 0x21); // Display Update Controll
    gdeq046t82_write_data(gdeq046t82_device, 0x00);    // See page 27 on datasheet
    gdeq046t82_write_data(gdeq046t82_device, 0x00);    // single chip application
    if(fast_refresh) {
        gdeq046t82_write_command(gdeq046t82_device, 0x1A); // Write to temperature register
        gdeq046t82_write_data(gdeq046t82_device, 0x5A);
        gdeq046t82_write_command(gdeq046t82_device, 0x22);
        gdeq046t82_write_data(gdeq046t82_device, 0xd7);
    } else {
        gdeq046t82_write_command(gdeq046t82_device, 0x22);
        gdeq046t82_write_data(gdeq046t82_device, 0xf7);
    }
    gdeq046t82_write_command(gdeq046t82_device, 0x20);
}

/*
  Update the display in grayscale mode.

  NOT WORKING!
*/
void gdeq046t82_update_grayscale(void) {
    //Just in case some muppet tries to call the update function too quickly
    gdeq046t82_busy_wait(GDEQ046T82_BUSY_TIMEMOUT);

    gdeq046t82_write_command(gdeq046t82_device, 0x21); // Display Update Controll
    gdeq046t82_write_data(gdeq046t82_device, 0x00);    // See page 27 on datasheet
    gdeq046t82_write_data(gdeq046t82_device, 0x00);    // single chip application
    gdeq046t82_write_command(gdeq046t82_device, 0x22);
    gdeq046t82_write_data(gdeq046t82_device, 0xc7);
    gdeq046t82_write_command(gdeq046t82_device, 0x20);
}

void gdeq046t82_power_on(struct spi_device *spi) {
    gdeq046t82_write_command(spi, 0x22);
    gdeq046t82_write_data(spi, 0xe0);
    gdeq046t82_write_command(spi, 0x20);
    gdeq046t82_busy_wait(GDEQ046T82_BUSY_TIMEMOUT);
}

void gdeq046t82_reset(void) {
    //Cycle the RESET pin
    gpiod_set_value(reset_gpio, 0);
    mdelay(10);
    gpiod_set_value(reset_gpio, 1);
}

//Deep sleep function
void gdeq046t82_sleep(void)
{   
  gdeq046t82_write_command(gdeq046t82_device, 0x10);
  gdeq046t82_write_data(gdeq046t82_device, 0x01);
}

/*
  Initialise the display with grayscale LUT waveforms.

  NOT WORKING!
*/
int gdeq046t82_init_grayscale(struct spi_device *spi) {

    // Example initialization sequence
    pr_info("gdeq046t82: Initialise Display in Grayscale mode.\n");

    //Initialisation routine from
    //https://github.com/ZinggJM/GxEPD2/blob/master/src/gdeq/GxEPD2_426_GDEQ0426T82.cpp#L349
    gdeq046t82_write_command(spi, 0x0C); //set soft start
    gdeq046t82_write_data(spi, 0xAE);
    gdeq046t82_write_data(spi, 0xC7);
    gdeq046t82_write_data(spi, 0xC3);
    gdeq046t82_write_data(spi, 0xC0);
    gdeq046t82_write_data(spi, 0x80);
    gdeq046t82_write_command(spi, 0x01); // Driver output control
    gdeq046t82_write_data(spi, (GDEQ046T82_HEIGHT - 1) & 0xFF);
    gdeq046t82_write_data(spi, ((GDEQ046T82_HEIGHT - 1) >> 8) && 0b00000011);
    gdeq046t82_write_data(spi, 0x02); // SM (interlaced) ??
    gdeq046t82_write_command(spi, 0x3C); // Border setting
    gdeq046t82_write_data(spi, 0x00); // LUT0 (white)
    gdeq046t82_write_command(spi, 0x18); // use the internal temperature sensor
    gdeq046t82_write_data(spi, 0x80);
    gdeq046t82_set_ram_area(0, 0, GDEQ046T82_WIDTH, GDEQ046T82_HEIGHT);
    gdeq046t82_write_command(spi, 0x32); //Send new LUT values
    spi_write(spi, lut_4G, 105);
    gdeq046t82_write_command(spi, 0x03); //VGH
    gdeq046t82_write_data(spi, lut_4G[105]);
    gdeq046t82_write_command(spi, 0x04); //
    gdeq046t82_write_data(spi, lut_4G[106]); //VSH1
    gdeq046t82_write_data(spi, lut_4G[107]); //VSH2
    gdeq046t82_write_data(spi, lut_4G[108]); //VSL
    gdeq046t82_write_command(spi, 0x2C);     //VCOM Voltage
    gdeq046t82_write_data(spi, lut_4G[109]); //0x1C
    for(int i = 0; i < (GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT) / 8; i++) {
        out_buffer_black[i] = 0xFF;
        out_buffer_gray[i] = 0xFF;
    }

    gdeq046t82_write_buffer(0x26, out_buffer_gray); // set previous
    gdeq046t82_write_buffer(0x24, out_buffer_black); // set current

    return 0;
}

/*
  Initialise the display in regular B&W mode.
 */
int gdeq046t82_init_display(struct spi_device *spi)
{
    // Example initialization sequence
    pr_info("gdeq046t82: Initialise Display in B&W mode.\n");

    //Initialisation routine from
    //https://github.com/ZinggJM/GxEPD2/blob/master/src/gdeq/GxEPD2_426_GDEQ0426T82.cpp#L349
    gdeq046t82_reset();
    mdelay(10);
    gdeq046t82_write_command(spi, 0x12);
    mdelay(10);
    gdeq046t82_write_command(spi, 0x18);
    gdeq046t82_write_data(spi, 0x80);
    gdeq046t82_write_command(spi, 0x0C);
    gdeq046t82_write_data(spi, 0xAE);
    gdeq046t82_write_data(spi, 0xC7);
    gdeq046t82_write_data(spi, 0xC3);
    gdeq046t82_write_data(spi, 0xC0);
    gdeq046t82_write_data(spi, 0x80);
    gdeq046t82_write_command(spi, 0x01);
    gdeq046t82_write_data(spi, (GDEQ046T82_HEIGHT - 1) & 0xFF);
    gdeq046t82_write_data(spi, ((GDEQ046T82_HEIGHT - 1) >> 8) && 0b00000011);
    gdeq046t82_write_data(spi, 0x02);
    gdeq046t82_write_command(spi, 0x3C);
    gdeq046t82_write_data(spi, 0x01);
    gdeq046t82_set_ram_area(0, 0, GDEQ046T82_WIDTH, GDEQ046T82_HEIGHT);

    return 0;
}


int gdeq046t82_fb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue,
                                   unsigned transp, struct fb_info *info)
{
    // Set color register if necessary (usually for palette-based displays)
    return 0;
}

int gdeq046t82_fb_blank(int blank, struct fb_info *info)
{
    // Handle blanking if necessary
    return 0;
}

/*
Just frees the GPIOs in case of probe failure
 */
static void gdeq046t82_free_gpios(void) {
    gpiod_put(dc_gpio);
    gpiod_put(busy_gpio);
    gpiod_put(reset_gpio);
}

/*
 Just frees the framebuffer on unload
*/
static void gdeq046t82_free_framebuffer(struct fb_info *info) {
    unregister_framebuffer(info);
    vfree(info->screen_base);
    framebuffer_release(info);
}

/*
 Down-samples the 8bpp framebuffer or other image to 2bpp for the
 display.
*/
void gdeq046t82_framebuffer_to_buffer(void) {
    const int gray_threshold_low = 85;
    const int gray_threshold_high = 170;
    const int red_weight = 212;
    const int green_weight = 715;
    const int blue_weight = 72;

    //For each pixel, extract the R, G, and B components,
    //and then convert to gray.
    for(int i = 0; i < info->screen_size; i += 3) {
        int r = info->screen_base[i];
        int g = info->screen_base[i + 1];
        int b = info->screen_base[i + 2];
        int gray = (int)((red_weight * r + green_weight * g + blue_weight * b) / 1000);

        /*Now the display takes 1-bit arrays for data. One for black/white
        * and one for gray. So we convert our gray data in to two
        * high/low arrays.
        */
        if(gray > gray_threshold_high)
            out_buffer_black[(int)(i / BPP)] |= (0x01 << (7 - ((i / 3) % 8)));
        else
            out_buffer_black[(int)(i / BPP)] &= ~(0x01 << (7 - ((i / 3) % 8)));
        if(gray > gray_threshold_low && gray < gray_threshold_high)
            out_buffer_gray[(int)(i / BPP)] |= (0x01 << (7 - ((i / 3) % 8)));
        else
            out_buffer_gray[(int)(i / BPP)] &= ~(0x01 << (7 - ((i / 3) % 8)));
    }
}

static ssize_t gdeq046t82_write(struct fb_info *info, const char __user *buf, size_t count, loff_t *ppos) {
    ssize_t written = 0;

    if (*ppos >= info->screen_size)
        return -ENOSPC;

    if (*ppos + count > info->screen_size)
        count = info->screen_size - *ppos;

    if (copy_from_user(info->screen_base + *ppos, buf, count))
        return -EFAULT;

    *ppos += count;
    written = count;

    //Now down-conver the data for use internally
    gdeq046t82_framebuffer_to_buffer();

    return written;
}

static ssize_t gdeq046t82_read(struct fb_info *info, char __user *buf, size_t count, loff_t *ppos) {
 
    ssize_t read_bytes = 0;

    if (*ppos >= info->screen_size)
        return 0; // No more data to read

    if (*ppos + count > info->screen_size)
        count = info->screen_size - *ppos; // Adjust count to avoid overflow

    if (copy_to_user(buf, info->screen_base + *ppos, count))
        return -EFAULT; // Copy data from framebuffer memory to user space

    *ppos += count; // Update the file offset to the new position
    read_bytes = count;

    return read_bytes;
}

int gdeq046t82_probe(struct spi_device *spi)
{
    struct device *dev = &spi->dev;
    int ret;

    pr_info("gdeq046t82: Probing device.\n");

    gdeq046t82_device = spi;

    // Get the DC GPIO pin from the device tree
    dc_gpio = gpiod_get_index(dev, "dc", 0, GPIOD_OUT_HIGH);
    if (IS_ERR(dc_gpio)) {
        dev_err(dev, "Invalid DC GPIO pin\n");
        return -EINVAL;
    }

    // Get the BUSY GPIO pin from the device tree
    busy_gpio = gpiod_get_index(dev, "busy", 0, GPIOD_IN);
    if (IS_ERR(busy_gpio)) {
        dev_err(dev, "Invalid BUSY GPIO pin\n");
        return -EINVAL;
    }

    // Get the RESET GPIO pin from the device tree
    reset_gpio = gpiod_get_index(dev, "res", 0, GPIOD_OUT_HIGH);
    if(IS_ERR(reset_gpio)) {
        dev_err(dev, "Invalid RESET GPIO pin\n");
        return -EINVAL;
    }

    info = framebuffer_alloc(0, dev);
    if (!info) {
        dev_err(dev, "Failed to allocate framebuffer\n");
        gdeq046t82_free_gpios();
        return -ENOMEM;
    }
    info->var = (struct fb_var_screeninfo) {
        .xres = GDEQ046T82_WIDTH,
        .yres = GDEQ046T82_HEIGHT,
        .bits_per_pixel = BPP,
        .red = {16, 8, 0},
        .green = {8, 8, 0},
        .blue = {0, 8, 0},
        .activate = FB_ACTIVATE_NOW,
    };
    info->fix.line_length = GDEQ046T82_WIDTH * (BPP / 8);
    info->fix.visual = FB_VISUAL_TRUECOLOR;
    info->fbops = &gdeq046t82_fb_ops;
    info->screen_size = GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT * (BPP / 8);
    info->screen_base = vzalloc(info->screen_size);
    if (!info->screen_base) {
        dev_err(dev, "Failed to allocate framebuffer memory\n");
        framebuffer_release(info);
        gdeq046t82_free_gpios();
        return -ENOMEM;
    }
    info->fbops = &gdeq046t82_fb_ops;


    ret = register_framebuffer(info);
    if (ret < 0) {
        dev_err(dev, "Failed to register framebuffer\n");
        vfree(info->screen_base);
        framebuffer_release(info);
        gdeq046t82_free_gpios();
        return ret;
    }

    // Create sysfs entry 
    device_create_file(&spi->dev, &dev_attr_update_display);

    pr_info("Framebuffer registered: %s\n", info->fix.id);

    //Display Initialisation routine
    gdeq046t82_init_display(spi);

    //Test routine
    gdeq046t82_clear();

    //Put the driver to sleep
    gdeq046t82_sleep();

    return 0;
}


/*
  IOCTL, mmap, and sysfs
 */
static int gdeq046t82_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg) {
    switch (cmd) {
        case GDEQ046T82_IOCTL_UPDATE_DISPLAY:
            // Copy new data to the internal buffer.
            // The actual refresh happens later.
            gdeq046t82_framebuffer_to_buffer();
            break;
        case GDEQ046T82_IOCTL_FAST_REFRESH: // Perform a fast refresh
            gdeq046t82_draw_out_buffers(true);
            gdeq046t82_sleep();
            break;
        case GDEQ046T82_IOCTL_FULL_REFRESH: // Perform a fast refresh
            gdeq046t82_draw_out_buffers(false);
            gdeq046t82_sleep();
            break;
        default:
            return -ENOTTY; // Command not supported
    }
    return 0;
}

static int gdeq046t82_mmap(struct fb_info *info, struct vm_area_struct *vma) {
    unsigned long pfn = virt_to_phys(info->screen_base) >> PAGE_SHIFT;

    // Map the framebuffer memory
    return remap_pfn_range(vma, vma->vm_start, pfn, info->screen_size, vma->vm_page_prot);
}

/*
  Sysfs entries for manual use
  If the user sends a '1' then do a fast refresh,
  else do a full refresh.
*/
static ssize_t gdeq046t82_sysfs_update(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    pr_info("gdeq046t82: sysfs triggered update\n");

    gdeq046t82_init_display(gdeq046t82_device);
   
    //Check the value sent from the user
    if(buf[0] == '1')
        gdeq046t82_draw_out_buffers(true);
    else
        gdeq046t82_draw_out_buffers(false);

    //Put the device to sleep
    gdeq046t82_sleep();

    return count;
}

/**
 * Cleanup!
 */
void gdeq046t82_remove(struct spi_device *spi)
{
    pr_info("gdeq046t82: Removing device\n");

    //Remove sysfs entry
    device_remove_file(&spi->dev, &dev_attr_update_display);

    //Free the framebuffer
    gdeq046t82_free_framebuffer(info);

    // Free GPIOs
    gdeq046t82_free_gpios();
    
    return;
}

void gdeq046t82_shutdown(struct spi_device *spi) {
    //Do nothing, I think
}

static const struct of_device_id gdeq046t82_dt_ids[] = {
    { .compatible = "gooddisplay,gdeq046t82" },
    {},
};
MODULE_DEVICE_TABLE(of, gdeq046t82_dt_ids);

static struct spi_driver gdeq046t82_driver = {
    .driver = {
        .name = "gdeq046t82",
        .of_match_table = gdeq046t82_dt_ids,
    },
    .probe = gdeq046t82_probe,
    .remove = gdeq046t82_remove,
	.shutdown = gdeq046t82_shutdown
};

module_spi_driver(gdeq046t82_driver);

MODULE_AUTHOR("BasicCode");
MODULE_DESCRIPTION("Good Display GDEQ046T82 e-ink display driver");
MODULE_LICENSE("GPL");
