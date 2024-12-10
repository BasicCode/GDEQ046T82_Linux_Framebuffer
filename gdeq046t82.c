// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DRM driver for Good Display GDEQ046T82 e-ink panels
 * using the SSD1677 controller IC.
 */

#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#include <drm/drm_mipi_dbi.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_vram_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper.h>
#include <video/mipi_display.h>
#include <drm/drm_framebuffer.h>

//Constants
#define GDEQ046T82_HEIGHT 480
#define GDEQ046T82_WIDTH 800
#define GDEQ046T82_BUSY_TIMEMOUT 10000
//#define BPP 24
#define TIMER_INTERVAL 10 * HZ // 10 seconds in jiffies

//Control signals
struct gpio_desc *dc_gpio, *busy_gpio, *reset_gpio;

//Dummy or double buffer
u8 out_buffer_black[(GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT) / 8];
u8 out_buffer_gray[(GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT) / 8];

//Global full-refresh counter
int refresh_counter = 0;

/*
 Down-samples the 8bpp framebuffer or other image to 2bpp for the
 display.
*/
void gdeq046t82_framebuffer_to_buffer(u8 *buf, u32 width, u32 height, u8 bpp) {
    const int gray_threshold_low = 50;
    const int gray_threshold_high = 150;
    const int red_weight = 212;
    const int green_weight = 715;
    const int blue_weight = 72;
	int bytes_per_pixel = bpp / 8;

    //For each pixel, extract the R, G, and B components,
    //and then convert to gray.
    for(int i = 0; i < width * height * bytes_per_pixel; i += bytes_per_pixel) {
        int r = buf[i];
        int g = buf[i + 1];
        int b = buf[i + 2];

        int gray = (int)((red_weight * r + green_weight * g + blue_weight * b) / 1000);
        /*Now the display takes 1-bit arrays for data. One for black/white
        * and one for gray. So we convert our gray data in to two
        * high/low arrays.
        */
        if(gray > gray_threshold_high)
            out_buffer_black[(int)(i / bpp)] |= (0x01 << (7 - ((i / bytes_per_pixel) % 8)));
        else
            out_buffer_black[(int)(i / bpp)] &= ~(0x01 << (7 - ((i / bytes_per_pixel) % 8)));
        if(gray > gray_threshold_low && gray < gray_threshold_high)
            out_buffer_gray[(int)(i / bpp)] |= (0x01 << (7 - ((i / bytes_per_pixel) % 8)));
        else
            out_buffer_gray[(int)(i / bpp)] &= ~(0x01 << (7 - ((i / bytes_per_pixel) % 8)));
    }
}

static void gdeq046t82_pipe_enable(struct drm_simple_display_pipe *pipe,
				struct drm_crtc_state *crtc_state,
				struct drm_plane_state *plane_state)
{

}

static void gdeq046t82_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;

	DRM_DEBUG_KMS("\n");
}


/*
 Wait for the busy signal to clear
*/
int gdeq046t82_busy_wait(unsigned long int timeout_ms) {
    unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);

    while (time_before(jiffies, timeout)) {
        if (gpiod_get_value(busy_gpio) == 0) {
            //Wait for BUSY to go LOW
            return 0;
        }
        msleep(10); // Poll every 10 milliseconds
    }

    pr_err("gdeq046t82: busy_wait Timeout occurred\n");
    return -ETIMEDOUT;
}

/*
 * Instruct the display to perform either a partial (TRUE), or full (FALSE) update.
 */
void gdeq046t82_full_update(struct mipi_dbi *dbi, int fast_refresh) {
    //Just in case some muppet tries to call the update function too quickly
    gdeq046t82_busy_wait(GDEQ046T82_BUSY_TIMEMOUT);

    mipi_dbi_command(dbi, 0x21, 0x00, 0x00); // Display Update Controll,  See page 27 on datasheet
    if(fast_refresh) {
        mipi_dbi_command(dbi, 0x1A, 0x5A); // Write to temperature register
        mipi_dbi_command(dbi, 0x22, 0xD7);
    } else {
        mipi_dbi_command(dbi, 0x22, 0xF7);
    }
    mipi_dbi_command(dbi, 0x20);
}

/*
 * Set the partial ram area in the display controller memory.
 */
void gdeq046t82_set_ram_area(struct mipi_dbi *dbi, u16 x, u16 y, u16 width, u16 height) {
    gdeq046t82_busy_wait(GDEQ046T82_BUSY_TIMEMOUT); //Wait for the device to become ready

    mipi_dbi_command(dbi, 0x11, 0x03); // x increment, y increment
    mipi_dbi_command(dbi, 0x44, 
		(x & 0xFF), 
		((x >> 8) & 0b00000011 ), 
		((x + width - 1) & 0xFF), 
		((x + width - 1) >> 8 & 0b00000011));
    mipi_dbi_command(dbi, 0x45, 
		(y & 0xFF), 
		((y >> 8) & 0b00000011 ), 
		((y + height - 1) & 0xFF), 
		((y + height - 1) >> 8 & 0b00000011));
    mipi_dbi_command(dbi, 0x4e, 
		(x & 0xFF), 
		((x >> 8) & 0b00000011 ));
    mipi_dbi_command(dbi, 0x4f, 
		(y & 0xFF), 
		((y >> 8) & 0b00000011 ));
}



static void gdeq046t82_pipe_update(struct drm_simple_display_pipe *pipe,
				struct drm_plane_state *old_state)
{
    struct drm_plane_state *state = pipe->plane.state;
    struct drm_framebuffer *fb = state->fb;
    struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
    struct mipi_dbi *dbi = &dbidev->dbi;
	struct drm_gem_dma_object *gem_obj;
    struct drm_gem_vram_object *vram_obj;
    void *vaddr;
    u32 width, height;
    u8 bpp;
    int ret, idx;


    if (!fb)
        return;

    if (!drm_dev_enter(pipe->crtc.dev, &idx))
        return;

    width = fb->width;
    height = fb->height;
    bpp = fb->format->cpp[0] * 8; // bits per pixel

    gem_obj = drm_fb_dma_get_gem_obj(fb, 0);
	void *src = gem_obj->vaddr;

	gdeq046t82_framebuffer_to_buffer((u8 *)src, width, height, bpp);

	gdeq046t82_set_ram_area(dbi, 0, 0, GDEQ046T82_WIDTH, GDEQ046T82_HEIGHT);

    /* Write the framebuffer content to the display */
    mipi_dbi_command_buf(dbi, 0x26,
                               out_buffer_gray,
                               (GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT) / 8);
    mipi_dbi_command_buf(dbi, 0x24,
                               out_buffer_black,
                               (GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT) / 8);
	
	//Update the display. Full refresh every 6th update.
	if(refresh_counter % 6 == 0) {
		gdeq046t82_full_update(dbi, false); // full refresh
	} else {
		gdeq046t82_full_update(dbi, true); //Partial update
	}
	refresh_counter++;

	//Verry slow updates
	msleep(1000);

    drm_dev_exit(idx);
}

/*
 Clear the display to white
*/
void gdeq046t82_clear(struct mipi_dbi *dbi) {
    //Fill the buffer with some value
    for(int i = 0; i < (GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT) / 8; i++) {
        out_buffer_black[i] = 0xFF;
        out_buffer_gray[i] = 0xFF;
    }

	

    gdeq046t82_set_ram_area(dbi, 0, 0, GDEQ046T82_WIDTH, GDEQ046T82_HEIGHT);

	mipi_dbi_command_buf(dbi, 0x26, out_buffer_gray, (GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT) / 8);

    gdeq046t82_set_ram_area(dbi, 0, 0, GDEQ046T82_WIDTH, GDEQ046T82_HEIGHT);

	mipi_dbi_command_buf(dbi, 0x24, out_buffer_black, (GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT) / 8);

    gdeq046t82_full_update(dbi, false); // full refresh
}

static void gdeq046t82_enable(struct drm_simple_display_pipe *pipe,
			     struct drm_crtc_state *crtc_state,
			     struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	int ret, idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

    // Example initialization sequence
    pr_info("gdeq046t82: Initialise Display in B&W mode.\n");

    //Initialisation routine from
    //https://github.com/ZinggJM/GxEPD2/blob/master/src/gdeq/GxEPD2_426_GDEQ0426T82.cpp#L349
	mipi_dbi_hw_reset(dbi);

    msleep(10);
    mipi_dbi_command(dbi, 0x12);
    msleep(10);
    mipi_dbi_command(dbi, 0x18, 0x80);
	mipi_dbi_command(dbi, 0x0c, 0xae, 0xc7, 0xc3, 0xc0, 0x80); //Something wrong with this specific call
    mipi_dbi_command(dbi, 0x01, 
		(GDEQ046T82_HEIGHT - 1) & 0xFF, 
		((GDEQ046T82_HEIGHT - 1) >> 8) && 0b00000011, 
		0x02,
		0x02);
    mipi_dbi_command(dbi, 0x3C, 0x01);

    gdeq046t82_set_ram_area(dbi, 0, 0, GDEQ046T82_WIDTH, GDEQ046T82_HEIGHT);

	//Clear the display
	gdeq046t82_clear(dbi);

	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);

out_exit:
	drm_dev_exit(idx);
}


static const u32 gdeq045t82_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static const struct drm_simple_display_pipe_funcs gdeq046t82_pipe_funcs = {
	.mode_valid = mipi_dbi_pipe_mode_valid,
	.enable = gdeq046t82_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = gdeq046t82_pipe_update,
};


static const struct drm_display_mode gdeq046t82_mode = {
	DRM_SIMPLE_MODE(800, 480, 95, 65),
};

DEFINE_DRM_GEM_DMA_FOPS(gdeq046t82_fops);

static const struct drm_driver gdeq046t82_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &gdeq046t82_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	.name			= "gdeq046t82",
	.desc			= "Good Display GDEQ046T82",
	.date			= "20241206",
	.major			= 1,
	.minor			= 0,
};

static int gdeq046t82_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *rs;
	u32 rotation = 0;
	int ret;

	dbidev = devm_drm_dev_alloc(dev, &gdeq046t82_drm_driver,
				    struct mipi_dbi_dev, drm);
	if (IS_ERR(dbidev))
		return PTR_ERR(dbidev);

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	dbi->reset = devm_gpiod_get(dev, "res", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset))
		return dev_err_probe(dev, PTR_ERR(dbi->reset), "Failed to get GPIO 'reset'\n");

	// DC GPIO line
	dc_gpio = devm_gpiod_get(dev, "dc", GPIOD_OUT_HIGH);
	if (IS_ERR(dc_gpio)) {
		dev_err(dev, "Invalid DC GPIO pin\n");
		return PTR_ERR(dc_gpio);
	}

	// BUSY GPIO Line
	busy_gpio = devm_gpiod_get(dev, "busy", GPIOD_IN);
	if (IS_ERR(busy_gpio)) {
		dev_err(dev, "Invalid BUSY GPIO pin\n");
		return PTR_ERR(busy_gpio);
	}

	ret = mipi_dbi_spi_init(spi, dbi, dc_gpio);
	if (ret)
		return ret;

	//RESET GPIO line
	reset_gpio = dbi->reset;

	//Let's not read from this controller for the moment
	dbi->read_commands = NULL;

	ret = mipi_dbi_dev_init_with_formats(dbidev, &gdeq046t82_pipe_funcs,
					     gdeq045t82_formats, ARRAY_SIZE(gdeq045t82_formats),
					     &gdeq046t82_mode, rotation, GDEQ046T82_WIDTH * GDEQ046T82_HEIGHT * 4);
	if (ret)
		return ret;

/*
	ret = mipi_dbi_dev_init(dbidev, &gdeq046t82_pipe_funcs, &gdeq046t82_mode, rotation);
	if (ret)
		return ret;
*/

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_fbdev_generic_setup(drm, 0);

	pr_info("gdeq046t82: Registered DRM Device as Tiny DRM.\n");

	return 0;
}

static void gdeq046t82_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void gdeq046t82_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static const struct of_device_id gdeq046t82_dt_ids[] = {
    { .compatible = "gooddisplay,gdeq046t82" },
    {},
};
MODULE_DEVICE_TABLE(of, gdeq046t82_dt_ids);

static struct spi_driver gdeq046t82_spi_driver = {
    .driver = {
        .name = "gdeq046t82",
        .of_match_table = gdeq046t82_dt_ids,
    },
    .probe = gdeq046t82_probe,
    .remove = gdeq046t82_remove,
};

module_spi_driver(gdeq046t82_spi_driver);

MODULE_AUTHOR("BasicCode");
MODULE_DESCRIPTION("Good Display GDEQ046T82 DRM driver");
MODULE_LICENSE("GPL v2");