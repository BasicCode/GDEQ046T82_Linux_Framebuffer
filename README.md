# GDEQ046T82_Linux_Framebuffer
A Linux kernel module for the Good Display GDEQ046T82 display which uses the SSD1677 controller chip.
  
**This is still a work in progress. Does not work yet.** So far it just enumerates as a framebuffer device, accepts data and converts it to the appropriate pixel format, and updates the display on command.
  
## Usage
The driver uses *ioctl*, *sysfs*, and *mmap*. The internal *fb_read()* function takes data written to */dev/fbx*, converts it to 2bpp data, and stores it in an internal buffer for rendering later. When you are ready to render the display use either ioctl or sysfs to trigger an update.
  
 * Tested on a Raspberry Pi 4. A Device Tree Overlay file is provided for the RPi 4.
 * A small helper sctips (*make_add.sh*) is provided because developing the driver requires frequent restarts. This will buid the driver, 'insmod' it, and set permissions on the sysfs location for user access.
 * A short python script (*fb_test.py*) is provided which sends an image to the display and calls the update function. Alternatively you can test it manually:
 ```
 $ dd if=/dev/urandom of=/dev/fb0
 $ echo 1 > /sys/bus/spi/devices/spi0.0/update_display
 ```

### ioctl
 * GDEQ046T82_IOCTL_UPDATE_DISPLAY 0
 Converts the provided buffer in to 2bpp data and stores it for updating at a later time.
 * GDEQ046T82_IOCTL_FAST_REFRESH 1
 Sends the stored buffer to the display and triggers a "fast update".
 * GDEQ046T82_IOCTL_FULL_REFRESH 2
 Sends the stored buffer to the display and triggers a "full update".

### sysfs
Sending either a '1', or '0' to the *update_display* sysfs function will send the internal buffers to the display, and trigger either a fast or full refresh respectively. For example:
  
Trigger a "fast" update of the display.
```
echo 1 > /sys/bus/spi/devices/spi0.0/update_display
```

### mmap
Should work with mmap but haven't confirmed yet.

## References
 * [Product page](https://www.good-display.com/product/457.html)
 * [Controller IC](https://v4.cecdn.yun300.cn/100001_1909185148/SSD1677.pdf)
 * [GxEPD2 Arduino Code](https://github.com/ZinggJM/GxEPD2_4G/blob/master/src/gdeq/GxEPD2_426_GDEQ0426T82.cpp)
 * [Manufacturer's Sample Code](https://www.good-display.com/companyfile/1158.html)