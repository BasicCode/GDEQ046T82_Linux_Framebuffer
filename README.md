# GDEQ046T82_Linux_Framebuffer
A Linux kernel module for the Good Display GDEQ046T82 display which uses the SSD1677 controller chip.
  
**This is still a work in progress. Does not work yet.**
  
## Usage
The driver uses *ioctl*, *sysfs*, and *mmap*. The internal *fb_read()* function takes data written to */dev/fbx*, converts it to 2bpp data, and stores it in an internal buffer for rendering later.

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
```echo 1 > /sys/bus/spi/devices/spi0.0/update_display```

### mmap
Should work with mmap but haven't confirmed yet.