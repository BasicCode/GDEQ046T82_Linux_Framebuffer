# GDEQ046T82_Linux_Framebuffer
A Linux kernel module for the Good Display GDEQ046T82 display which uses the SSD1677 controller chip.
  
**This is still a work in progress. Only Partially Works.** So far it loads as a DRM device using the DRM Tiny system, and adds Framebuffer support. It is sufficient to play DOOM poorly.
  
## Usage
The driver uses DRM Tiny and also add framebuffer support.
* Kernel version 6.1.81 (STM Linux)

## References
 * [Product page](https://www.good-display.com/product/457.html)
 * [Controller IC](https://v4.cecdn.yun300.cn/100001_1909185148/SSD1677.pdf)
 * [GxEPD2 Arduino Code](https://github.com/ZinggJM/GxEPD2_4G/blob/master/src/gdeq/GxEPD2_426_GDEQ0426T82.cpp)
 * [Manufacturer's Sample Code](https://www.good-display.com/companyfile/1158.html)