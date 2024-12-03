import fcntl
import os
from PIL import Image

# IOCTL commands
GDEQ046T82_IOCTL_BASE = ord('W')
GDEQ046T82_IOCTL_UPDATE_DISPLAY = (GDEQ046T82_IOCTL_BASE << 8) | 0
GDEQ046T82_IOCTL_FAST_REFRESH = (GDEQ046T82_IOCTL_BASE << 8) | 1
GDEQ046T82_IOCTL_FULL_REFRESH = (GDEQ046T82_IOCTL_BASE << 8) | 2

# Path to the image file
image_path = "test.jpg"

def display_image(image_path):
    # Open the image
    with Image.open(image_path) as img:
        # Resize the image to match the framebuffer resolution
        # You should adjust these dimensions to your specific resolution
        img = img.resize((800, 480))

        # Convert the image to RGB format
        img = img.convert('RGB')

        # Prepare the framebuffer data
        pixel_data = img.tobytes()

        try:
            with open("/dev/fb0", "r+b") as fb:
                # Write the pixel data to the framebuffer
                fb.write(pixel_data)

                # Perform the ioctl operation to update the display
                fcntl.ioctl(fb, GDEQ046T82_IOCTL_UPDATE_DISPLAY)
                # Tell the display to fast refresh
                fcntl.ioctl(fb, GDEQ046T82_IOCTL_FAST_REFRESH)
                print("Image displayed successfully")
        except OSError as e:
            print(f"Failed to open framebuffer device or perform ioctl: {e}")

if __name__ == "__main__":
    display_image(image_path)
