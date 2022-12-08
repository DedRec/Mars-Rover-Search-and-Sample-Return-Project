import cv2  # OpenCV for perspective transform
import matplotlib
import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import glob  # For reading in a list of images from a folder
import imageio

def perspect_transform(img, src, dst, kernel_size=3):
    # Apply gaussian filter that smooths to removes noise
    img = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # same size as input images

    return warped


def color_thresh(img, rgb_thresh=(160, 160, 160), kernel_size=3):
    # apply gaussian filter uses kernel to blur the image to get rid of noise on output and increase accuracy of mappying.
    img = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])

    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will contain a boolean array with "True" whenever threshold is met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                   & (img[:, :, 1] > rgb_thresh[1]) \
                   & (img[:, :, 2] > rgb_thresh[2])

    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


# rover_coords function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float32)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float32)

    return x_pixel, y_pixel


# to_polar_coords function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    # Calculate angle away from vertical x-axis for each pixel
    angles = np.arctan2(y_pixel, x_pixel)

    return dist, angles


# rotate_pix function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))

    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos

    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world


def debug():
    path = '../IMG_RUN/*'
    img_list = glob.glob(path)
    if len(img_list) != 0:
        idx = np.random.randint(0, len(img_list) - 1)
        image = mpimg.imread(img_list[idx])
        dst_size = 5
        bottom_offset = 6
        source = np.float32([[19, 140],[303, 140],[200, 95],[120, 95]])
        destination = np.float32([[image.shape[1] / 2 - dst_size,
                                   image.shape[0] - bottom_offset],
                                  [image.shape[1] / 2 + dst_size, image.shape[0] - bottom_offset],
                                  [image.shape[1] / 2 + dst_size, image.shape[0] - dst_size * 2 - bottom_offset],
                                  [image.shape[1] / 2 - dst_size, image.shape[0] - dst_size * 2 - bottom_offset]])
        i = 0
        while len(img_list) > 0:
            image = mpimg.imread(img_list[i])
            warped = perspect_transform(image, source, destination)  # wraped is the bird-eye view perspective
            threshed = color_thresh(warped)

            # Calculate pixel values in rover-centric coords and distance/angle to all pixels
            xpix, ypix = rover_coords(threshed)
            dist, angles = to_polar_coords(xpix, ypix)
            mean_dir = np.mean(angles)
            fig = plt.figure(figsize=(12, 9))
            plt.subplot(221)
            plt.imshow(image)
            plt.subplot(222)
            plt.imshow(warped)
            plt.subplot(223)
            plt.imshow(threshed, cmap='gray')
            plt.subplot(224)
            plt.plot(xpix, ypix, '.')
            plt.ylim(-160, 160)
            plt.xlim(0, 160)
            arrow_length = 100
            x_arrow = arrow_length * np.cos(mean_dir)
            y_arrow = arrow_length * np.sin(mean_dir)
            plt.arrow(0, 0, x_arrow, y_arrow, color='red', zorder=2, head_width=10, width=2)
            plt.savefig('../output_RUN/image' + str(i) + '.png')
            matplotlib.pyplot.close()
            i = i+1

if __name__ == '__main__':
    debug()

