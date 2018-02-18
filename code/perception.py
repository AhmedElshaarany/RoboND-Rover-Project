import numpy as np
import cv2


# Define Global variables
ROLL_THRESH = 0.35
PITCH_THRESH = 0.35

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    obstacle_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    obstacle_select[~above_thresh] = 1
    # Return the binary image
    return color_select, obstacle_select

# Define function that thresholds yellow rocks
def rock_extractor(img, lower_hsv_threshold, upper_hsv_threshold):
    # convert image to hsv color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # extract locations that fall within range of thresholds
    mask = cv2.inRange(hsv, lower_hsv_threshold, upper_hsv_threshold)
    return mask    


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
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

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    img = Rover.img
    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                  [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed, obstacle_map = color_thresh(warped)
    lower_yellow = np.array([80,100,100])
    upper_yellow = np.array([100,255,255])
    rock_location_in_img = rock_extractor(img, lower_yellow, upper_yellow)
    rock_location = perspect_transform(rock_location_in_img, source, destination)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacle_map * 255
    Rover.vision_image[:,:,1] = rock_location * 255
    Rover.vision_image[:,:,2] = threshed * 255
    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(threshed)
    xpix_obs, ypix_obs = rover_coords(obstacle_map)
    xpix_rock, ypix_rock = rover_coords(rock_location)
    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10
    navigable_x_world, navigable_y_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(xpix_obs, ypix_obs, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)
    rock_x_world, rock_y_world = pix_to_world(xpix_rock, ypix_rock, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if Rover.pitch > 180:
        pitch_diff = 360 - Rover.pitch
    else:
        pitch_diff = Rover.pitch

    if Rover.roll > 180:
        roll_diff = 360 - Rover.roll
    else:
        roll_diff = Rover.roll
        
    if pitch_diff < PITCH_THRESH and roll_diff < ROLL_THRESH:
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 10
    # 8) Convert rover-centric pixel positions to polar coordinates
    dist, angles = to_polar_coords(xpix, ypix)
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists = dist
    Rover.nav_angles = angles
    
    rock_dist, rock_angles = to_polar_coords(xpix_rock, ypix_rock)
    mean_dist_rock = np.mean(rock_dist)
    mean_angle_rock = np.mean(rock_angles)
    print("Mean Rock Dist = " + str(mean_dist_rock))
    print("Mean Rock Angle = " + str(mean_angle_rock))
    if not np.isnan(mean_dist_rock):
        Rover.distance_to_nearest_rock = mean_dist_rock
        Rover.angle_to_nearest_rock = mean_angle_rock
    
    return Rover
