import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160), Above = True):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    if Above:
        above_thresh = (img[:,:,0] >= rgb_thresh[0]) \
                    & (img[:,:,1] >= rgb_thresh[1]) \
                    & (img[:,:,2] >= rgb_thresh[2])
        # Index the array of zeros with the boolean array and set to 1
        color_select[above_thresh] = 1
    else:
        below_thresh = (img[:,:,0] <= rgb_thresh[0]) \
                    & (img[:,:,1] <= rgb_thresh[1]) \
                    & (img[:,:,2] <= rgb_thresh[2])
        color_select[below_thresh] = 1
    
    # Return the binary image
    return color_select

def find_rocks(img):
    lower_bound = [100, 100, 0]
    upper_bound = [250, 250, 60]
    rock_idx_1 = color_thresh(img, rgb_thresh=lower_bound, Above=True)
    rock_idx_2 = color_thresh(img, rgb_thresh=upper_bound, Above=False)
    
    return (rock_idx_1.reshape([1, -1]) & rock_idx_2.reshape([1, -1])).reshape(img[:,:,0].shape)


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
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    return warped, mask


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
# Set a bottom offset to account for the fact that the bottom of the image 
# is not the position of the rover but a bit in front of it
# this is just a rough guess, feel free to change it!
    bottom_offset = 6
    image = Rover.img
#     img_max = np.float32(np.max(image))
#     image = np.float32(255)/img_max * image
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw
    world_size = Rover.worldmap.shape[0]
    scale =  2*dst_size
    warped , mask = perspect_transform(Rover.img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    nav_threshed = color_thresh(warped)
    obs_threshed = color_thresh(warped, Above = False)
#     nav_threshed = np.absolute(np.float32(obs_threshed) - 1) * mask 
    rock_threshed = find_rocks(warped)
    
    Rover.vision_image[:,:,0] = obs_threshed * 255
    Rover.vision_image[:,:,2] = nav_threshed * 255
    nav_xpix, nav_ypix = rover_coords(nav_threshed)
    obs_xpix, obs_ypix = rover_coords(obs_threshed)
    
    # 5) Convert rover-centric pixel values to world coords
    nav_x_world, nav_y_world = pix_to_world(nav_xpix, nav_ypix, xpos, ypos, yaw, world_size, scale)
    obs_x_world, obs_y_world = pix_to_world(obs_xpix, obs_ypix, xpos, ypos, yaw, world_size, scale)
    
    
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(nav_xpix, nav_ypix)
    
    rock_xpix, rock_ypix = rover_coords(rock_threshed)
    rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, xpos, ypos, yaw, world_size, scale)
    Rover.sample_dist , Rover.sample_angle = to_polar_coords(rock_xpix, rock_ypix)
   
    if rock_threshed.any():
#         rock_xpix, rock_ypix = rover_coords(rock_threshed)
#         rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, xpos, ypos, yaw, world_size, scale)
#         Rover.sample_dist , Rover.sample_angle = to_polar_coords(rock_xpix, rock_ypix)
#         rock_idx = np.argmin(Rover.sample_dist)
#         xcen = rock_x_world[rock_idx]
#         ycen = rock_x_world[rock_idx]
#         Rover.worldmap[ycen, xcen, 1] = 255
        Rover.vision_image[:,:,1] = rock_threshed * 255
    else:
        Rover.vision_image[:,:,1] = 0
        
    
    if Rover.roll <= 0.5 or Rover.roll >= 359.5:
        if Rover.pitch <= 0.5 or Rover.pitch >= 359.5:
            Rover.worldmap[obs_y_world, obs_x_world, 0] += 255
            Rover.worldmap[nav_y_world, nav_x_world, :] += 255
            Rover.worldmap[rock_x_world, rock_y_world, 1] += 255
    
    
    
    
    
    return Rover