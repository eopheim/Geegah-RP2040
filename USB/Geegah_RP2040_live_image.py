from GeegahImager import RP1
import matplotlib.pyplot as plt
import numpy as np
import time
from datetime import datetime
import cv2

"""
    Ethan Opheim
    Jan 14, 2025
    Geegah Inc.

"""

verbose = False

save_video = False

rp1 = RP1(vco_freq=1853)

window_name = "Geegah: RP1 Imager"

# Set opencv window to use openGL
# RPi OS 
#cv2.namedWindow('I', cv2.WINDOW_AUTOSIZE | cv2.WINDOW_OPENGL)

# Windows 11
cv2.namedWindow(window_name, cv2.WINDOW_KEEPRATIO)

screen_width = 720
image_width = 128
# Screen width divided by ultrasonic image width
scale_factor = screen_width / image_width

# Ultrasonic image dimensions
dimensions = (screen_width,screen_width)

# Delay based on defined frame rate
# also used to set frame rate for video
fps = 7
delay = 1/fps

########################## VIDEO ###########################################
# Timestamp for file name
now = datetime.now()
timestamp = now.strftime('%b-%d-%Y_%H%M%S')

if save_video:
    # Below VideoWriter object will create 
    # a frame of above defined The output  
    # is stored in 'filename.avi' file. 
    result = cv2.VideoWriter(f'imager_{timestamp}.avi', cv2.VideoWriter_fourcc(*'MJPG'), fps=fps, frameSize=(screen_width*2,screen_width)) 

############################################################################

# Save frame data to subtract from future frames
i_frame_adc_air, q_frame_adc_air, i_frame_volts_air, q_frame_volts_air = rp1.get_frame()

# Define scale bar length in pixels
scale_bar_length = int(scale_factor * 16) # scale factor * number of pixels

# Just for viewing imager, loop for ever
while True:
    start_time = time.time()

    i_frame_adc, q_frame_adc, i_frame_volts, q_frame_volts = rp1.get_frame()

    if 1:
        i_image = i_frame_volts - i_frame_volts_air
        q_image = q_frame_volts - q_frame_volts_air
    else:
        i_image = i_frame_adc - i_frame_adc_air
        q_image = q_frame_adc - q_frame_adc_air

    if verbose:
        print(f"SPI transfer time: {time.time()-start_time}")

    start_time1 = time.time()

    # Flip image left right and up down
    i_image = np.flipud(np.fliplr(i_image))
    q_image = np.flipud(np.fliplr(q_image)) 

    # Normalize voltage to 8-bit grayscale values
    i_normalized_sub_image = cv2.normalize(i_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    q_normalized_sub_image = cv2.normalize(q_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U) 
   
    # Invert image for subtraction mode
    i_inverted_image = cv2.bitwise_not(i_normalized_sub_image)
    q_inverted_image = cv2.bitwise_not(q_normalized_sub_image)
    
    # Scale the image to fit the width of the screen
    i_scaled_image = cv2.resize(i_inverted_image, dimensions, interpolation=cv2.INTER_NEAREST)
    q_scaled_image = cv2.resize(q_inverted_image, dimensions, interpolation=cv2.INTER_NEAREST)
   
    # Convert grayscale image to 8-bit RGB
    i_color_image = cv2.cvtColor(i_scaled_image, cv2.COLOR_GRAY2RGB)
    q_color_image = cv2.cvtColor(q_scaled_image, cv2.COLOR_GRAY2RGB)

    # Place distance scale bar text on image
    cv2.putText(i_color_image, "I, " + str(50*16) + " um", (10,dimensions[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,255), 2, 2)

    cv2.putText(q_color_image, "Q, " + str(50*16) + " um", (10,dimensions[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,255), 2, 2)
    
    # Place distance scale bar on image
    i_color_image = cv2.line(i_color_image, (150,dimensions[1]-17), (150+scale_bar_length,dimensions[1]-17), (255,255,255), 2)
    q_color_image = cv2.line(q_color_image, (150,dimensions[1]-17), (150+scale_bar_length,dimensions[1]-17), (255,255,255), 2)

    combined_image = np.hstack((i_color_image, q_color_image))

    if save_video:
        result.write(combined_image)

    # Calculate time remaining in loop to meet fps period, uses to try and keep consistant frame times
    wait_time_ms = (int((delay - (time.time() - start_time))*1e3))
    
    if save_video:
        # Skip displaying frame if time remaining is less than 1 ms
        if wait_time_ms <= 0:
            continue
    else:
        wait_time_ms = 1

    # Display image
    cv2.imshow(window_name, combined_image)

    if verbose:
        print(f"Image processing time: {time.time()-start_time1}")
        print(f"fps: {1/(time.time()-start_time)}")
    
    # Keep image displayed for remaining time, exit program if opencv window was closed
    if cv2.waitKey(wait_time_ms) & 0xFF == ord('q') or cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
        cv2.destroyAllWindows()
        break

if save_video:
    result.release()
cv2.destroyAllWindows()

exit()