# Geegah RP2040 Image Acquisition

# Introduction
This repository contains Python scripts for image acquisition and processing using the Geegah ultrasonic imager with RP2040 as the micro-processor. The Geegah Imager consists of a 128 x 128 or 256 x 256 array of transducers that transmit and receive high-frequency ultrasonic waves. These scripts are tailored to enable single/multi-frequency measurements for all the pixels as well as acquiring the whole transmit/receive pulses with multiple echoes. Additionally, scripts are provided to generate images from the raw ".dat" files and further compute the acoustic parameters of the acquired data. 

# Features
These scripts allow users to control the following features of the GHz ultrasonic transducers.
1. Real-time visualization of images
2. Frequency sweep
3. Obtaining a full pulse-echo response from each transducer (A-scan)

# Prerequisites
Before you use any of the acquisition of post-processing scripts, ensure you have the following on your computer:
1. Python 3. x installed
3. Python libraries for operations: sys, os, time, math, NumPy
4. Python libraries for image generation, visualization, and analysis: matplotlib, OpenCV (video generation and analysis)

An example of installing Matplotlib and OpenCV module using pip.

```bash
python -m pip install -U matplotlib
```

```bash
pip install opencv-python
```

# Image acquisition and processing scripts
The following scripts all allow for data acquisition using the Opal Kelly Imager. Each acquisition script has a section that allows users to load the raw data to generate images and compute different acoustic parameters.

1. **Geegah_RP2040_Liveimaging.py**: Acquisition of N number of frames with an option to visualize them in real-time for the entire 128 x 128 pixels. This operates at a fixed frequency and echo acquisition timing.
2.  **Geegah_RP2040_Liveimaging_withNoEcho.py**: Acquisition of N number of frames with an option to visualize them in real-time for the entire 128 x 128 pixels. This operates at a fixed frequency and echo acquisition timing. This also includes acquiring baseline acquisition at a later time when the echo dies off which can be used to calculate the true signal change of the samples yielding more accurate Acoustic Impedance.
3. **Geegah_RP2040_FrequencySweep.py**: Acquisition of images at a range of frequencies (1.5 - 2.0 GHz, with a step as low as 0.01 MHz). Allows capturing N frames at a fixed acquisition echo timing.
4. **Geegah_RP2040_SNH.py**: Acquisition of images at different echo timings with a step of 5 ns. This allows capturing N frames at a fixed frequency. 


# Helper libraries
These consist of classes or functions that support the acquisition and processing of scripts. These do not have to be run individually.

1. **geegah_hp.py** Helper functions for image acquisition and post-processing


# Getting started: 

**Clone the repository**
```bash
git clone git@github.com:Geegah-Inc/Geegah-RP2040.git
```

**Or, directly download the zip**
Click on 1) **Code** drop-down menu and click **Download ZIP** 
![alt text](https://github.com/Geegah-Inc/Geegah-RP2040/blob/main/ZIP_download.png)

**Usage Example**
All the scripts have been divided into the following sections:

1) Importing modules
2) Creating a directory and sub-folders to save raw and processed images
3) Changing parameters of interest
4) Finding and initializing the board/connection
5) Dummy frames
6) Acquiring air frames/background frames
7) Acquiring main sample frames
8) Plotting desired acoustic parameters
9) Relevant Post-processing

**1. Importing Modules**<br />
You simply have to run this once to load the necessary helper functions and Python libraries
```python
#import modules

import geegah_hp
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
import os
import numpy as np
import time
import serial
```python

**2. Directory assignment** <br />
Change the **foldernam**e to represent the main folder where all the sub-folders will be created, and the raw data files will be saved. <br />
Select the **savedirname** as the filepath where the **foldername** will be created

```python
foldername = "SampleExperiment4"
path = "C:/Users/anujb/Downloads"

savedirname = os.path.join(path, foldername, "")
```python

**3. Changing parameters of interest**<br />
This is the section where you need to assign correct values to the parameter that you might want to change for the acquisition


```python
liveplot = True #boolean for plotting images real-time, True or False
frequency = 1853.5 #Pulse frequency in MHz, with resolution of 0.1 MHz

#Selection of firing/receiving pixels, ROI 

liveplot = True #boolean for plotting images real-time, True or False, set this as True for live plotting
frequency = 1853 #Pulse frequency in MHz, with a resolution of 0.1 MHz

```python

Before proceeding, please ensure the Geegah Imager is powered on and connected to the PC via USB A/C.  <br />

**4. Initializing the board connection and settings**<br />
This section ensures the board is connected, and configures the relevant timing, DAC, and frequency settings to the connected board. 

```python
#BOARD SETUP AND INITIALIZATION


GPIO_PINNO_TEST = 27
# VCO, ad4351 PINS, SPI0, DEV0
GPIO_NUM_VCO_LE = 7  #SPI0_CE1 from Rpi

# SPI DAC new device, Configured on SPI0 with dedicated GPIO for C#
GPIO_NUM_DAC_MOSI = 10
GPIO_NUM_DAC_MISO = 9
GPIO_NUM_DAC_CLK = 11
GPIO_NUM_DAC_CE = 8 # STANDARD SPI0, dev0 CE, not used for DAC
GPIO_NUM_DAC_CE0B =22 # Dedicated DAC for 

pinDict_Main = dict(gpio_num_PINNO_TEST = GPIO_PINNO_TEST, 
                    gpio_num_DAC_CE0B = GPIO_NUM_DAC_CE0B,
                    gpio_num_VCO_LE = GPIO_NUM_VCO_LE);

...
...
...
#code continues#
```
...
...

```python

**5. Dummy frames after board configuration** <br />
This function acquires N number of frames to clear out the buffers so that the frames are acquired in the configured settings. There are 2 buffers, each storing total bits of I,Q (128x128 pixels), which are automatically filled once the board is connected. When any setting is changed, these buffers need to be cleared out before the frame representing the correct signal fills them up which is transferred to RPi4/PC during frame acquisition. Therefore, at least 2 frames need to be acquired and discarded after switching any timing-related, frequency, or board settings. This section takes care of that where N_dummy represents the number of dummy frames to discard. 

```python
geegah_hp.dummy_frames(spi_obj_pico, n = N_dummy)
```

Please ensure the chip surface is cleaned properly before running this section.


**6. Acquiring baseline frames** <br />
Enter the number of air frames/baseline frames to acquire for the experiment by changing  **NAIRSAMPLES** variable. 

```python
#%% CAPTURE BASELINE ECHO 


print("\n\n\n AQUISITION OF AIR FRAMES NOW: CLEAN THE IMAGER SURFACE")
a=input("Press Enter to proceed with air frames acquisition \n")

frames = 5
time_i = time.time()
for jj in range(frames):
    frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = geegah_hp.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                       n_bytes_block_arg=n_bytes_block_arg)
    #write AIR ECHO DAT FILES
    baseline_file_name = BLE_save_dir+"frame"+str(jj)+".dat"
    geegah_hp.writeFile(baseline_file_name, frames_data_nD_baseline)

...
...
...
#code continues#
```
**7. Acquiring sample frames** <br />
Enter the number of sample frames to acquire for the experiment by changing  the **NUM_IMAGE_SAMPLES** variable. <br />
If you have enabled the plotting (liveplot = True), a plot window pops up displaying the calculated Magnitude (V) of the signal real time.
```python

numFrames = 400


print("\n\n\n AQUISITION OF SAMPLE FRAMES NOW: PLACE THE SAMPLE NOW")
a=input("Press Enter to proceed with sample frames acquisition \n")

#initialize image window
if liveplot == True: 
    fig2,ax2 = plt.subplots(1)
    fig2.set_size_inches(2,2)
    mytitle = fig2.suptitle('Real-time Magnitude (V):  ')
       
    im2 = np.flipud(np.rot90(Q_AE_VOLTS,1))
    pos201 = ax2.imshow(im2, vmin = -0.1, vmax = 0.1, interpolation = 'bilinear')
    fig2.colorbar(pos201)
    base_title ='Real-time Magnitude (V):  '


time_i = time.time()
for jj in range(numFrames):
    #read an image
    frames_data_nD_sample,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = geegah_hp.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                           n_bytes_block_arg=n_bytes_block_arg)
    
...
...
...
#extract AIR frames for LIVE PLOTTING if liveplot == True
...
...
#LOOP FOR MAIN SAMPLE FRAMES ACQUISITION
#code continues#
```

**8. Process the images** <br />
Enter the details for loading previously saved data: Foldername, Path to saved data, # of airframes, and # of sample frames.
This will load all the echo/no-echo, I and Q values for Baseline and Sample frames and place them in their respective arrays. It will then compute the arrays for Magnitude, Phase, Reflection coefficient, and Acoustic Impedance.
```python

foldername_PP = "Test"
path_PP = "C:/Users/anujb/Downloads"
savedirname_PP = os.path.join(path_PP, foldername_PP, "")

BLE_save_dir = savedirname_PP+'rawdata_baseline_echo/'
rawdata_save_dir = savedirname_PP+'rawdata_echo/'

num_AIR_Frames = 10 #Number of air/baseline frames to consider during computation
num_SAMPLE_Frames = 100 #Number of frames to measured sample frames

I_A_E, Q_A_E, I_A_NE, Q_A_NE = [],[],[],[]
I_S_E, Q_S_E = [],[]

#LOAD AIR DATA
for frame in range(num_AIR_Frames):
   
    AE_filename = BLE_save_dir + "DATA"+str(frame)+".dat"
  
...
...
...
#code continues#
```
**9. Plotting** <br />
Change the variable **LIST_TO_PLOT** to the array of the acoustic parameter you want to plot. **Parameter** is just the parameter name. <br />
Running this section will create a new folder named **Parameter** in your directory and save the ".png" of the frames. It will also generate a video of these images.

```python
LIST_to_PLOT = ACOUSTIC_IMP
Parameter = "Acoustic Impedance"
geegah_hp.imgvid_plot_IMG(LIST_to_PLOT,savedirname_PP, 
                          foldername = Parameter,
                          vmin = 0.6,vmax = 3)
```




# Contact

Anuj Baskota
anuj@geegah.com


   

   


