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

1. **Geegah_RP2040_Imaging.py**: Acquisition of N number of frames with an option to visualize them in real-time for the entire 128 x 128 pixels. This operates at a fixed frequency and echo acquisition timing.
2. **Geegah_RP2040_FrequencySweep.py**: Acquisition of images at a range of frequencies (1.5 - 2.0 GHz, with a step as low as 0.01 MHz). Allows capturing N frames at a fixed acquisition echo timing.
3. **Geegah_RP2040_SNH.py**: Acquisition of images at different echo timings with a step of 5 ns. This allows capturing N frames at a fixed frequency. 



# Helper libraries
These consist of classes or functions that support the acquisition and processing of scripts. These do not have to be run individually.

1. **geegah_hp.py** Helper functions for image acquisition and post-processing

# Other files for driver support (included)

1. okFrontPanel.dll
2. _ok.pyd
3. xem7305_GG222.bit

# Getting started: 

**Clone the repository**
```bash
git clone git@github.com:Geegah-Inc/Geegah_OK.git
```

**Or, directly download the zip**
Click on 1) **Code** drop-down menu and click **Download ZIP** 
![alt text](https://github.com/Geegah-Inc/Geegah_OK/blob/main/ZIP_download.png)

**Usage Example**
All the scripts have been divided into the following sections:

1) Importing modules
2) Creating a directory and sub-folders to save raw and processed images
3) Changing parameters of interest
4) Finding and initializing the board
5) Reloading the board after first initialization
6) Acquiring air frames/background frames
7) Acquiring main sample frames
8) Plotting desired acoustic parameters
9) Relevant Post-processing

**1. Importing Modules**<br />
You simply have to run this once to load the necessary helper functions and Python libraries
```python
import fpga 
import sys
import numpy as np
import matplotlib.pyplot as plt
import geegah_hp
import time
import cv2 #optional for video generation
import os
```

**2. Directory assignment** <br />
Change the **foldernam**e to represent the main folder where all the sub-folders will be created, and the raw data files will be saved. <br />
Select the **savedirname** as the filepath where the **foldername** will be created

```python
foldername = "SampleExperiment4"
path = "C:/Users/anujb/Downloads"

savedirname = os.path.join(path, foldername, "")
```

**3. Changing parameters of interest**<br />
This is the section where you need to assign correct values to the parameter that you might want to change for the acquisition

```python

liveplot = True #boolean for plotting images real-time, True or False
frequency = 1853.5 #Pulse frequency in MHz, with resolution of 0.1 MHz

#Selection of firing/receiving pixels, ROI 
col_min = 0 #integer, 0<col_min<127
col_max = 127 #integer, 0<col_max<127
row_min = 0 #integer, 0<row_min<127
row_max = 127 #integer, 0<row_max<127

row_no = row_max - row_min
col_no = col_max - col_min
roi_param = [col_min, col_max, row_min, row_max]
num_Frames = 100 #Number of frames to acquire for sample, integer, num_Frames > 0

```
Please ensure the Geegah Imager is powered on and connected to the PC via USB A/C before proceeding.  <br />

**4. Finding and initializing the board**<br />
The first section of the code sets up the connection with the FPGA board. If the board is not connected, or if the driver is missing, an error message appears. 

```python
xem = fpga.fpga()
board_name = xem.BoardName()
if board_name != "XEM7305":
    print("Problem: board name = " + board_name)  
    sys.exit()
print("Board: " + xem.di.deviceID + " " + xem.di.serialNumber)
```
The second part of the code is loading the DAC in the board, which prepares individual pixels for imaging. It further loads other timing and pulse settings as well. <br /> 
You do not have to change anything here. You also only need to run this section once, as it takes approximately 30 seconds - 1.5 minutes to load all the pixels (128 x 128) <br />
Re-run this if you restarted the console or the board connection was interrupted mid-acquisition. 

```python
#bit file to use (before DAC changes)
bit_file_name = "xem7305.bit"
xem.Configure(bit_file_name) # use older bit file
print("Version: " + xem.Version() + " serial number " + str(xem.SerialNumber()))
print("Sys clock = %8.4f MHz" % xem.SysclkMHz())
...
...
...
#code continues#
```
**5. Reloading the board after first initialization** <br />
This function can be run to re-initialize the board once the FPGA setup code has already been run.

```python
geegah_hp.reload_board(xem, frequency, roi_param)
```

Please ensure the chip surface is cleaned properly before running this section.


**6. Acquiring baseline frames** <br />
Enter the number of air frames/baseline frames to acquire for the experiment by changing  **NAIRSAMPLES** variable. 

```python
NAIRSAMPLES = 10
N_ZERO_PAD = len(str(NAIRSAMPLES))
i_time = time.time()

for mycount in range(NAIRSAMPLES):
   
    geegah_hp.configTiming(xem,term_count,TX_SWITCH_EN_SETTINGS,PULSE_AND_SETTINGS,RX_SWITCH_EN_SETTINGS,GLOB_EN_SETTINGS,LO_CTRL_SETTINGS,ADC_CAP_SETTINGS)
    
    air_baseline_echo_filename = BLE_save_dir + "DATA"+str(mycount).zfill(N_ZERO_PAD)+".dat"
...
...
...
#code continues#
```
**7. Acquiring sample frames** <br />
Enter the number of sample frames to acquire for the experiment by changing  the **NUM_IMAGE_SAMPLES** variable. <br />
If you have enabled the plotting (liveplot = True), a plot window pops up displaying the calculated Magnitude (V) of the signal real time.
```python

NUM_IMAGE_SAMPLES =  20
N_ZERO_PAD_IM = len(str(NUM_IMAGE_SAMPLES))
time_stamp = []
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
Enter the details for loading previously saved data: Foldername, Path to saved data, ROI col/rows min/max values, # of airframes, and # of sample frames.
This will load all the echo/no-echo, I and Q values for Baseline and Sample frames and place them in their respective arrays. It will then compute the arrays for Magnitude, Phase, Reflection coefficient, and Acoustic Impedance.
```python

foldername_PP = "SampleExperiment4"
path_PP = "C:/Users/anujb/Downloads"
savedirname_PP = os.path.join(path, foldername, "")
BLE_save_dir = savedirname_PP+'rawdata_baseline_echo/'
BLNE_save_dir = savedirname_PP+'rawdata_baseline_no_echo/'
rawdata_save_dir = savedirname_PP+'rawdata_echo/'
rawdata_ne_save_dir = savedirname_PP+'rawdata_no_echo/'
#Selection of firing/receiving pixels, ROI
#THIS MUST MATCH WITH THE SAME VARIABLES THAT WERE USED DURING ACQUISITION
col_min = 0 #integer, 0<col_min<127
col_max = 127  #integer, 0<col_max<127
row_min = 0 #integer, 0<row_min<127
row_max = 127 #integer, 0<row_max<127

roi_param = [col_min, col_max, row_min, row_max]
num_AIR_Frames = 1 #Number of air/baseline frames to consider during computation
num_SAMPLE_Frames = 20 #Number of frames to measured sample frames

I_A_E, Q_A_E, I_A_NE, Q_A_NE = [],[],[],[]
I_S_E, Q_S_E, I_S_NE, Q_S_NE = [],[],[],[]
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


   

   


