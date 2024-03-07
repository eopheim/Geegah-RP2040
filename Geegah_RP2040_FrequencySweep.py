
"""
Created on Wed Oct 30 12:22:10 2023

@author: geegah
"""
#import modules

import geegah_hp
import os
import numpy as np
import time
import serial
import math

#%% Make directories  to save files in
foldername = "WATER DROP FSWEEP"
path = "C:/Users/anujb/Downloads"

savedirname = os.path.join(path, foldername, "")

if not os.path.exists(savedirname):
    os.makedirs(savedirname)

#folder to dump raw .dat files in 
rawdata_save_dir = savedirname + "rawdata/"
if not os.path.exists(rawdata_save_dir):
    os.makedirs(rawdata_save_dir)
#folder to dump baseline .dat files in 
basedata_save_dir = savedirname + "baseline/"
if not os.path.exists(basedata_save_dir):
    os.makedirs(basedata_save_dir)
#folder to store images in
img_save_dir = savedirname + "images/"
if not os.path.exists(img_save_dir):
    os.makedirs(img_save_dir)
#folder to store video in
vid_save_dir = savedirname + "video/"
if not os.path.exists(vid_save_dir):
    os.makedirs(vid_save_dir)

print("Done Setting Up Folders")
#%%IMAGING PARAMETERS
#Frequency sweep start, end, and delta
#Frequencies in MHz

f_start = 1700
f_end = 2000
f_delta = 5 #can be as low as 0.01
num_frames =  1 #CHANGE THIS TO ACQUIRE MULTIPLE FRAMES at the same frequency. 

#%% Pin assignments AND INITIAL SETUP
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
                    
# CLKn frequency for SPI in Hz
CLK_FREQ_SPI_PICO =int(8e6)
#INITIALIZE AND SET FREQUENCY
spi_obj_pico, n_bytes_block_arg = geegah_hp.settings_freqswitch_RP2040(frequency = 1850, pinDict_Main = pinDict_Main )

#SET TIME SETTINGS to acquire at echo time
ser = serial.Serial('/dev/ttyACM0', 115200)
geegah_hp.switch_RP2040time(ser, mode = 1, time = 125, adc = 1)
ser.close()

#%% Dummy frames to discard
N_dummy = 5 #Number of dummy frames to discard, must be at least 2 to clear the 2 RP2040 buffers
geegah_hp.dummy_frames(spi_obj_pico, n = N_dummy)

#%% ACQUIRE AIR FRAMES by SWEEPING FREQUENCY
print("\n\n\n AQUISITION OF AIR FRAMES NOW: CLEAN THE IMAGER NOW")
a=input("Press Enter to proceed with AIR frames acquisition \n")
for myf in range(f_start*100,f_end*100,math.floor(f_delta*100)):
    
    f_to_use = myf/100
    spi_obj_pico, n_bytes_block_arg = geegah_hp.settings_freqswitch_RP2040(frequency = f_to_use, pinDict_Main = pinDict_Main )
    
    geegah_hp.dummy_frames(spi_obj_pico, n = 2)
    frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = geegah_hp.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                       n_bytes_block_arg=n_bytes_block_arg)
    
    baseline_echo_fname = basedata_save_dir + "freq"+str(myf)+".dat"
    geegah_hp.writeFile(baseline_echo_fname, frames_data_nD_baseline)
    
    time.sleep(0.1)
    print("DONE acquiring air frame at frequency = "+str(f_to_use))

print("DONE ACQUIRING AIR FRAMES BY SWEEPING FREQUENCY")

#%% ACQUIRE SAMPLE FRAMES by SWEEPING FREQUENCY

print("\n\n\n AQUISITION OF SAMPLE FRAMES NOW: PLACE THE SAMPLE NOW")
a=input("Press Enter to proceed with sample frames acquisition \n")

for myf in range(f_start*100,f_end*100,math.floor(f_delta*100)):
    f_to_use = myf/100
    spi_obj_pico, n_bytes_block_arg = geegah_hp.settings_freqswitch_RP2040(frequency = f_to_use, pinDict_Main = pinDict_Main )
    
    geegah_hp.dummy_frames(spi_obj_pico, n = 2)
    frames_data_nD_sample,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = geegah_hp.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                       n_bytes_block_arg=n_bytes_block_arg)
    sample_echo_fname = rawdata_save_dir + "freq"+str(myf)+".dat"
    geegah_hp.writeFile(sample_echo_fname, frames_data_nD_sample)
    
    time.sleep(0.1)
    print("DONE acquiring sample frame at frequency = "+str(f_to_use))

print("DONE ACQUIRING SAMPLE FRAMES BY SWEEPING FREQUENCY")

#%% PROCESSING DATA


print("\n\n\nTHIS SECTION IS FOR POST PROCESSING:")
a=input("Press y to PROCEED with post processing, else press n")

if a == "n" or a == "N":
    print("Stopping the code now:")
    exit()
    
#CHANGE THE FOLLOWING VARIABLES IF LOADING AN OLD DATASET

foldername_PP = "Test"
path_PP = "C:/Users/anujb/Downloads"
savedirname_PP = os.path.join(path_PP, foldername_PP, "")

basedata_save_dir = savedirname_PP + "baseline/"
rawdata_save_dir = savedirname_PP+ "rawdata/"

#Frequency sweep start, end, and delta
#Frequencies in MHz
f_start = 1700
f_end = 2000
f_delta = 1 #can be as low as 0.01

#EXTRACTING AIR FREQUENCIES, frames
I_A_E = []#In phase, baseline, 
Q_A_E = []#Out of phase, baseline, Echo

MAG_A = []
PHASE_A = []

for myf in range(f_start*100,f_end*100,math.floor(f_delta*100)):
    
    AE_filename = basedata_save_dir + "freq"+str(myf)+".dat"
    I_ADC_AE, Q_ADC_AE, I_VOLTS_AE, Q_VOLTS_AE = geegah_hp.loadSavedRawDataRP2040(AE_filename)
    I_A_E.append(I_VOLTS_AE)
    Q_A_E.append(Q_VOLTS_AE)

    MAG_AIR = np.sqrt(np.square(I_VOLTS_AE)+np.square(Q_VOLTS_AE ))
    PHASE_AIR = np.arctan2(I_VOLTS_AE, Q_VOLTS_AE)
    MAG_A.append(MAG_AIR)
    PHASE_A.append(PHASE_AIR)

print("FINISHED LOADING AIR FREQUENCY DATA")

#EXTRACTING SAMPLE FREQUENCIES, frames
#THESE LISTS would contain lists represingting different frequencies
#Each frequency list would contain the N frames acquired

I_S_E = [] #In phase, sample, Echo
Q_S_E = []#Out of phase, sample, Echo

MAG_S= []
PHASE_S = []

for myf in range(f_start*100,f_end*100,math.floor(f_delta*100)):
    
    SE_filename = rawdata_save_dir + "freq"+str(myf)+".dat"
    I_ADC_SE, Q_ADC_SE, I_VOLTS_SE, Q_VOLTS_SE = geegah_hp.loadSavedRawDataRP2040(SE_filename)
    I_S_E.append(I_VOLTS_AE)
    Q_S_E.append(Q_VOLTS_AE)

    MAG_SAMPLE = np.sqrt(np.square(I_VOLTS_SE)+np.square(Q_VOLTS_SE))
    PHASE_SAMPLE = np.arctan2(I_VOLTS_SE, Q_VOLTS_SE)
    MAG_S.append(MAG_SAMPLE)
    PHASE_S.append(PHASE_SAMPLE)

print("FINISHED LOADING SAMPLE FREQUENCY DATA")


MAG_ADJ = np.array(MAG_S) - np.array(MAG_A)
PHASE_ADJ = np.array(PHASE_S) - np.array(PHASE_A)
#%%PLOTTING

LIST_to_PLOT = MAG_ADJ
foldername = "MAGNITUDE"
geegah_hp.imgvid_plot_fsweep(LIST_to_PLOT, savedirname, foldername, 
             start_freq = f_start, end_freq = f_end,
             step_freq = f_delta ,
             vmin = -0.1, vmax = 0.1)