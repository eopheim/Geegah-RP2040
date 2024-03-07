#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 30 12:22:10 2023

@author: geegah
"""
#import modules

import geegah_hp
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
import os
import numpy as np
import time
import serial
#%% Make directories  to save files in
foldername = "Test"
path = "C:/Users/anujb/Downloads"

savedirname = os.path.join(path, foldername, "")

if not os.path.exists(savedirname):
    os.makedirs(savedirname)
    
#folder to dump raw .dat files in for 1st acoustic echo data
rawdata_save_dir = savedirname + "rawdata_echo/"
if not os.path.exists(rawdata_save_dir):
    os.makedirs(rawdata_save_dir)

#folder to dump raw .dat files in for no echo: AIR/SAMPLE
BLNE_save_dir = savedirname + "rawdata_baseline_noecho/"
if not os.path.exists(BLNE_save_dir):
    os.makedirs(BLNE_save_dir)

#folder to store baseline with echo files in 
BLE_save_dir = savedirname + "rawdata_baseline_echo/"
if not os.path.exists(BLE_save_dir):
    os.makedirs(BLE_save_dir)

#folder to store images in
img_save_dir = savedirname + "images/"
if not os.path.exists(img_save_dir):
    os.makedirs(img_save_dir)

#folder to store video in
vid_save_dir = savedirname + "video/"
if not os.path.exists(vid_save_dir):
    os.makedirs(vid_save_dir)

print("Done Setting Up Folders")

#%% Parameter selections
liveplot = True #boolean for plotting images real-time, True or False, set this as True for live plotting
frequency = 1853 #Pulse frequency in MHz, with resolution of 0.1 MHz

#%% Pin assignments AND INITIAL SETUP
#BOARD SETUP AND INITIALIZATION
print("INITIALIZING THE BOARD SETTINGS")
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

# SEtup the GPIO
geegah_hp.setup_GPIO(pinDict_arg = pinDict_Main)

#Create the sPI object for VCO
# SPI-0, DEV-1, with 
# Get VCO SPi object
spi_obj_vco = geegah_hp.get_spi_vco()

# VCO settings and registers
freq_target_MHz = 1853
OUTEN_user = 1
PSET_user = 3
regs=geegah_hp.calc_vco_reg_values(freq_target_MHz,OUTEN_user ,PSET_user)

regshx=[]
for i in range(len(regs)):
    regshx.append("{:#010x}".format(regs[i]))
print("The registers are:", regshx)


# Create list from regs
 # This is basically same as regbytesbyte
regs_list_of_ints  = list(map(geegah_hp.int2bytes, regs))

#Start writing from the re5 down to reg0
for i in range(len(regs)-1,-1,-1):
    regToWrite_list =regs_list_of_ints[i]
    
    print("writing register",i,regToWrite_list) 
    # Because we connected to LE to CE pin of the pi, and keep CE of the VCO on the board floating
    # LE pin needs to be lowered and made high to load the registers
    #GPIO.output(GPIO_NUM_VCO_LE, 0)
    spi_obj_vco.writebytes(regToWrite_list)
    #GPIO.output(GPIO_NUM_VCO_LE, 1)

# Setup SPI for DAC MAXIM 5252, Bus0, DEV 0 but with dedicated CS GPIO pin at GPIO_NUM_DAC_CE0B
DAC_val = 3815

spi_obj_dac = geegah_hp.get_spi_dac_MAXIM5123()
geegah_hp.writeDAC_MAXIM5123(spi_obj_dac, DAC_val, GPIO_NUM_DAC_CE0B)
# GET PICO SPI object
spi_obj_pico = geegah_hp.get_spi_pico(CLK_FREQ_SPI_PICO)
# Block read Size
n_bytes_block_arg=1<<12

#TIME SETTINGS

#finds the appropriate port of rpi2040 and assigns it to ser
#currently the device's port is '/dev/ttyACM0'
#make sure that the IMAGER BOARD (rpi2040) is plugged in and "ON"
#MODE 1 refers to acquisition at 1 sample and hold time
#timing refers to time elapsed after receive is turned on (the same time when the pulsing ends) in nano seconds
#Mode 0 would alternate between echo time (timing) and timing = 450ns
#Mode 2 would acquire the echo time (timing) first and capture consecutive frames at 5ns timing increments until 450ns
#                                                                                               
ser = serial.Serial('/dev/ttyACM0', 115200)
geegah_hp.switch_RP2040time(ser, mode = 1, time = 450, adc = 1)

#%% Dummy frames to discard
N_dummy = 5 #Number of dummy frames to discard, must be at least 2 to clear the 2 RP2040 buffers
geegah_hp.dummy_frames(spi_obj_pico, n = N_dummy)

#%%
#CAPTURE BASELINE FRAMES FIRST: 
#NOECHO frames capture first, time switch to echo, then echo baseline capture
#EVERY time time is switched, acquire few dummy frames toclear the buffer

print("\n\n\n AQUISITION OF AIR FRAMES NOW: CLEAN THE IMAGER SURFACE")
a=input("Press Enter to proceed with air frames acquisition \n")

AIR_frames = 10

for jj in range(AIR_frames):
    frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = geegah_hp.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                       n_bytes_block_arg=n_bytes_block_arg)
    #write AIR ECHO DAT FILES
    baseline_noecho_fname = BLNE_save_dir+"frame"+str(jj)+".dat"
    geegah_hp.writeFile(baseline_noecho_fname, frames_data_nD_baseline)
    _,_,I_ANE_VOLTS, Q_ANE_VOLTS = geegah_hp.loadRawByteDataRP2040(frames_data_nD_baseline)
    time.sleep(0.01)
    print("DONE no-echo frame = "+str(jj))
    
print("DONE ACQUIRING BASELINE I NO-ECHO AND Q NO-ECHO")

#SWITCH THE TIME TO ECHO ACQUISITION
geegah_hp.switch_RP2040time(ser, mode = 1, time = 125, adc = 1)
ser.Close()
# Dummy frames to discard
N_dummy = 3 #Number of dummy frames to discard, must be at least 2 to clear the 2 RP2040 buffers
geegah_hp.dummy_frames(spi_obj_pico, n = N_dummy)
#CAPTURE BASELINE ECHO 
# Save the Baseline Frames  

time_i = time.time()
for jj in range(AIR_frames):
    frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = geegah_hp.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                       n_bytes_block_arg=n_bytes_block_arg)
    #write AIR ECHO DAT FILES
    baseline_file_name = BLE_save_dir+"frame"+str(jj)+".dat"
    geegah_hp.writeFile(baseline_file_name, frames_data_nD_baseline)
    _,_,I_AE_VOLTS, Q_AE_VOLTS = geegah_hp.loadRawByteDataRP2040(frames_data_nD_baseline)
    MAG_AIR  = np.sqrt(np.square(I_AE_VOLTS - I_ANE_VOLTS)+np.square(Q_AE_VOLTS - Q_ANE_VOLTS))
    PHASE_AIR = np.arctan2(I_AE_VOLTS - I_ANE_VOLTS, Q_AE_VOLTS - Q_ANE_VOLTS)
    time.sleep(0.001)
    print("DONE frame = "+str(jj))
    
time_f = time.time()
del_time = time_f - time_i
fps = AIR_frames/del_time
print("average fps = " + str(fps))
print("DONE ACQUIRING BASELINE I AND Q ECHO")

#%%REAL TIME ACQUISITION
#PLOTS MAGNITUDE OF SAMPLE - MAGNITUDE OF AIR
SAMPLE_frames = 100
#128x128 pixels displays very small with OpenCV
#want to rescale up to a larger image size of 512 x 512 pixels 

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
#time track
time_i = time.time()
for jj in range(SAMPLE_frames):
    #read an image
    frames_data_nD_sample,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = geegah_hp.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                           n_bytes_block_arg=n_bytes_block_arg)
    
    #write AIR ECHO DAT FILES
    sampple_file_name = rawdata_save_dir+"frame"+str(jj)+".dat"
    geegah_hp.writeFile(sampple_file_name, frames_data_nD_sample)
    if liveplot == True:
   
        _,_,I_SE_VOLTS, Q_SE_VOLTS = geegah_hp.loadRawByteDataRP2040(frames_data_nD_sample)
        MAG_SAMPLE = np.sqrt(np.square(I_SE_VOLTS - I_ANE_VOLTS)+np.square(Q_SE_VOLTS - I_ANE_VOLTS))
        PHASE_SAMPLE = np.arctan2(I_SE_VOLTS - I_ANE_VOLTS, Q_SE_VOLTS - Q_ANE_VOLTS)
         
        MAG = MAG_SAMPLE - MAG_AIR
        PHASE = PHASE_SAMPLE - PHASE_SAMPLE
        RCOEF = MAG_SAMPLE/MAG_AIR
        
        #plot MAG
        
        mytitle.set_text(base_title+str(jj)+' of ' +str(SAMPLE_frames-1))
        sub_image = np.flipud(np.rot90(MAG,1))
        pos201.set_data(MAG)
        #pos201.set_clim(0.1,0.2)
        print(np.median(MAG))
        pos201.set_clim(sub_image.min(), sub_image.max())
        #redraw
        fig2.canvas.draw_idle()
        fig2.savefig(img_save_dir+'plot'+str(jj)+'.png')
    plt.pause(0.001)
    
time_f = time.time()
del_time = time_f - time_i
fps = SAMPLE_frames/del_time
print("average fps = " + str(fps))
print("DONE ACQUIRING SAMPLE I AND Q ECHO")

#%%CLOSE EVERYTHING
plt.close("all")
spi_obj_vco.close()  #Close the SPI object
spi_obj_pico.close() # Close the SPI object
spi_obj_dac.close()
GPIO.setwarnings(True)
GPIO.cleanup()
print("Done")
#%%###############################################################%%
#POST-PROCESSING SECTION
#LOADING FRAMES

print("\n\n\nTHIS SECTION IS FOR POST PROCESSING:")
a=input("Press y to PROCEED with post processing, else press n")

if a == "n" or a == "N":
    print("Stopping the code now:")
    exit()
    
foldername_PP = "Test"
path_PP = "C:/Users/anujb/Downloads"
savedirname_PP = os.path.join(path_PP, foldername_PP, "")

BLE_save_dir = savedirname_PP+'rawdata_baseline_echo/'
BLNE_save_dir = savedirname_PP+'rawdata_baseline_no_echo/'
rawdata_save_dir = savedirname_PP+'rawdata_echo/'

num_AIR_Frames = 10 #Number of air/baseline frames to consider during computation
num_SAMPLE_Frames = 100 #Number of frames to measured sample frames

I_A_E, Q_A_E, I_A_NE, Q_A_NE = [],[],[],[]
I_S_E, Q_S_E = [],[]

#LOAD AIR DATA
for frame in range(num_AIR_Frames):
   
    AE_filename = BLE_save_dir + "DATA"+str(frame)+".dat"
    ANE_filename =  BLNE_save_dir + "DATA"+str(frame)+".dat"
    
    I_ADC_AE, Q_ADC_AE, I_VOLTS_AE, Q_VOLTS_AE = geegah_hp.loadSavedRawDataRP2040(AE_filename)
    I_ADC_ANE, Q_ADC_ANE, I_VOLTS_ANE, Q_VOLTS_ANE = geegah_hp.loadSavedRawDataRP2040(ANE_filename)
    
    I_A_E.append(I_VOLTS_AE)
    Q_A_E.append(Q_VOLTS_AE)
    I_A_NE.append(I_VOLTS_ANE)
    Q_A_NE.append(Q_VOLTS_ANE)
    
    print("Currently loading AIR FRAME: "+str(frame))
print("FINISHED LOADING AIR FRAMES")

#LOADING SAMPLE DATA
for frame in range(num_SAMPLE_Frames):
    
    SE_filename = rawdata_save_dir+"DATA"+str(frame)+".dat"
    I_ADC_SE, Q_ADC_SE, I_VOLTS_SE, Q_VOLTS_SE = geegah_hp.loadSavedRawDataRP2040(SE_filename)
    I_S_E.append(I_VOLTS_SE)
    Q_S_E.append(Q_VOLTS_SE)
    
    print("Currently loading SAMPLE FRAME: "+str(frame))
print("FINISHED LOADING SAMPLE FRAMES")

#NUMPY ARRAY CONVERSION
I_A_E, Q_A_E, I_A_NE, Q_A_NE = np.array(I_A_E),np.array(Q_A_E),np.array(I_A_NE),np.array(Q_A_NE)
I_S_E, Q_S_E= np.array(I_S_E),np.array(Q_S_E)

#AVERAGING THE BASELINE ECHO AND NO_ECHO TO CALCULATE MAGNITUDE AND PHASE
I_A_E_av = np.average(I_A_E, axis = 0)
Q_A_E_av = np.average(Q_A_E, axis = 0)
I_A_NE_av = np.average(I_A_NE, axis = 0)
Q_A_NE_av = np.average(Q_A_NE, axis = 0)
#COMPUTING MAGNITUDE, PHASE, REFLECTION COEFFICIENT, and ACOUSTIC IMPEDANCE

MAG_AIR = np.sqrt(np.square(I_A_E_av - I_A_NE_av)+np.square(Q_A_E_av - Q_A_NE_av))
PHASE_AIR = np.arctan2(I_A_E_av-I_A_NE_av, Q_A_E_av-Q_A_NE_av)

MAG_SAMPLE = []
PHASE_SAMPLE = []
RCOEF = []

for frame in range(num_SAMPLE_Frames):
    MAG = np.sqrt(np.square(I_S_E[frame] - I_A_NE_av)+np.square(Q_S_E[frame] - Q_A_NE_av))
    PHASE = np.arctan2(I_S_E[frame]-I_A_NE_av, Q_S_E[frame]- I_A_NE_av)
    MAG_SAMPLE.append(MAG)
    PHASE_SAMPLE.append(PHASE)
    RCOEF.append(MAG/MAG_AIR)
    
#ACOUSTIC PARAMETERS COMPUTATION
MAGNITUDE_ADJ = np.array(MAG_SAMPLE)-np.array(MAG_AIR)
PHASE_ADJ = np.array(PHASE_SAMPLE) - np.array(PHASE_AIR)
REFLECTION_COEFFICIENT = np.array(MAG_SAMPLE)/MAG_AIR

ACOUSTIC_IMP = [geegah_hp.impedance_si(jj, array = True) for jj in REFLECTION_COEFFICIENT]

#%%PLOTTING 

LIST_to_PLOT = REFLECTION_COEFFICIENT
Parameter = "Reflection coefficient"
geegah_hp.imgvid_plot_IMG(LIST_to_PLOT,savedirname_PP, 
                          foldername = Parameter,
                          vmin = 0.6,vmax = 1.2)
plt.colorbar()

#%%
LIST_to_PLOT = PHASE_ADJ
Parameter = "Phase (radians)"
geegah_hp.imgvid_plot_IMG(LIST_to_PLOT,savedirname_PP, 
                          foldername = Parameter,
                          vmin = -0.2,vmax = 0.2)
plt.colorbar()

#%%
LIST_to_PLOT = MAGNITUDE_ADJ
Parameter = "Magnitude (V)"
geegah_hp.imgvid_plot_IMG(LIST_to_PLOT,savedirname_PP, 
                          foldername = Parameter,
                          vmin = -0.15,vmax = 0.08)
plt.colorbar()