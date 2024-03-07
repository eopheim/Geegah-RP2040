# -*- coding: utf-8 -*-
"""
Created on Tue Apr 13 20:59:26 2021

@author: Justin
"""

#ACQUISITION FUNCTIONS
def acqSingleFrame_FSWEEP(xem, ADCNUM, file_name,):
    import math
    
    xem.Open()
    xem.SelectADC(ADCNUM)
    xem.SelectFakeADC(0)
    xem.EnablePgen(0)
    xem.ResetFifo()
    xem.EnablePipeTransfer(1)
    xem.StartAcq()
    
    # Set the array size to match the data, but make it a multiple of 1024
    nbytes = 128*128*2*2
    nbytes = 1024 * math.ceil(nbytes/1024)
    byte_data = bytearray(nbytes)
    nbytes = xem.GetPipeData(byte_data)
    
    #print ("GetPipeData returned ", nbytes)
    f = open(file_name, "wb")
    f.write(byte_data)
    f.close()
    #print("Wrote data to roi.dat")
    xem.Close()
    return byte_data

def acqSingleFrameROI(xem, ADCNUM, file_name, c1 = 0, c2 = 127, c3 = 0, c4 = 127):
    import math
    
    xem.Open()
    xem.SelectADC(ADCNUM)
    xem.SelectFakeADC(0)
    xem.EnablePgen(0)
    xem.ResetFifo()
    xem.EnablePipeTransfer(1)
    xem.StartAcq()
    
    # Set the array size to match the data, but make it a multiple of 1024
    #print("1")
    nbytes = ((c2 - c1 + 1) * (c4 - c3 + 1))*2*2
    nbytes = 1024 * math.ceil(nbytes/1024)
    byte_data = bytearray(nbytes)
    nbytes = xem.GetPipeData(byte_data)
    #print("3")
    #print ("GetPipeData returned ", nbytes)
    f = open(file_name, "wb")
    f.write(byte_data)
    f.close()
    #print("Wrote data to roi.dat")
    #print("5")
    xem.Close()
    return byte_data

def acqSingleFrameCAL(xem, ADCNUM):
  
    xem.Open()
    xem.SelectADC(ADCNUM)
    xem.SelectFakeADC(0)
    xem.EnablePgen(0)
    xem.ResetFifo()
    xem.EnablePipeTransfer(1)
    xem.StartAcq()

    nbytes = 128*128*2*2
    byte_data = bytearray(nbytes)
    nbytes = xem.GetPipeData(byte_data)
    xem.Close()
    return byte_data


#SETTINGS FUNCTIONS

def reload_board(xem, frequency, roi_param):
    xem.Open()
    freq = frequency
    OUTEN = 1
    PSET =3 
    xem.SetROI(roi_param[0],roi_param[1],roi_param[2],roi_param[3])
    configureVCO(xem,freq,OUTEN,PSET)
    xem.Close()

def writeFile(file_name, byte_data):
    
    with open(file_name, "wb") as f:
        f.write(byte_data)
        
def switch_RP2040time(serial_main, mode, time, adc):
  
    mode = mode
    adc = adc
    timing = time #NO_ECHO ACQUISITION FIRST
    packet = str(str(mode)+'.'+str(timing)+'.'+str(adc)+'\n')
    print(packet)
    serial_main.write(packet.encode())

def loadRawByteDataRP2040(byte_data):
  
    I_RAW, Q_RAW = convertToIQImageRP(byte_data)
    I_ADC, Q_ADC, I_VOLTS, Q_VOLTS = convertADCToVoltsRP(I_RAW, Q_RAW)
    return I_ADC, Q_ADC, I_VOLTS, Q_VOLTS

def loadSavedRawDataRP2040(file_name):
    f = open(file_name, 'rb')
    MYDAT = f.read()
    f.close()
    I_RAW, Q_RAW = convertToIQImageRP(MYDAT)
    I_ADC, Q_ADC, I_VOLTS, Q_VOLTS = convertADCToVoltsRP(I_RAW, Q_RAW)
    return I_ADC, Q_ADC, I_VOLTS, Q_VOLTS

def impedance_si(ref_coef, array = True):
    
    csi = 8433
    psi = 2329
    zsi = csi*psi
    if array == False:
        zsamp = (zsi*(1-ref_coef))/(1+ref_coef)
        return zsamp/1e6
    if array ==True:
        new_list = []
        for jj in ref_coef:
            zsamp = (zsi*(1-jj))/(1+jj)
            new_list.append(zsamp/1e6)
        return new_list

def convertToIQImageRP(byte_data):
    import numpy as np
    wi = 0

    imgBytesI = np.zeros(128*128)
    imgBytesQ = np.zeros(128*128)
    for row in range (128):
        for col in range(128):
            wi = row*128 + col
            iwrd = (byte_data[4 * wi + 1] + 256*byte_data[4 * wi + 0]) #swap +0 and +1
            qwrd = (byte_data[4 * wi + 3] + 256*byte_data[4 * wi + 2]) #swap +2 and +3
            imgBytesI[wi] = iwrd
            imgBytesQ[wi] = qwrd
            
    IMG_I=imgBytesI.reshape([128,128])
    IMG_Q=imgBytesQ.reshape([128,128])
    return IMG_I, IMG_Q

def loadSavedRawDataROI(file_name, c1 = 0, c2 = 127, r1 = 0, r2 = 127):
    import numpy as np
    f = open(file_name, 'rb')
    MYDAT = f.read()
    f.close()
    rows = r2 - r1+1
    cols = c2 - c1+1
    imgBytesI = np.zeros(rows*cols)
    imgBytesQ = np.zeros(rows*cols)
    
    for row in range(rows):
        for col in range(cols):
            wi = row*cols + col  # Correction: use cols instead of rows
            iwrd = (MYDAT[4 * wi + 0] + 256*MYDAT[4 * wi + 1])
            qwrd = (MYDAT[4 * wi + 2] + 256*MYDAT[4 * wi + 3])
            imgBytesI[wi] = iwrd
            imgBytesQ[wi] = qwrd
            
    J_MYIMAGE_I = imgBytesI.reshape([rows, cols])
    J_MYIMAGE_Q = imgBytesQ.reshape([rows, cols])
    I_IMAGE_ADC = J_MYIMAGE_I / 16  # correct bit shift
    Q_IMAGE_ADC = J_MYIMAGE_Q / 16  # correct bit shift
    I_IMAGE_VOLTS = I_IMAGE_ADC * 1e-3  # convert to volts
    Q_IMAGE_VOLTS = Q_IMAGE_ADC * 1e-3  # convert to volts
    return I_IMAGE_ADC, Q_IMAGE_ADC, I_IMAGE_VOLTS, Q_IMAGE_VOLTS

    
def loadSavedRawDataFromBytes(bytes1):
    import numpy as np

    rows = 128
    cols = 128
    imgBytesI = np.zeros(rows*cols)
    imgBytesQ = np.zeros(rows*cols)
    
    for row in range(rows):
        for col in range(cols):
            wi = row*cols + col  # Correction: use cols instead of rows
            iwrd = (bytes1[4 * wi + 0] + 256*bytes1[4 * wi + 1])
            qwrd = (bytes1[4 * wi + 2] + 256*bytes1[4 * wi + 3])
            imgBytesI[wi] = iwrd
            imgBytesQ[wi] = qwrd
            
    J_MYIMAGE_I = imgBytesI.reshape([rows, cols])
    J_MYIMAGE_Q = imgBytesQ.reshape([rows, cols])
    I_IMAGE_ADC = J_MYIMAGE_I / 16  # correct bit shift
    Q_IMAGE_ADC = J_MYIMAGE_Q / 16  # correct bit shift
    I_IMAGE_VOLTS = I_IMAGE_ADC * 1e-3  # convert to volts
    Q_IMAGE_VOLTS = Q_IMAGE_ADC * 1e-3  # convert to volts
    return I_IMAGE_ADC, Q_IMAGE_ADC, I_IMAGE_VOLTS, Q_IMAGE_VOLTS



#save settings_file
def saveSettingsFile(savedirname,bit_file_name,freq,OUTEN,PSET,term_count,TX_SWITCH_EN_SETTINGS,PULSE_AND_SETTINGS,RX_SWITCH_EN_SETTINGS,GLOB_EN_SETTINGS,LO_CTRL_SETTINGS,ADC_CAP_SETTINGS,DAC_VOLTAGE,ADC_TO_USE):
    
    from datetime import datetime
    now = datetime.now()
    dt_string = now.strftime("%m/%d/%Y %H:%M:%S")
    
    settings_file_name = savedirname + "settings.txt"
    
    # write to file
    f = open(settings_file_name, "w")
    f.write("date M/D/Y and time H/M/S = " + dt_string +"\n")
    
    f.write("Bit File Name: "+ bit_file_name +"\n") #write bit file name
    f.write("Frequency: "+str(freq)+" MHz"+"\n") #Frequency
    f.write("OUTEN: "+ str(OUTEN)+"\n")
    f.write("PSET: " + str(PSET)+"\n")
    f.write("Term Count: " + str(term_count)+"\n")
    f.write("TX_SWITCH_EN Settings: "+str(TX_SWITCH_EN_SETTINGS)+"\n")
    f.write("PULSE_AND Settings: " + str(PULSE_AND_SETTINGS)+"\n")
    f.write("RX_SWITCH_EN Settings: " + str(RX_SWITCH_EN_SETTINGS)+"\n")
    f.write("GLOB_EN Settings: " + str(GLOB_EN_SETTINGS)+"\n")
    f.write("LO_CTRL Settings: " + str(LO_CTRL_SETTINGS)+"\n")
    f.write("ADC_CAP Settings: " + str(ADC_CAP_SETTINGS)+"\n")
    f.write("DAC Voltage for All Pixels: " + str(DAC_VOLTAGE)+"\n")
    f.write("ADC Used: " + str(ADC_TO_USE)+"\n")
    
    f.close()
    print("Wrote data to " + settings_file_name)


#set up DAC to be the same for all pixels
def setAllPixSameDAC(xem,DAC_VOLTAGE):
    myDACVal = convertVoltToDAC(DAC_VOLTAGE)
    for row in range(128):
        print("loading DAC ROW (",row,") of 128")
        for col in range(128):
            for i_or_q in range(2):
                #print("loading DAC entry (",row,col,i_or_q,")") #if you want to print per pixel
                xem.LoadDACEntry(row, col, i_or_q, myDACVal) 
    print("Done loading DAC table")

#configure timing registers
def configTiming(xem,term_count,TX_SWITCH_EN_SETTINGS,PULSE_AND_SETTINGS,RX_SWITCH_EN_SETTINGS,GLOB_EN_SETTINGS,LO_CTRL_SETTINGS,ADC_CAP_SETTINGS):
    xem.Open()
    xem.SetTerminalCount(term_count) 
    xem.SetTiming(*((0,)+TX_SWITCH_EN_SETTINGS))   # TX_SWITCH_EN
    xem.SetTiming(*((1,)+PULSE_AND_SETTINGS) )   # PULSE_AND
    xem.SetTiming(*((2,)+RX_SWITCH_EN_SETTINGS))    # RX_SWITCH_EN
    xem.SetTiming(*((3,)+GLOB_EN_SETTINGS))    # GLOB_EN
    xem.SetTiming(*((4,)+LO_CTRL_SETTINGS))     # LO_CTRL
    xem.SetTiming(*((5,)+ADC_CAP_SETTINGS))  # ADC_CAPTURE #80 81
    xem.Close()

"""

freq ==> frequency in MHz --> resolution of 0.1MHz
OUTEN--> output enable-->0 for disable, 1 for enable
PSET --> select RF output power

power settings for PSET
0 --> Enabled, -4 dbm
1 --> Enabled, -1 dbm
2 --> Enabled, +2 dbm
3 --> Enabled, +5 dbm
don't use 2 or 3 unless really needed
"""
#configure VCO
def configureVCO(xem,freq,OUTEN,PSET):
    R0, R1, R2, R3, R4, R5 = calc_vco_reg_values(freq,OUTEN,PSET)
    #configure VCO registers on FPGA
    setXEMVCORegs(xem, R0, R1, R2, R3, R4, R5)

#configure VCO
def configureVCO_10khz(xem,freq,OUTEN,PSET):
    R0, R1, R2, R3, R4, R5 = calc_vco_reg_values_10khz(freq,OUTEN,PSET)
    #configure VCO registers on FPGA
    setXEMVCORegs(xem, R0, R1, R2, R3, R4, R5)
    
def configureVCO_10khz_fsweep(xem,freq,OUTEN,PSET):
    xem.Open()
    R0, R1, R2, R3, R4, R5 = calc_vco_reg_values_10khz(freq,OUTEN,PSET)
    #configure VCO registers on FPGA
    setXEMVCORegs(xem, R0, R1, R2, R3, R4, R5)
    xem.Close()
#convert raw ADC data to bit-shift corrected ADC data and convert to voltage
def convertADCToVolts(I_IMAGE, Q_IMAGE):
    I_IMAGE_ADC = I_IMAGE/16 #correct bit shift
    Q_IMAGE_ADC = Q_IMAGE/16 #correct bit shift
    I_IMAGE_VOLTS = I_IMAGE_ADC*1e-3 #convert to volts
    Q_IMAGE_VOLTS = Q_IMAGE_ADC*1e-3 #convert to volts
    return I_IMAGE_ADC, Q_IMAGE_ADC, I_IMAGE_VOLTS, Q_IMAGE_VOLTS


#convert raw ADC data to bit-shift corrected ADC data and convert to voltage
def convertADCToVoltsRP(I_IMAGE, Q_IMAGE):
    I_IMAGE_ADC = I_IMAGE/1 #correct bit shift
    Q_IMAGE_ADC = Q_IMAGE/1 #correct bit shift
    I_IMAGE_VOLTS = I_IMAGE_ADC*1e-3 #convert to volts
    Q_IMAGE_VOLTS = Q_IMAGE_ADC*1e-3 #convert to volts
    return I_IMAGE_ADC, Q_IMAGE_ADC, I_IMAGE_VOLTS, Q_IMAGE_VOLTS

#convert voltage to DAC value (for xem7305.bit)
def convertVoltToDAC(myVolt):
    import math
    myDACVal = (myVolt-2.9622)*(512/(-0.33))
    myDACVal = math.floor(myDACVal)
    return myDACVal

#set VCO registers
def setXEMVCORegs(xem, R0, R1, R2, R3, R4, R5 ):
    xem.SetRegField(xem.spi_wdata, R5)
    xem.SetRegField(xem.vco_spi_go, 1)
    xem.SetRegField(xem.spi_wdata, R4)
    xem.SetRegField(xem.vco_spi_go, 1)
    xem.SetRegField(xem.spi_wdata, R3)
    xem.SetRegField(xem.vco_spi_go, 1)
    xem.SetRegField(xem.spi_wdata, R2)
    xem.SetRegField(xem.vco_spi_go, 1)
    xem.SetRegField(xem.spi_wdata, R1)
    xem.SetRegField(xem.vco_spi_go, 1)
    xem.SetRegField(xem.spi_wdata, R0)
    xem.SetRegField(xem.vco_spi_go, 1)


"""

freq_des ==> frequency in MHz --> resolution of 0.1MHz
OUTEN--> output enable-->0 for disable, 1 for enable
PSET --> select RF output power

power settings for PSET
0 --> Enabled, -4 dbm
1 --> Enabled, -1 dbm
2 --> Enabled, +2 dbm
3 --> Enabled, +5 dbm
don't use 2 or 3 unless really needed
"""

def calc_vco_reg_values(freq_des_user,OUTEN_user,PSET_user):
    import math
    
    freq_des = freq_des_user
    OUTEN = OUTEN_user
    PSET = PSET_user
    #keep OUTEN in bounds
    if (OUTEN > 1):
        OUTEN = 0
    #keep PSET in bounds
    if (PSET > 3):
        PSET = 0
    
    #this is fixed, not user specifiable
    freq_res = 0.1 # in MHz
    
    #make sure it is in the correct resolution
    freq_actual = freq_res * round(freq_des / freq_res)
    
    #calculate divider setting
    myDIVSEL = math.ceil(math.log(math.ceil(2200/freq_actual))/math.log(2))
    myDIV = 2**(myDIVSEL);
    
    #reference oscillator
    refin = 10 # in MHz
    #VCO PFD (phase frequency detector) frequency
    myPFD = 10 # in MHz
    
    mylumped = freq_actual*myDIV/myPFD #N in the ADI software
    
    myINT = math.floor(mylumped)
    
    #mod goes from 2 to 4095
    #frac goes from 0 to mod-1
    #if 0 for frac, use 2 for mod
    
    #MOD = Refin/Fres
    #fdes_dec = freq_actual - math.floor(freq_actual)
    if ((mylumped - myINT) == 0):
        myMOD = 2
        myFRAC = 0
    else:
        myMOD = refin/freq_res
        myFRAC = myMOD * (mylumped-math.floor(mylumped))
        myMOD = round(myMOD) #if some decimal stuff left
        myFRAC = round(myFRAC)
        
        while(math.gcd(myMOD,myFRAC)>1):
            mygcd = math.gcd(myMOD,myFRAC)
            myMOD = round(myMOD/mygcd)
            myFRAC = round(myFRAC/mygcd)
        
    #structure of register 0
    #0 at db31, 16 bit INT, 12 bit FRAC, 3 control bits 000
    reg0 = (myFRAC*(2**3)) + (myINT*(2**15))
    #structure of register 1
    reg1 = 0x8008001  + (myMOD * (2**3))
    
    #calculate reg4
    reg4 = (2**23) #fundamental feedback select
    reg4 = reg4 + (myDIVSEL*(2**20)) #divider select
    reg4 = reg4 + (2**2) #control bits
    reg4 = reg4 + (PSET*(2**3)) #power setting
    reg4 = reg4 + (OUTEN*(2**5)) #output enable setting
    reg4 = reg4 + ((80)*(2**12)) #band select clock divider value
    #registers that are unchanged
    reg2 = 0x4E42
    reg3 = 0x4B3
    reg5 = 0x580005

    return reg0, reg1, reg2, reg3, reg4, reg5

def calc_vco_reg_values_10khz(freq_des_user,OUTEN_user,PSET_user):
    import math
    
    freq_des = freq_des_user
    OUTEN = OUTEN_user
    PSET = PSET_user
    #keep OUTEN in bounds
    if (OUTEN > 1):
        OUTEN = 0
    #keep PSET in bounds
    if (PSET > 3):
        PSET = 0
    
    #this is fixed, not user specifiable
    freq_res = 0.01 # in MHz
    
    #make sure it is in the correct resolution
    freq_actual = freq_res * round(freq_des / freq_res)
    
    #calculate divider setting
    myDIVSEL = math.ceil(math.log(math.ceil(2200/freq_actual))/math.log(2))
    myDIV = 2**(myDIVSEL);
    
    #reference oscillator
    refin = 10 # in MHz
    #VCO PFD (phase frequency detector) frequency
    myPFD = 10 # in MHz
    
    mylumped = freq_actual*myDIV/myPFD #N in the ADI software
    
    myINT = math.floor(mylumped)
    
    #mod goes from 2 to 4095
    #frac goes from 0 to mod-1
    #if 0 for frac, use 2 for mod
    
    #MOD = Refin/Fres
    #fdes_dec = freq_actual - math.floor(freq_actual)
    if ((mylumped - myINT) == 0):
        myMOD = 2
        myFRAC = 0
    else:
        myMOD = refin/freq_res
        myFRAC = myMOD * (mylumped-math.floor(mylumped))
        myMOD = round(myMOD) #if some decimal stuff left
        myFRAC = round(myFRAC)
        
        while(math.gcd(myMOD,myFRAC)>1):
            mygcd = math.gcd(myMOD,myFRAC)
            myMOD = round(myMOD/mygcd)
            myFRAC = round(myFRAC/mygcd)
        
    #structure of register 0
    #0 at db31, 16 bit INT, 12 bit FRAC, 3 control bits 000
    reg0 = (myFRAC*(2**3)) + (myINT*(2**15))
    #structure of register 1
    reg1 = 0x8008001  + (myMOD * (2**3))
    
    #calculate reg4
    reg4 = (2**23) #fundamental feedback select
    reg4 = reg4 + (myDIVSEL*(2**20)) #divider select
    reg4 = reg4 + (2**2) #control bits
    reg4 = reg4 + (PSET*(2**3)) #power setting
    reg4 = reg4 + (OUTEN*(2**5)) #output enable setting
    reg4 = reg4 + ((80)*(2**12)) #band select clock divider value
    #registers that are unchanged
    reg2 = 0x4E42
    reg3 = 0x4B3
    reg5 = 0x580005

    return reg0, reg1, reg2, reg3, reg4, reg5


#ACQISITION FOR RP2040



def setup_GPIO(pinDict_arg):
    import RPi.GPIO as GPIO
   
    GPIO.setmode(GPIO.BCM) #address the GPIOs via their GPIO number
    GPIO.setup(pinDict_arg["gpio_num_PINNO_TEST"], GPIO.OUT)
    GPIO.setup(pinDict_arg["gpio_num_DAC_CE0B"], GPIO.OUT)
    #GPIO.setup(pinDict_arg["gpio_num_VCO_LE"], GPIO.OUT) #LE for VCO

    print('Finished setting up GPIO')
    
    
def get_spi_pico(CLK_FREQ_SPI_PICO):  
    import spidev
           
    spi = None
    
    try: 
        spi = spidev.SpiDev()
        bus = 0
        device = 0
        spi.open(bus, device)            
        spi.max_speed_hz = int(CLK_FREQ_SPI_PICO)   #PICO implementation has max frequency of clock as 10MHz (100ns period)
        spi.mode = 0b00
        spi.lsbfirst = False 
        spi.bits_per_word = 8
            
    except Exception as e:
        print("Error is:",e)
        #GPIO.cleanup()
        if spi:
            spi.close()
            spi = None
    
    return spi

# Load DAC input and DAC registers simultainously: 
# Set bits 15-->13 to 010, bit 0 to 0, then 12-bit data in bits 12-->1.
def writeDAC_MAXIM5123(spi_obj, val, gpio_num_DAC_CE0B):  #Value argument in decimal 0-4095.
    import RPi.GPIO as GPIO
    if val>4095:
        print("Error - value too big for DAC")
    else:
        GPIO.output(gpio_num_DAC_CE0B,0)  #Enable the chip select of the DAC
        print('Updating DAC with value %i and expected voltage of %4.3f V'%( val,round(get_DACVoltage_fromValue_5123(val),3)),)
        DACValLower = (val<<1)&0xfe
        DACValUpper = (val >> 7) | 0x40
        #print(DACValUpper)
        #print(DACValLower)
        spi_obj.writebytes([DACValUpper,DACValLower])
        GPIO.output(gpio_num_DAC_CE0B,1)  #Disable the chip select of the DAC
    return()

def get_DACVoltage_fromValue_5123(val):
    """
    convert DAC value to calibrated
    Use the  amplifier feedback resistor values
    """
    if val>4095:
        print("Error - value too big for DAC")
        return None
    Vref = 1.25
    Rf = 12000
    Ri = 8450
    Av = 1+Rf/Ri
    dac_voltage = val/4095*(Av)*Vref
    return dac_voltage


#POST-PROCESSING
def imgvid_plot_IMG(matrix_list, directory, foldername, 
             vmin = 0.1, vmax = 3):
    import matplotlib.pyplot as plt
    import os
    import cv2
    """this function allows user to generate images, and video
    directly from the main matrices lists.
    matrix_list: List containing n number of frames. 
    directory: file path for saving these images
    foldername: A new folder of this string will be created in 
        the directory. This foldername string will also appear in
        the title
    vmin = lower value for the colorbar of the images
    vmax = upper value for the colorbar of the images
    
    """
    fig, ax = plt.subplots(figsize=(9, 9))
    savedirec = directory+foldername + '/'
    if not os.path.exists(savedirec):
        os.makedirs(savedirec)
    fps = 7
    img_array = []
    frame_title = 0
    for i in range(len(matrix_list)):
        ax.cla()
        out = matrix_list[i]

        pos = ax.imshow(out, vmin=vmin, vmax=vmax, cmap = 'hot', interpolation = 'bilinear')
        ax.set_title(foldername +" : Frame="+str(i))
        ax.set_xlabel("Columns")
        ax.set_ylabel("Rows")
        cbar = fig.colorbar(pos)

        plt.pause(0.01)
        img_name = savedirec+'/'+'frame'+str(frame_title)+'.png'
        fig.savefig(img_name)
        img = cv2.imread(img_name)
        height, width, layers = img.shape
        size = (width,height)
        img_array.append(img)
        frame_title = frame_title + 1
        
        while i<len(matrix_list)-1:
            cbar.remove()
        
    vid_file_name = savedirec + foldername+'_'+'video.avi'
    out = cv2.VideoWriter(vid_file_name,cv2.VideoWriter_fourcc(*'DIVX'), fps, size)
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()
    print("Done generating images and video from frames")
    
def convert_TwoBytes_To_Int(byte_1, byte_0):
    """
    Converts 2bytes data 
    byte_1 : MSB ,1 byte
    byte_0 : LSB , 1 byte
    """
    
    return (byte_1&0xFF)<<8 | (byte_0 & 0xFF)
    

def read_single_frame(spi_obj_arg):
    
    # Number of 16bit transfer per frame
    n_16bit_transfer_perFrame  =128*128*2     # 128 row X 128 col X 2 modes(I and Q) + 1sync indicator
    """
    read a frame from the spi_obj, eaxh transfer is 16 bit long
    Both I and Q data in a single frame as 1D array
    TODO:  Update it to block read once the FW supports it
    """
    
    # Returnn buffer Signature to indicate which buffer you are reading From
    bufferSignature = -1
    frameDataOut=[]
    # First two bytes ie frame sync transfer will be either #0xFFFF or #0xEFFF based on which buffer you are reading
    # Then it will spit 128*128*2 = n_16bit_transfer_perFrame
    
    N_BYTES_BLOB = 2
    
    
    # Keep track of unsyncronized data
    missedBlobCount = 0
    # TODO call with timeout
    while (missedBlobCount < n_16bit_transfer_perFrame+10):
        # Read 2 Bytes
        curData_TwoByte_List = spi_obj_arg.readbytes(N_BYTES_BLOB)
        #print(curData_TwoByte_List)
        blobValue = convert_TwoBytes_To_Int(*curData_TwoByte_List)
        
        if (blobValue == 0xEFFF) | (blobValue == 0xFFFF):            # Then the following n_16bit_transfer_perFrame will be the frame
            break
        else:
            missedBlobCount = missedBlobCount+1
            # display every 1000 value
            if missedBlobCount%1000 ==0:
                print(missedBlobCount,"{:X}".format(blobValue))
    
    # Save in case you want to check for any trouble
    bufferSignature = blobValue

    # TODO: Update this to block read later
    for byteIdx in range(n_16bit_transfer_perFrame):

        blobValue = convert_TwoBytes_To_Int(*spi_obj_arg.readbytes(N_BYTES_BLOB))
        frameDataOut.append(blobValue)
    
    
    return frameDataOut, bufferSignature, missedBlobCount


def getFrames_I_and_Q(spi_ob_arg=None, nFrames = 1):
    import numpy as np
    # Array parameters
    N_rows = 128
    N_cols = 128

    # Grabs nFrames and 
    # Returns numpy arrays of the data read from the 
    # create images
    frames_I = np.zeros((nFrames,N_rows, N_cols),dtype='uint16')
    frames_Q = np.zeros((nFrames,N_rows, N_cols),dtype='uint16')
    missedBlobCount_1D = 0*np.zeros((nFrames,))
    bufferSignature_1D = 0*np.zeros((nFrames,))
 
    flagMissedBlob = False
    firstMissedBlobCount = 0
    for frameIdx in range(nFrames):
        # Get the single Frame
        frameDataOut, bufferSignature, missedBlobCount = read_single_frame(spi_ob_arg)
        # Check if any Frame is Missed
        if not flagMissedBlob & (missedBlobCount>0):
            flagMissedBlob = False
            firstMissedBlobCount = missedBlobCount
     
        # ASsume the first I and then Q comes
        frames_I[frameIdx,:,:] = np.reshape(np.array(frameDataOut[::2], dtype='uint16'), (N_rows,N_cols))
        frames_Q[frameIdx,:,:] = np.reshape(np.array(frameDataOut[1::2], dtype='uint16'), (N_rows,N_cols))
        missedBlobCount_1D[frameIdx] = missedBlobCount
        bufferSignature_1D[frameIdx] = bufferSignature
    
    if firstMissedBlobCount > 0:
        print("WARNING: Missed Blob: FirstMissedBlobCount %i"%(firstMissedBlobCount))
    print("Captured %i frames!"%nFrames)
    return frames_I, frames_Q , bufferSignature_1D, missedBlobCount_1D
    
    
    # Poll until you read 
    
def dummy_frames(spi_obj_pico, n = 2):
    # number of dummy frames
    N_dummy_frames=n
    # Number of Frames
    N_baseline_frames = 0
    # Add 2 dummy frames at the beginning
    frames_I_baseline, frames_Q_baseline , bufferSignature_1D_baseline, missedBlobCount_1D_baseline = getFrames_I_and_Q(spi_ob_arg=spi_obj_pico, nFrames = N_baseline_frames+N_dummy_frames)


def read_data_until_synchronized(spi_obj_arg, bufferMarker_List =[0xEFFF, 0xFFFF] ):
    """
    Read 2 bytes by 2 bytes  until you hit the buffer signatures 0xEFFF and 0xFFFF
    and then read 128*128*2*2 bytes all at once
    """
    


    """
    read a frame from the spi_obj, eaxh transfer is 16 bit long
    Both I and Q data in a single frame as 1D array
    TODO:  Update it to block read once the FW supports it
    """
    
    # Returnn buffer Signature to indicate which buffer you are reading From
    bufferSignature = -1

    # First two bytes ie frame sync transfer will be either #0xFFFF or #0xEFFF based on which buffer you are reading
    # Then it will spit 128*128*2 = n_16bit_transfer_perFrame
    
    N_BYTES_BLOB = 2
    flagSignatureFound = False
    
    # Keep track of unsyncronized data
    missedBlobCount = 0
    # Number of 16bit transfer per frame
    n_16bit_transfer_perFrame  =128*128*2  
    # TODO call with timeout
    while (missedBlobCount < n_16bit_transfer_perFrame+10):
        # Read 2 Bytes
        curData_TwoByte_List = spi_obj_arg.readbytes(N_BYTES_BLOB)
        #print(curData_TwoByte_List)
        blobValue = convert_TwoBytes_To_Int(*curData_TwoByte_List)
        # If the flagSignatureFound is True,we are synchronized and the next 128*128*2 (byte pairs) are good    
        flagSignatureFound = blobValue in bufferMarker_List
        if flagSignatureFound:            # Then the following n_16bit_transfer_perFrame will be the frame
            break
        else:
            missedBlobCount = missedBlobCount+1
            # display every 1000 value
            #if missedBlobCount%1000 ==0:
            #    print(missedBlobCount,"{:X}".format(blobValue))
                
            # dellater
            #if missedBlobCount<5:
             #   print(missedBlobCount,"{:X}".format(blobValue))
    
    # Save in case you want to check for any trouble
    bufferSignature = blobValue
    

    return  flagSignatureFound, bufferSignature, missedBlobCount



N_BYTES_BLOCK = 1 << 1  # 2**10 bytes for every Block
def blockRead_SPI_with2byteconversion_BIN( spi_obj_arg, num_bytes_to_read_arg, n_bytes_block_arg = N_BYTES_BLOCK, flag_verbose=False):
    import numpy as np
    import math
    # Read 
    #num_bytes_to_read_actual =  1<<num_bytes_to_read_arg.bit_length()
    
    # Make sure the odd numbers are converted to one higher
    num_bytes_to_read_actual = int(2*np.ceil(num_bytes_to_read_arg/2)) 
    
    
    #allocate space
    data_out_nD =  bytearray(num_bytes_to_read_actual) #np.zeros((num_bytes_to_read_actual,), dtype='uint16')
    
    _n_bytes_block = n_bytes_block_arg
    
    # the minimum size to be read is at leat N_BYTES_BLOCK
    num_bytes_to_read_temp  = max(num_bytes_to_read_actual, _n_bytes_block)
    n_blocks= math.floor(num_bytes_to_read_temp/_n_bytes_block)
    n_bytes_remainder = num_bytes_to_read_actual%_n_bytes_block
    
    
    #num_bytes_to_read_actual = n_blocks*N_BYTES_BLOCK
    #data_all = []
    for blockIdx in range(n_blocks): 
        
        #cur_slice = np.s_[blockIdx*_n_bytes_block:(blockIdx+1)*_n_bytes_block]
        ##print("Reading block: %i"%blockIdx)
        #data_out_nD[cur_slice] =  np.array(spi_obj_arg.readbytes(_n_bytes_block), dtype = 'uint16')
        data_out_nD[blockIdx*_n_bytes_block:(blockIdx+1)*_n_bytes_block]=spi_obj_arg.readbytes(_n_bytes_block)

    
    if n_bytes_remainder >0:
        
        #cur_slice = np.s_[n_blocks*_n_bytes_block:]
        ## Read the remainder
        #data_out_nD[cur_slice] =  np.array(spi_obj_arg.readbytes(n_bytes_remainder), dtype='uint16')
        data_out_nD[n_blocks*_n_bytes_block:]=spi_obj_arg.readbytes(n_bytes_remainder)
    
    # Now convert to two bytes
    #COMMENT THIS OUT--> JK
    # data_out_nD = convert_TwoBytes_To_Int(data_out_nD[0::2], data_out_nD[1::2])
    # if flag_verbose:   
    #     print("Requested n_bytes = %i, n_blocks=%i, Returned n_bytes = %i as %i uint16"%(num_bytes_to_read_arg,n_blocks,int (2*data_out_nD.size),data_out_nD.size))
    return data_out_nD #this is a byte array


def read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg,n_bytes_block_arg=1<<1,   flag_verbose = False, bufferMarker_List =[0xEFFF, 0xFFFF] ):
    
    """
    Read 2 bytes by 2 bytes  until you hit the buffer signatures 0xEFFF and 0xFFFF
    and then read 128*128*2*2 bytes all at once
    """
    import datetime
    # Number of 16bit transfer per frame
    n_16bit_transfer_perFrame  =128*128*2     # 128 row X 128 col X 2 modes(I and Q) + 1sync indicator

    missedBlobCount_1D = 0
    bufferSignature_1D = 0
    flagSignatureFound_1D = False 
    #Also store the timestamp
    timeStamp_1D = datetime.datetime.now() #np.array([datetime.datetime.now()]*nFrames)
    N_rows = 128
    N_cols = 128
    # Note that initally we will store each and every bit
    # But we will do the final conversion at the end
    # DIM 0: nFrames
    # DIM 1: 2   for I,Q
    # DIM 2: picoDAQ_Lib.N_rows
    # DIM 3: picoDAQ_Lib.N_cols

    num_bytes_to_blockread = n_16bit_transfer_perFrame*2
    
    # Some initializations
    flagSignatureFound = False
    blobValue = -1
    missedBlobCount = -1

    if flagSignatureFound:
        # We got the frame
        bufferSignature = blobValue
        missedBlobCount = 0
    else:
        flagSignatureFound, bufferSignature, missedBlobCount = read_data_until_synchronized(spi_obj_arg = spi_obj_arg,\
                                                                                 bufferMarker_List =[0xEFFF, 0xFFFF] )
            
    frame_data_1D=bytearray(2*2*N_rows*N_cols)
    if flagSignatureFound:
    # perform the block read            
        frame_data_1D = blockRead_SPI_with2byteconversion_BIN( spi_obj_arg=spi_obj_arg,\
                                  num_bytes_to_read_arg = num_bytes_to_blockread, n_bytes_block_arg = n_bytes_block_arg)
        
    else:
        if flag_verbose:   
            print("No signature found for frame {}. Assuming all 0's".format(1))
        frame_data_1D = bytearray(2*2*N_rows*N_cols) #-1*np.ones((n_16bit_transfer_perFrame,), dtype = 'uint16')
    
    # You Update timestamp
    timeStamp_1D = datetime.datetime.now()
    
    # You have your first signature
    bufferSignature_1D = bufferSignature
    # also record the previous missedBlobCount
    missedBlobCount_1D = missedBlobCount
    # Sto0re the flagSignatureFoung
    flagSignatureFound_1D  = flagSignatureFound
    frames_data_nD = frame_data_1D
    
    return frames_data_nD,timeStamp_1D, flagSignatureFound_1D, bufferSignature_1D, missedBlobCount_1D

def get_spi_vco():   
    import spidev          
    spi = None
    
    try: 
        spi = spidev.SpiDev()
        bus = 0
        device = 1
        spi.open(bus, device)            
        spi.max_speed_hz = int(5e5)   #AD4351 has max frequency of clock as 10MHz (100ns period)
        spi.mode = 0b00
        spi.lsbfirst = False
        
        
    except Exception as e:
        print("Error is:",e)
        #GPIO.cleanup()
        if spi:
            spi.close()
            spi = None
    
    return spi

def int2bytes(x):
    return (x>>24) & 0xff, (x>>16) & 0xff , (x>>8) & 0xff, x & 0xff

# MAXIM 5123
def get_spi_dac_MAXIM5123():    
    import spidev           
    spi = None
    
    try: 
        spi = spidev.SpiDev()
        bus = 0
        device = 0
        spi.open(bus, device)            
        spi.max_speed_hz = int(1e6)   #AD4351 has max frequency of clock as 10MHz (100ns period)
        spi.mode = 0b10
        spi.lsbfirst = False
        
        
    except Exception as e:
        print("Error is:",e)
        #GPIO.cleanup()
        if spi:
            spi.close()
            spi = None
    
    return spi


def settings_freqswitch_RP2040(pinDict_Main, freuquency = 1853, ):
        # SEtup the GPIO
    GPIO_NUM_DAC_CE0B =22
    CLK_FREQ_SPI_PICO =int(8e6)
    setup_GPIO(pinDict_arg = pinDict_Main)
    
    #Create the sPI object for VCO
    # SPI-0, DEV-1, with 
    # Get VCO SPi object
    spi_obj_vco = get_spi_vco()
    freq_target_MHz = freuquency
    OUTEN_user = 1
    PSET_user = 3
    regs=calc_vco_reg_values(freq_target_MHz,OUTEN_user ,PSET_user)

    regshx=[]
    for i in range(len(regs)):
        regshx.append("{:#010x}".format(regs[i]))
    print("The registers are:", regshx)

    # Create list from regs
     # This is basically same as regbytesbyte
    regs_list_of_ints  = list(map(int2bytes, regs))
    
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
    spi_obj_dac = get_spi_dac_MAXIM5123()
    writeDAC_MAXIM5123(spi_obj_dac, DAC_val, GPIO_NUM_DAC_CE0B)
    # GET PICO SPI object
    spi_obj_pico = get_spi_pico(CLK_FREQ_SPI_PICO)
    # Block read Size
    n_bytes_block_arg=1<<12
    
    return spi_obj_pico, n_bytes_block_arg

def imgvid_plot_fsweep(matrix_list, directory, foldername, 
             start_freq = 1600, end_freq = 2000,
             step_freq = 1,
             vmin = 0.1, vmax = 5):
    
    import matplotlib.pyplot as plt
    import math
    import os
    import cv2
    import numpy as np
    """this function allows user to generate images, and video
    directly from the main matrices lists.
    matrix_list: List containing n number of 128x128 frames. 
    directory: file path for saving these images
    foldername: A new folder of this string will be created in 
        the directory. This foldername string will also appear in
        the title
    start_freq = start frequency: appears in the title
    end_freq = end frequency
    step_frequency = the difference between two consecutive frequencies
        in the frequency sweep, also known as f_delta
    vmin = lower value for the colorbar of the images
    vmax = upper value for the colorbar of the images
    
    """
    fig, ax = plt.subplots(figsize=(9, 9))
    savedirec = directory+foldername + '/'
    if not os.path.exists(savedirec):
        os.makedirs(savedirec)
   
    fps = 7
    img_array = []
    frames_num = len(matrix_list[0])
    for myf in range(start_freq*100,end_freq*100,math.floor(step_freq*100)):
        f_counter = 0
        freq_title = np.round(myf/100,2)
        freq_folder = savedirec+'_FREQ_'+str(myf)+'/'
        if not os.path.exists(freq_folder):
            os.makedirs(freq_folder)
        
        for jj in range(frames_num):
            ax.cla()
            out = matrix_list[f_counter][jj]
    
            pos = ax.imshow(out, vmin=vmin, vmax=vmax, cmap = 'cividis', interpolation = 'bilinear')
            ax.set_title(foldername +" : Frequency="+str(freq_title)+'MHz, Frame ='+str(jj))
            ax.set_xlabel("Columns")
            ax.set_ylabel("Rows")
            cbar = fig.colorbar(pos)
    
            plt.pause(0.01)
            img_name = freq_folder+'/'+'freq'+str(freq_title)+'MHz_Frame '+str(jj)+'.png'
            fig.savefig(img_name)
            img = cv2.imread(img_name)
            height, width, layers = img.shape
            size = (width,height)
            img_array.append(img)
            cbar.remove()
        vid_file_name = freq_folder+"VID_"+str(myf)+'MHz'+'.avi'
        out = cv2.VideoWriter(vid_file_name,cv2.VideoWriter_fourcc(*'DIVX'), fps, size)
        for i in range(len(img_array)):
            out.write(img_array[i])
        out.release()
        f_counter = f_counter+1

        freq_title = freq_title + step_freq    
    print("Done generating images and video from frames")

