import math
import serial
import serial.tools.list_ports
import re

import ft4222
from ft4222.SPI import Cpha, Cpol
from ft4222.SPIMaster import Mode, Clock, SlaveSelect

'''
    Ethan Opheim
    Jan 14, 2025
    Geegah Inc.
'''

class RP1:
    def __init__(self, vco_freq = 1843, verbose = False):
        """RP1 imager class for configuring DAC and VCO. Get buffer frame data from the RP2040.

        Args:
            vco_freq (int, optional): Set the VCO frequency in Mhz. Defaults to 1843.
            verbose (bool, optional): Enable debug print output. Defaults to False.
        """
        # Debugging output
        self.verbose = verbose

        self.vco_freq = vco_freq

        self.DAC_val = 3815 # 3815

        print("Connecting FT4222...")

        # FT4222 SPI0 SS0O: RP2040_CS
        self._spi_rp2040 = ft4222.openByDescription('FT4222 A')
        # FT4222 SPI0 SS1O: VCO_LE
        self._spi_vco = ft4222.openByDescription('FT4222 B')
        # FT4222 SPI0 SS2O: DAC_CS
        self._spi_dac = ft4222.openByDescription('FT4222 C')

        self._spi_rp2040.setClock(ft4222.SysClock.CLK_60)

        # init spi master
        self._spi_rp2040.spiMaster_Init(Mode.SINGLE, Clock.DIV_8, Cpol.IDLE_LOW, Cpha.CLK_LEADING, SlaveSelect.SS0)
        self._spi_vco.spiMaster_Init(Mode.SINGLE, Clock.DIV_32, Cpol.IDLE_LOW, Cpha.CLK_LEADING, SlaveSelect.SS1)
        self._spi_dac.spiMaster_Init(Mode.SINGLE, Clock.DIV_32, Cpol.IDLE_LOW, Cpha.CLK_LEADING, SlaveSelect.SS2)

        print("Setting up VCO...")

        # Setup VCO
        self._vco()

        print("Setting up DAC...")

        # Setup DAC
        self._dac()

        print("Clearing buffers and syncing to buffer 0...")
        
        self._spi_rp2040.spiMaster_Init(Mode.SINGLE, Clock.DIV_8, Cpol.IDLE_LOW, Cpha.CLK_LEADING, SlaveSelect.SS0)

        # Read 2 frames to read zeros in buffer after RP2040 restart
        for i in range(2):
            self.get_frame()

        # Sync RP2040 to buffer 0
        self._sync_to_buffer_0()


    def get_frame(self):
        """Returns the next frame from the RP2040 buffer.

        Returns:
        [(128,128), (128,128), (128,128), (128,128)]: I ADC, Q ADC, I Volts, Q Volts
        """
        # Read frame signature word
        self._spi_rp2040.spiMaster_SingleRead(2, False)

        # Read frame buffer
        frames_data_nD_sample = self._spi_rp2040.spiMaster_SingleRead(128*128*2, False)
        frames_data_nD_sample += self._spi_rp2040.spiMaster_SingleRead(128*128*2, False)
        
        # Convert frames into ADC counts and Volts
        i_frame_adc, q_frame_adc, i_frame_volts, q_frame_volts = self._loadRawByteDataRP2040(frames_data_nD_sample)

        return i_frame_adc, q_frame_adc, i_frame_volts, q_frame_volts
    
    def set_rp2040_params(self, mode = 1, time = 125, adc = 1):
        """Configure RP2040 mode, global enable delay, and adc. Required RP2040 USB serial connection

        Args:
            mode (int, optional): 0: no echo delay, 1: global enable delay time. Defaults to 1.
            time (int, optional): increments of 5ns >= 25ns. Defaults to 125.
            adc (int, optional): 1: Gain = 5, 2: Gain = 1. Defaults to 1.
        """
        # Check inputs are valid
        if mode != 0 and mode != 1:
            print("Error: Invalid mode selection. Must be 0 or 1.")
            exit()

        if time%5 or time <25:
            print("Error: Invalid time. Must be multiple of 5ns and >=25ns.")

        if adc != 1 and adc != 2:
            print("Error: Invalid ADC selection. Must be 1 or 2.")

        # Get the serial port for the connected RP2040
        port = self._get_rp2040_serial_port()
        if port != None:
            ser = serial.Serial(port, 115200)

            packet = str(str(mode)+'.'+str(time)+'.'+str(adc)+'\n')

            ser.write(packet.encode())

            self._sync_to_buffer_0()
    
    def _get_rp2040_serial_port(self):
        ports = serial.tools.list_ports.comports()

        # If the ports list is empty, print error message to check USB connection
        if ports == []:
            print("Error: No serial ports found! Check RP2040 connection")
            exit()

        for port, desc, hwid in sorted(ports):
            # Checks if USB VID:PID matches RP2040 HWID
            if re.search('VID:PID=2E8A:000A',hwid) != None:
                # Save serial number of the connected RP2040
                ser_loc = re.search('SER=',hwid).span()[1]
                self.rp2040_ser_num = hwid[ser_loc:ser_loc+16]

                # Return first valid com port found
                return port
            
        print("Error: No RP2040 serial ports found! Check connection")
        return None

    def _sync_to_buffer_0(self):
        # Read words until buffer 1 signature (0xEFFF) is found
        while True:
            d = self._spi_rp2040.spiMaster_SingleRead(2,False)
            d = int.from_bytes(d)
            if (d == 0xEFFF): 
                if self.verbose:
                    print("Found 0xEFFF signature for sync")
                break

        # Read frame buffer 1 to sync back to 0xFFFF signature
        if self.verbose:
            print("Reading until 0xFFFF buffer reached")

        self._spi_rp2040.spiMaster_SingleRead(128*128*2, False)
        self._spi_rp2040.spiMaster_SingleRead(128*128*2, False)

        # Clear out zeros in buffer on initilization
        # for i in range(2):
        #     self._spi_rp2040.spiMaster_SingleRead(2, False) # Read signature word
        #     # Read frame buffer
        #     self._spi_rp2040.spiMaster_SingleRead(128*128*2, False)
        #     self._spi_rp2040.spiMaster_SingleRead(128*128*2, False)
        #     time.sleep(100e-3)
    
    def _vco(self):
        self._spi_vco.spiMaster_Init(Mode.SINGLE, Clock.DIV_32, Cpol.IDLE_LOW, Cpha.CLK_LEADING, SlaveSelect.SS1)

        # VCO settings and registers
        # Parameter selections

        # reference oscillator
        refin = 10 # in MHz
        # VCO PFD (phase frequency detector) frequency
        myPFD = 10 # in MHz
        # this is fixed, not user specifiable
        freq_res = 0.01 # in MHz

        freq_target_MHz = self.vco_freq # Pulse frequency in MHz, with resolution of 0.1 MHz
        OUTEN = 1
        PSET = 3

        #make sure it is in the correct resolution
        freq_actual = freq_res * round(freq_target_MHz / freq_res)
        
        #calculate divider setting
        myDIVSEL = math.ceil(math.log(math.ceil(2200 / freq_actual)) / math.log(2))
        myDIV = 2**(myDIVSEL)
        
        mylumped = freq_actual * myDIV / myPFD #N in the ADI software
        
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
            myMOD = refin / freq_res
            myFRAC = myMOD * (mylumped - math.floor(mylumped))
            myMOD = round(myMOD) #if some decimal stuff left
            myFRAC = round(myFRAC)
            
            while(math.gcd(myMOD, myFRAC) > 1):
                mygcd = math.gcd(myMOD, myFRAC)
                myMOD = round(myMOD / mygcd)
                myFRAC = round(myFRAC / mygcd)
            
        #structure of register 0
        #0 at db31, 16 bit INT, 12 bit FRAC, 3 control bits 000
        reg0 = (myFRAC * (2**3)) + (myINT * (2**15))
        #structure of register 1
        reg1 = 0x8008001  + (myMOD * (2**3))
        
        #calculate reg4
        reg4 = (2**23) #fundamental feedback select
        reg4 = reg4 + (myDIVSEL * (2**20)) #divider select
        reg4 = reg4 + (2**2) #control bits
        reg4 = reg4 + (PSET * (2**3)) #power setting
        reg4 = reg4 + (OUTEN * (2**5)) #output enable setting
        reg4 = reg4 + ((80) * (2**12)) #band select clock divider value
        
        # Registers that are unchanged
        reg2 = 0x4E42
        reg3 = 0x4B3
        reg5 = 0x580005

        regs = [reg0,reg1,reg2,reg3,reg4,reg5]

        regshx=[]
        for i in range(len(regs)):
            regshx.append("{:#010x}".format(regs[i]))
        if self.verbose:
            print("The vco registers are:", regshx)

        # Create list from regs
        # This is basically same as regbytesbyte
        regs_list_of_ints  = list(map(self._int2bytes, regs))

        # Write register values
        for i in range(len(regs)-1,-1,-1):
            regToWrite_list = regs_list_of_ints[i]

            byte_array = bytearray(regToWrite_list)
            self._spi_vco.spiMaster_SingleWrite(byte_array, True)

    def _int2bytes(self, x):
        return (x>>24) & 0xff, (x>>16) & 0xff , (x>>8) & 0xff, x & 0xff

    def _dac(self):
        self._spi_dac.spiMaster_Init(Mode.SINGLE, Clock.DIV_32, Cpol.IDLE_LOW, Cpha.CLK_LEADING, SlaveSelect.SS2)

        if self.verbose:
            print(f"DAC voltage: {self._get_DACVoltage_fromValue_5123(self.DAC_val)}")

        self._writeDAC_MAXIM5123(self.DAC_val)
    
    def _get_DACVoltage_fromValue_5123(self,val):
        """
        convert DAC value to calibrated
        Use the  amplifier feedback resistor values
        """
        if val > 4095:
            print("Error - value too big for DAC")
            return None
        Vref = 1.25
        Rf = 12000
        Ri = 8450
        Av = 1 + Rf / Ri
        dac_voltage = val / 4095 * (Av) * Vref

        return dac_voltage
    
    def _writeDAC_MAXIM5123(self, val):  #Value argument in decimal 0-4095.
        if val > 4095:
            print("Error - value too big for DAC")
        else:
            if self.verbose:
                print('Updating DAC with value %i and expected voltage of %4.3f V'%(val, round(self._get_DACVoltage_fromValue_5123(val),3)),)
            
            DACValLower = (val << 1) & 0xFE
            DACValUpper = (val >> 7) | 0x40
            byte_array = bytearray([DACValUpper,DACValLower])

            self._spi_dac.spiMaster_SingleWrite(byte_array, True)
    
    def _loadRawByteDataRP2040(self, byte_data):
        I_RAW, Q_RAW = self._convertToIQImageRP(byte_data)
        I_ADC, Q_ADC, I_VOLTS, Q_VOLTS = self._convertADCToVoltsRP(I_RAW, Q_RAW)
        return I_ADC, Q_ADC, I_VOLTS, Q_VOLTS
    
    def _convertToIQImageRP(self, byte_data):
        import numpy as np
        wi = 0

        imgBytesI = np.zeros(128*128)
        imgBytesQ = np.zeros(128*128)
        for row in range (128):
            for col in range(128):
                wi = row * 128 + col

                iwrd = (byte_data[4 * wi + 1] + 256*byte_data[4 * wi + 0]) #swap +0 and +1
                qwrd = (byte_data[4 * wi + 3] + 256*byte_data[4 * wi + 2]) #swap +2 and +3

                imgBytesI[wi] = iwrd
                imgBytesQ[wi] = qwrd

        
        IMG_I = imgBytesI.reshape([128,128])
        IMG_Q = imgBytesQ.reshape([128,128])

        # Move first column to last to fix order issue with the I frames
        IMG_I = np.roll(IMG_I, -1, axis=1)
        #IMG_Q = np.roll(IMG_Q, -1, axis=1)

        return IMG_I, IMG_Q
    
    #convert raw ADC data to bit-shift corrected ADC data and convert to voltage
    def _convertADCToVoltsRP(self, I_IMAGE, Q_IMAGE):
        I_IMAGE_ADC = I_IMAGE / 1 #correct bit shift
        Q_IMAGE_ADC = Q_IMAGE / 1 #correct bit shift
        I_IMAGE_VOLTS = I_IMAGE_ADC * 1e-3 #convert to volts
        Q_IMAGE_VOLTS = Q_IMAGE_ADC * 1e-3 #convert to volts
        
        return I_IMAGE_ADC, Q_IMAGE_ADC, I_IMAGE_VOLTS, Q_IMAGE_VOLTS