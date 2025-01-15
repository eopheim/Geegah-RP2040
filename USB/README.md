# Setup
## Python Modules
Install the following python modules:

```bash
pip install ft4222 serial numpy opencv-python matplotlib
```

## Board Setup
There are two USB connectors for the RP1 imager.
>RP2040 USB is a serial connection to the RP2040 for changing mode, timing, and ADC selection. 

>SPI USB is a USB to SPI used to configure the VCO and DAC and provide 5V power to the imager board. It is also used to get the frame buffers from the RP2040.

RP2040 USB is only required when changing parameters. The imager can be used with the SPI USB only which will use default values on the RP2040.

## FT4222 Drivers
FTDI provides documentation for installing the FT4222 drivers onto the corresponding host system:
[Installation Guides](https://ftdichip.com/document/installation-guides/)
[Windows 10/11](https://ftdichip.com/wp-content/uploads/2023/11/AN_396_FTDI_Drivers_Installation_Guide_for_Windows_10_11.pdf)
[Mac OS](https://ftdichip.com/wp-content/uploads/2020/08/AN_134_FTDI_Drivers_Installation_Guide_for_MAC_OSX-1.pdf)
[Linux](https://ftdichip.com/wp-content/uploads/2020/08/AN_220_FTDI_Drivers_Installation_Guide_for_Linux-1.pdf)

## USB FT4222 Python Class

[GeegahImager.py](GeegahImager.py) 

Create a RP1 object. This can optionally take the VCO frequency and enable debug printing.

```python
RP1(vco_freq, verbose)
```

Main Python class for configuring and getting frames from the RP2040 based imager over USB. There are two main methods:

```python
set_rp2040_params(mode, glob_en_delay, adc)
```

Configures RP2040 parameters. Requires RP2040 USB connection.

```python
get_frame()
```

Gets I and Q frame data from the RP2040 buffer and returns the I and Q ADC and Voltage arrays (128 x  128) x 4

## Live Image Example
[Geegah_RP2040_live_image.py](Geegah_RP2040_live_image.py)
This provides a basic example of configuring the RP2040 imager and getting image frames. The I and Q voltage frames and modified and displayed using OpenCV. The script can also optionally save the frames to a video.

## RP2040 Firmware
[RP2040_Imager.uf2](RP2040_Imager.uf2)
RP2040 UF2 firmware file. Upload this to the RP2040 over the RP2040 USB to insure compatible firmware is used. This firmware version has been modified from the main original to allow the RP2040 USB to be disconnected. It also sets default values to Mode 1, Delay 125us, ADC1.


[Firmware source code](https://github.com/eopheim/RP2040_Imager_No_Echo)