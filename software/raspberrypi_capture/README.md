Sample program that grabs a frame on the Pi and outputs it as a PGM file

Defaults to SPI channel 0 - update the "/dev/spidev0.*" value accordingly

Build with 'make'

Usage: ./raspberrypi_capture [-t <lepton_type>]
  -t <lepton_type>: Specify Lepton type (2 for 2.x, 3 for 3.x). Default is 2.


