#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 13 07:26:47 2019

@author: frainville

library requirements
python3 -m pip install intelhex
python3 -m pip install crcmod

"""

import re
import os
from intelhex import IntelHex
import crcmod

SOURCEFOLDER = "C:\\github\\stm32-can-bootloader\\APP\\Debug"

HEX_file = ""
HEX = ""
outputstr = ""

# browse BTLD_PROJECT folder for all .hex files
temp_list = os.listdir(SOURCEFOLDER)

hexfound=0

hex_list=[]
for file in temp_list:
    # Check extension
    if os.path.splitext(file)[1]==".hex":
        hex_list.append(file)
        hexfound=1

# if no .hex file display error message and exit script
if hexfound==0:
    print(f'!!! No hex files found in {BTLD_PROJECT} folder, script will stop !!!')
    quit()
    
# list all .hex files and input which file to use from user
print(f'list of .hex file found in {SOURCEFOLDER} :')
    
for idx,file in enumerate(hex_list):
    print(f'\t{idx} {file}')

HEX_file = hex_list[0]

print(f'{HEX_file} will be used!\n')

# read selected .hex file and copy content in BTLD_HEX
# BTLD_HEX_file
path=os.path.join(f'{SOURCEFOLDER}',f'{HEX_file}')
f = open(path,"r")
HEX=f.read()
f.close()

# Copy HEX to outputstr
outputstr=HEX

# convert hex to bin
ih=IntelHex()
ih.fromfile(path,format='hex')
bf=ih.tobinarray()

#pad the array (flash size) for CRC calculation 220kb
while len(bf)<(220*1024):
    bf.append(255)

# calculate checksum for APP_HEX
crc32_func=crcmod.mkCrcFun(int(0x104c11db7),initCrc=int(0xffffffff),rev=False,xorOut=0)

CHECKSUM=crc32_func(bytearray(bf))
CHECKSUM=f'0x{CHECKSUM:X}'

#make filename
output_hex_file=f'{os.path.splitext(HEX_file)[0]} {CHECKSUM}.bin'

# save Binfile
f = open(output_hex_file,'w+b')
#binary_format =bytearray(bf)
#f.write(binary_format)
f.write(bf)
f.close()
