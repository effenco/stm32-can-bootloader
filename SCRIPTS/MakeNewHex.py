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

BTLD_PROJECT = "BOOT"
APP_PROJECT = "APP"
ProjectPath = 'C:\\github\\stm32-can-bootloader'

def clean_hex_from_folder(folder):

    print(f'Cleaning {folder}')
    old_path=os.getcwd()
    
    # browse BTLD_PROJECT folder for all .hex files
    temp_list = os.listdir(folder)
    
    hexfound=0
    
    os.chdir(folder)
    
    hex_list=[]
    for file in temp_list:
        # Check extension
        if os.path.splitext(file)[1]==".hex":
            hex_list.append(file)
            hexfound=1
    
    # if no .hex file display error message and exit script
    if hexfound==0:
        print(f'No hex files found in {BTLD_PROJECT} folder')
    else:
        for file in hex_list:
            os.remove(file)
            print(f'Removed {file}')

    os.chdir(old_path)

def cleanAll():
    clean_hex_from_folder(BTLD_PROJECT)
    clean_hex_from_folder(APP_PROJECT)
    clean_hex_from_folder(os.getcwd())

def CopyHexRelease(project):
    old_path=os.getcwd()
    os.chdir(project)
    
    [hexfound, hexlist]=GetHexList('Debug')
    
    if hexfound==0:
        # if no .hex file display error message and exit script
        print(f'!!! No hex files found in {project}\Debug folder, script will stop !!!')
        os.chdir(old_path)
        return 0
    else:
        # list all .hex files and input which file to use from user
        print(f'list of .hex file found in {project}\Debug :')
            
        for idx,file in enumerate(hexlist):
            print(f'\t{idx} {file}')
        
        HEX_file = hexlist[0]
        
        print(f'{HEX_file} will be used!\n')
        
        # read selected .hex file and copy content in BTLD_HEX
        # BTLD_HEX_file
        path=os.path.join('Debug',f'{HEX_file}')
        f = open(path,"r")
        HEX=f.read()
        f.close()
        
        # Copy HEX to outputstr
        outputstr=HEX
        
        CHECKSUM=GetHexCrc(path)
        
        #make filename
        output_hex_file=f'{os.path.splitext(HEX_file)[0]} {CHECKSUM}.hex'
        
        print(f'Output will be {output_hex_file}\n')
        
        # save outputstr
        f = open(output_hex_file,'w+')
        f.write(outputstr)
        f.close()
    
        os.chdir(old_path)
        return 1

def GetHexCrc(file):
    # convert hex to bin
    ih=IntelHex()
    ih.fromfile(file,format='hex')
    bf=ih.tobinarray()
        
    #pad the array (flash size) for CRC calculation 220kb
    while len(bf)<(220*1024):
        bf.append(255)
        
    # calculate checksum for APP_HEX
    crc32_func=crcmod.mkCrcFun(int(0x104c11db7),initCrc=int(0xffffffff),rev=False,xorOut=0)
        
    checksum_val=crc32_func(bytearray(bf))
    checksum_val=f'0x{checksum_val:X}'
    
    return checksum_val

def GetHexList(folder):
    # browse BTLD_PROJECT folder for all .hex files
    temp_list = os.listdir(folder)
    
    found=0
    
    hex_list=[]
    for file in temp_list:
        # Check extension
        if os.path.splitext(file)[1]==".hex":
            hex_list.append(file)
            found=1
    return [found,hex_list]

def MakeCombinedHex(prj_a,prj_b):
    
    [a_found,a_list] = GetHexList(prj_a)
    
    if a_found==0:
        print(f'!!! No hex files found in {prj_a} folder, script will stop !!!')
        return 0
    
    print(f'list of .hex file found in {prj_a} :')
    
    printlist(a_list)
    
    if len(a_list)==1:
        A_HEX_file = a_list[0]
    else:
        #prompt
        user_number  = input('which file do you wish to use? :')
        if( user_number.isdigit()):
            user_number=int(user_number)
            if user_number>=0 and user_number<len(a_list):
                A_HEX_file = a_list[user_number]
            else:
                print('!!! Wrong input !!!')
                return 0
        else:
            print('!!! Wrong input !!!')
            return 0
    
    print(f'{A_HEX_file} will be used!\n')
    
    
    
    
    [b_found,b_list] = GetHexList(prj_b)
    
    if b_found==0:
        print(f'!!! No hex files found in {prj_a} folder, script will stop !!!')
        return 0

    print(f'list of .hex file found in {prj_b} :')
    
    printlist(b_list)
    
    if len(b_list)==1:
        B_HEX_file = b_list[0]
    else:
        #prompt
        user_number  = input('which file do you wish to use? :')
        if( user_number.isdigit()):
            user_number=int(user_number)
            if user_number>=0 and user_number<len(b_list):
                B_HEX_file = b_list[user_number]
            else:
                print('!!! Wrong input !!!')
                return 0
        else:
            print('!!! Wrong input !!!')
            return 0
    
    print(f'{B_HEX_file} will be used!\n')
    
    # read selected boot .hex file and copy content in BTLD_HEX
    # BTLD_HEX_file
    path=os.path.join(f'{prj_a}',f'{A_HEX_file}')
    f = open(path,"r")
    A_HEX=f.read()
    f.close()
    
    # read selected app .hex file and copy content in APP_HEX
    # APP_HEX_file
    path=os.path.join(f'{prj_b}',f'{B_HEX_file}')
    f = open(path,"r")
    B_HEX=f.read()
    f.close()
    
    # Copy BTLD_HEX to outputstr
    outputstr=A_HEX
    
    # remove the last line :00000001FF\n
    outputstr=outputstr.replace(':00000001FF\n','')
    
    # concatenate APP_HEX at the end of outputstr
    outputstr=f'{outputstr}{B_HEX}'
    

    path=os.path.join(f'{prj_b}',f'{B_HEX_file}')

    CHECKSUM=GetHexCrc(path)
    
    #make filename
    output_hex_file=f'{prj_b} BTLD AND APP {CHECKSUM}.hex'
    
    print(f'Output will be {output_hex_file}\n')
    
    # save outputstr
    f = open(output_hex_file,'w+')
    f.write(outputstr)
    f.close()

    return 1

def printlist(thelist):
    for idx,file in enumerate(thelist):
        print(f'\t{idx} {file}')

os.chdir(ProjectPath)
main_path=os.getcwd()

cleanAll()

if CopyHexRelease(BTLD_PROJECT):

    if CopyHexRelease(APP_PROJECT):
        
        if MakeCombinedHex(BTLD_PROJECT,APP_PROJECT):
            print('sucess!')
        else:
            print('Flaied')
    else:
        print('Flaied')
else:
    print('Flaied')

os.chdir(main_path)

#MakeCombinedReleaseHex()

input()
