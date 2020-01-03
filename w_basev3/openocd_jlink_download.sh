#!/bin/sh
# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
# 
#   http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.
#

# Called with following variables set:
#  - CORE_PATH is absolute path to @apache-mynewt-core
#  - BSP_PATH is absolute path to hw/bsp/bsp_name
#  - BIN_BASENAME is the path to prefix to target binary,
#    .elf appended to name is the ELF file
#  - IMAGE_SLOT is the image slot to download to (for non-mfg-image, non-boot)
#  - FEATURES holds the target features string
#  - EXTRA_JTAG_CMD holds extra parameters to pass to jtag software
#  - MFG_IMAGE is "1" if this is a manufacturing image
#  - FLASH_OFFSET contains the flash offset to download to
#  - BOOT_LOADER is set if downloading a bootloader
#FLASH_OFFSET=0x8000000
#IMAGE_SLOT=0
#CORE_PATH=/mnt/c/dev/wProto-MyNewt/wProtoMyNewt/repos/apache-mynewt-core
#BSP_PATH=C:/dev/wProto-MyNewt/wProtoMyNewt/hw/bsp/wyresRevB-BasedStm32l152discovery
#BSP_PATH2=/mnt/C/dev/wProto-MyNewt/wProtoMyNewt/hw/bsp/wyresRevB-BasedStm32l152discovery
#BIN_BASENAME=bin/targets/wyresRevB_bootloader/app/boot/mynewt/mynewt

. $CORE_PATH/hw/scripts/openocd.sh

CFG="-s $BSP_PATH -f jlink.cfg"
echo "1. $CFG"
if [ "$MFG_IMAGE" ]; then
    FLASH_OFFSET=0x08000000
fi
echo "2."
common_file_to_load
echo "3."
openocd_load
echo "4."
openocd_reset_run
echo "5."
