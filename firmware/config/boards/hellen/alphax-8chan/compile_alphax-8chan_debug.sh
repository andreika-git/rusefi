#!/bin/bash

export EXTRA_PARAMS="-DRAMDISK_INVALID -DEFI_LUA=FALSE"
export DEBUG_LEVEL_OPT="-O0 -ggdb -g"
export INCLUDE_ELF=yes
cd ..
bash ../common_make.sh config/boards/hellen/alphax-8chan/meta-info_f4.env
