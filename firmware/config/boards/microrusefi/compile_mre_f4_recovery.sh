#!/bin/bash

export EXTRA_PARAMS="-DIGNORE_FLASH_CONFIGURATION=TRUE"

# export DEBUG_LEVEL_OPT="-O0 -ggdb -g"

bash ../common_make.sh config/boards/microrusefi/meta-info_f4.env
