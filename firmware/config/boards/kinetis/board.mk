# List of all the board related files.
BOARDSRC = $(PROJECT_DIR)/config/boards/kinetis/board.c
BOARDSRC_CPP = $(PROJECT_DIR)/config/boards/kinetis/board_configuration.cpp

# Required include directories
BOARDINC = $(PROJECT_DIR)/config/boards/kinetis

# Define linker script file here
LDSCRIPT= $(STARTUPLD)/MKE1xF512.ld

PLATFORMSRC += $(PLATFORMSRC_CONTRIB)
PLATFORMINC += $(PLATFORMINC_CONTRIB)

HALSRC += $(CHIBIOS_CONTRIB)/os/hal/src/hal_comp.c

HW_LAYER_EMS_CPP += $(PROJECT_DIR)/hw_layer/trigger_input_comp.cpp
