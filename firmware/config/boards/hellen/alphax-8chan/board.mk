# Combine the related files for a specific platform and MCU.

# Target ECU board design
BOARDCPPSRC = $(BOARD_DIR)/board_configuration.cpp
DDEFS += -DEFI_MAIN_RELAY_CONTROL=TRUE

# Turn off stuff we don't have/need
DDEFS += -DBOARD_TLE8888_COUNT=0

# Add them all together
DDEFS += -DFIRMWARE_ID=\"AlphaX-8chan\"
DDEFS += -DEFI_SOFTWARE_KNOCK=TRUE -DSTM32_ADC_USE_ADC3=TRUE

# MM176_GP9
DDEFS += -DADC_MUX_PIN=Gpio::F2

include $(BOARDS_DIR)/hellen/hellen-common144.mk

ifeq ($(PROJECT_CPU),ARCH_STM32F7)
	# TODO: why do I struggle to fit into flash? compare with Proteus
	DDEFS += -DCH_DBG_ENABLE_ASSERTS=FALSE
	DDEFS += -DENABLE_PERF_TRACE=FALSE
    USE_OPT += -Wl,--defsym=FLASH_SIZE=768k
else ifeq ($(PROJECT_CPU),ARCH_STM32F4)
    # This board has trigger scope hardware!
    DDEFS += -DTRIGGER_SCOPE
    # serial ports only on F4
	DDEFS += $(PRIMARY_COMMUNICATION_PORT_USART2)
else
$(error Unsupported PROJECT_CPU [$(PROJECT_CPU)])
endif

DDEFS += -DSHORT_BOARD_NAME=alphax-8chan -DSTATIC_BOARD_ID=STATIC_BOARD_ID_ALPHAX_8CHAN
DDEFS += -DHW_HELLEN_8CHAN=1
