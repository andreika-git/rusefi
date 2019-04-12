/*
 * How to setup clock using clock driver functions:
 *
 * 1. Call CLOCK_InitXXX() to configure corresponding SCG clock source.
 *    Note: The clock could not be set when it is being used as system clock.
 *    In default out of reset, the CPU is clocked from FIRC(IRC48M),
 *    so before setting FIRC, change to use another avaliable clock source.
 *
 * 2. Call CLOCK_SetXtal0Freq() to set XTAL0 frequency based on board settings.
 *
 * 3. Call CLOCK_SetXxxModeSysClkConfig() to set SCG mode for Xxx run mode.
 *    Wait until the system clock source is changed to target source.
 *
 * 4. If power mode change is needed, call SMC_SetPowerModeProtection() to allow
 *    corresponding power mode and SMC_SetPowerModeXxx() to change to Xxx mode.
 *    Supported run mode and clock restrictions could be found in Reference Manual.
 */

#include "hal.h"
#include "fsl_smc.h"
#include "clock_config.h"

/* System clock frequency. */
//extern uint32_t SystemCoreClock;

/* Variables for BOARD_BootClockRUN configuration */
const scg_sys_clk_config_t g_sysClkConfig_BOARD_BootClockRUN = {
        .divSlow = kSCG_SysClkDivBy7,             /* Slow Clock Divider: divided by 7 */
        .divBus = kSCG_SysClkDivBy8,              /* Bus Clock Divider: divided by 8 */
        .divCore = kSCG_SysClkDivBy1,             /* Core Clock Divider: divided by 1 */
        .src = kSCG_SysClkSrcSysPll,              /* System PLL is selected as System Clock Source */
};
const scg_sosc_config_t g_scgSysOscConfig_BOARD_BootClockRUN = {
        .freq = KINETIS_CLK,                      /* System Oscillator frequency: 8000000Hz */
        .enableMode = kSCG_SysOscEnable,          /* Enable System OSC clock */
        .monitorMode = kSCG_SysOscMonitorDisable, /* Monitor disabled */
        .div1 = kSCG_AsyncClkDisable,             /* System OSC Clock Divider 1: Clock output is disabled */
        .div2 = kSCG_AsyncClkDisable,             /* System OSC Clock Divider 2: Clock output is disabled */
        .workMode = kSCG_SysOscModeExt,           /* Use external clock */
};
const scg_sirc_config_t g_scgSircConfig_BOARD_BootClockRUN = {
        .enableMode = kSCG_SircEnableInLowPower,/* Enable SIRC clock, Enable SIRC in low power mode */
        .div1 = kSCG_AsyncClkDisable,             /* Slow IRC Clock Divider 1: Clock output is disabled */
        .div2 = kSCG_AsyncClkDisable,             /* Slow IRC Clock Divider 2: Clock output is disabled */
        .range = kSCG_SircRangeHigh,              /* Slow IRC high range clock (8 MHz) */
};
const scg_firc_config_t g_scgFircConfig_EXT_BOARD_BootClockRUN = {
        .enableMode = kSCG_FircDisableRegulator,  /* Enable FIRC clock */
        .div1 = kSCG_AsyncClkDisable,             /* Fast IRC Clock Divider 1: Clock output is disabled */
        .div2 = kSCG_AsyncClkDisable,             /* Fast IRC Clock Divider 2: Clock output is disabled */
        .range = kSCG_FircRange48M,               /* Fast IRC is trimmed to 48MHz */
        .trimConfig = NULL,                       /* Fast IRC Trim disabled */
};
const scg_spll_config_t g_scgSysPllConfig_EXT_BOARD_BootClockRUN = {
        .enableMode = kSCG_SysPllEnable,          /* Enable SPLL clock */
        .monitorMode = kSCG_SysPllMonitorDisable, /* Monitor disabled */
        .div1 = kSCG_AsyncClkDivBy64,             /* System PLL Clock Divider 1: divided by 64 */
        .div2 = kSCG_AsyncClkDivBy8,              /* System PLL Clock Divider 2: divided by 8 */
        .src = kSCG_SysPllSrcSysOsc,              /* System PLL clock source is System OSC */
        .prediv = KINETIS_EXT_PLL_DIV,            /* 0=Divided by 1 */
        .mult = KINETIS_EXT_PLL_MUL,              /* Multiply Factor */
};

const scg_firc_config_t g_scgFircConfig_INT_BOARD_BootClockRUN = {
        .enableMode = kSCG_FircEnable,            /* Enable FIRC clock */
        .div1 = kSCG_AsyncClkDisable,             /* Fast IRC Clock Divider 1: Clock output is disabled */
        .div2 = kSCG_AsyncClkDisable,             /* Fast IRC Clock Divider 2: Clock output is disabled */
        .range = kSCG_FircRange48M,               /* Fast IRC is trimmed to 48MHz */
        .trimConfig = NULL,                       /* Fast IRC Trim disabled */
};
const scg_spll_config_t g_scgSysPllConfig_INT_BOARD_BootClockRUN = {
        .enableMode = kSCG_SysPllEnable,          /* Enable SPLL clock */
        .monitorMode = kSCG_SysPllMonitorDisable, /* Monitor disabled */
        .div1 = kSCG_AsyncClkDivBy64,             /* System PLL Clock Divider 1: divided by 64 */
        .div2 = kSCG_AsyncClkDivBy8,              /* System PLL Clock Divider 2: divided by 8 */
        .src = kSCG_SysPllSrcFirc,                /* System PLL clock source is Fast IRC */
        .prediv = KINETIS_INT_PLL_DIV,            /* Divided by Factor */
        .mult = KINETIS_INT_PLL_MUL,              /* Multiply Factor */
};

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_CONFIG_FircSafeConfig
 * Description   : This function is used to safely configure FIRC clock.
 *                 In default out of reset, the CPU is clocked from FIRC(IRC48M).
 *                 Before setting FIRC, change to use SIRC as system clock,
 *                 then configure FIRC. After FIRC is set, change back to use FIRC
 *                 in case SIRC need to be configured.
 * Param fircConfig  : FIRC configuration.
 *
 *END**************************************************************************/
static void CLOCK_CONFIG_FircSafeConfig(const scg_firc_config_t *fircConfig) {
    scg_sys_clk_config_t curConfig;
    const scg_sirc_config_t scgSircConfig = {.enableMode = kSCG_SircEnable,
                                             .div1 = kSCG_AsyncClkDisable,
                                             .div2 = kSCG_AsyncClkDivBy2,
                                             .range = kSCG_SircRangeHigh};
    scg_sys_clk_config_t sysClkSafeConfigSource = {
         .divSlow = kSCG_SysClkDivBy4, /* Slow clock divider */
         .divBus = kSCG_SysClkDivBy1,  /* Bus clock divider */
         .divCore = kSCG_SysClkDivBy1, /* Core clock divider */
         .src = kSCG_SysClkSrcSirc     /* System clock source */
    };
    /* Init Sirc. */
    CLOCK_InitSirc(&scgSircConfig);
    /* Change to use SIRC as system clock source to prepare to change FIRCCFG register. */
    CLOCK_SetRunModeSysClkConfig(&sysClkSafeConfigSource);
    /* Wait for clock source switch finished. */
    do
    {
         CLOCK_GetCurSysClkConfig(&curConfig);
    } while (curConfig.src != sysClkSafeConfigSource.src);

    /* Init Firc. */
    CLOCK_InitFirc(fircConfig);
    /* Change back to use FIRC as system clock source in order to configure SIRC if needed. */
    sysClkSafeConfigSource.src = kSCG_SysClkSrcFirc;
    CLOCK_SetRunModeSysClkConfig(&sysClkSafeConfigSource);
    /* Wait for clock source switch finished. */
    do
    {
         CLOCK_GetCurSysClkConfig(&curConfig);
    } while (curConfig.src != sysClkSafeConfigSource.src);
}

/*******************************************************************************
 * Code for BOARD_BootClockRUN configuration
 ******************************************************************************/
void ke1xf_clock_init(int isExternal) {
    scg_sys_clk_config_t curConfig;
    
    if (isExternal) {
	    /* Init SOSC according to board configuration. */
	    CLOCK_InitSysOsc(&g_scgSysOscConfig_BOARD_BootClockRUN);
	    /* Set the XTAL0 frequency based on board settings. */
	    CLOCK_SetXtal0Freq(g_scgSysOscConfig_BOARD_BootClockRUN.freq);
	}
    /* Init FIRC. */
    CLOCK_CONFIG_FircSafeConfig(isExternal ? &g_scgFircConfig_EXT_BOARD_BootClockRUN : &g_scgFircConfig_INT_BOARD_BootClockRUN);
    /* Set HSRUN power mode. */
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeHsrun(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateHsrun)
    {
    }

    /* Init SIRC. */
    //CLOCK_InitSirc(&g_scgSircConfig_BOARD_BootClockRUN);
    /* Init SysPll. */
    CLOCK_InitSysPll(isExternal ? &g_scgSysPllConfig_EXT_BOARD_BootClockRUN : &g_scgSysPllConfig_INT_BOARD_BootClockRUN);
    /* Set SCG to SPLL mode. */
    CLOCK_SetHsrunModeSysClkConfig(&g_sysClkConfig_BOARD_BootClockRUN);
    /* Wait for clock source switch finished. */
    do
    {
         CLOCK_GetCurSysClkConfig(&curConfig);
    } while (curConfig.src != g_sysClkConfig_BOARD_BootClockRUN.src);
    /* Set SystemCoreClock variable. */
    //SystemCoreClock = KINETIS_PLL_FREQ;

    /* Set PCC ADC0 selection */
    CLOCK_SetIpSrc(kCLOCK_Adc0, kCLOCK_IpSrcSysPllAsync);
    /* Set PCC ADC1 selection */
    CLOCK_SetIpSrc(kCLOCK_Adc1, kCLOCK_IpSrcSysPllAsync);
    /* Set PCC ADC2 selection */
    CLOCK_SetIpSrc(kCLOCK_Adc2, kCLOCK_IpSrcSysPllAsync);
    /* Set PCC LPSPI0 selection */
    CLOCK_SetIpSrc(kCLOCK_Lpspi0, kCLOCK_IpSrcSysPllAsync);
    /* Set PCC LPSPI1 selection */
    CLOCK_SetIpSrc(kCLOCK_Lpspi1, kCLOCK_IpSrcSysPllAsync);
    /* Set PCC LPUART0 selection */
    CLOCK_SetIpSrc(kCLOCK_Lpuart0, kCLOCK_IpSrcSysPllAsync);
    /* Set PCC LPUART1 selection */
    CLOCK_SetIpSrc(kCLOCK_Lpuart1, kCLOCK_IpSrcSysPllAsync);
    /* Set PCC LPTMR0 selection */
    CLOCK_SetIpSrc(kCLOCK_Lptmr0, kCLOCK_IpSrcSysPllAsync);
    /* Set PCC LPIT0 selection */
    CLOCK_SetIpSrc(kCLOCK_Lpit0, kCLOCK_IpSrcSysPllAsync);
    /* Set PCC FTM0 selection */
    CLOCK_SetIpSrc(kCLOCK_Ftm0, kCLOCK_IpSrcSysPllAsync);
    /* Set PCC FTM1 selection */
    CLOCK_SetIpSrc(kCLOCK_Ftm1, kCLOCK_IpSrcSysPllAsync);
    /* Set PCC FTM2 selection */
    CLOCK_SetIpSrc(kCLOCK_Ftm2, kCLOCK_IpSrcSysPllAsync);
    /* Set PCC FTM3 selection */
    CLOCK_SetIpSrc(kCLOCK_Ftm3, kCLOCK_IpSrcSysPllAsync);
}
