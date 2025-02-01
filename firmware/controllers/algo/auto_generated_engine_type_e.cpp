#include "global.h"
#include "engine_type_e.h"
// was generated automatically by rusEFI tool  from engine_type_e.h // by enum_to_string.jar tool on Sat Feb 01 01:29:30 UTC 2025
// see also gen_config_and_enums.bat



const char *getEngine_type_e(engine_type_e value){
switch(value) {
case engine_type_e::BMW_M52:
  return "BMW_M52";
case engine_type_e::BMW_M73_MRE:
  return "BMW_M73_MRE";
case engine_type_e::BMW_M73_MRE_SLAVE:
  return "BMW_M73_MRE_SLAVE";
case engine_type_e::DEFAULT_FRANKENSO:
  return "DEFAULT_FRANKENSO";
case engine_type_e::DISCOVERY_PDM:
  return "DISCOVERY_PDM";
case engine_type_e::DODGE_NEON_1995:
  return "DODGE_NEON_1995";
case engine_type_e::DODGE_NEON_2003_CRANK:
  return "DODGE_NEON_2003_CRANK";
case engine_type_e::DODGE_RAM:
  return "DODGE_RAM";
case engine_type_e::EEPROM_BENCH_ENGINE:
  return "EEPROM_BENCH_ENGINE";
case engine_type_e::ETB_BENCH_ENGINE:
  return "ETB_BENCH_ENGINE";
case engine_type_e::ET_BOSCH_QUICK_START:
  return "ET_BOSCH_QUICK_START";
case engine_type_e::ET_TEST_WITH_BOSCH_QUICK_START:
  return "ET_TEST_WITH_BOSCH_QUICK_START";
case engine_type_e::ET_UNUSED_56:
  return "ET_UNUSED_56";
case engine_type_e::FERRARI_F136:
  return "FERRARI_F136";
case engine_type_e::FORD_ASPIRE_1996:
  return "FORD_ASPIRE_1996";
case engine_type_e::FORD_COYOTE:
  return "FORD_COYOTE";
case engine_type_e::FORD_ESCORT_GT:
  return "FORD_ESCORT_GT";
case engine_type_e::FORD_INLINE_6_1995:
  return "FORD_INLINE_6_1995";
case engine_type_e::FRANKENSO_BMW_M73_F:
  return "FRANKENSO_BMW_M73_F";
case engine_type_e::FRANKENSO_MIATA_NA6_MAP:
  return "FRANKENSO_MIATA_NA6_MAP";
case engine_type_e::FRANKENSO_TEST_33810:
  return "FRANKENSO_TEST_33810";
case engine_type_e::FUEL_BENCH:
  return "FUEL_BENCH";
case engine_type_e::GM_LCV:
  return "GM_LCV";
case engine_type_e::GM_LTG:
  return "GM_LTG";
case engine_type_e::GM_SBC:
  return "GM_SBC";
case engine_type_e::GM_SBC_GEN5:
  return "GM_SBC_GEN5";
case engine_type_e::GY6_139QMB:
  return "GY6_139QMB";
case engine_type_e::HARLEY:
  return "HARLEY";
case engine_type_e::HELLEN_121_NISSAN_4_CYL:
  return "HELLEN_121_NISSAN_4_CYL";
case engine_type_e::HELLEN_121_NISSAN_6_CYL:
  return "HELLEN_121_NISSAN_6_CYL";
case engine_type_e::HELLEN_121_NISSAN_8_CYL:
  return "HELLEN_121_NISSAN_8_CYL";
case engine_type_e::HELLEN_121_NISSAN_ALMERA_N16:
  return "HELLEN_121_NISSAN_ALMERA_N16";
case engine_type_e::HELLEN_121_VAG_4_CYL:
  return "HELLEN_121_VAG_4_CYL";
case engine_type_e::HELLEN_121_VAG_8_CYL:
  return "HELLEN_121_VAG_8_CYL";
case engine_type_e::HELLEN_121_VAG_V6_CYL:
  return "HELLEN_121_VAG_V6_CYL";
case engine_type_e::HELLEN_121_VAG_VR6_CYL:
  return "HELLEN_121_VAG_VR6_CYL";
case engine_type_e::HELLEN_128_MERCEDES_4_CYL:
  return "HELLEN_128_MERCEDES_4_CYL";
case engine_type_e::HELLEN_128_MERCEDES_6_CYL:
  return "HELLEN_128_MERCEDES_6_CYL";
case engine_type_e::HELLEN_128_MERCEDES_8_CYL:
  return "HELLEN_128_MERCEDES_8_CYL";
case engine_type_e::HELLEN_154_HYUNDAI_COUPE_BK1:
  return "HELLEN_154_HYUNDAI_COUPE_BK1";
case engine_type_e::HELLEN_154_HYUNDAI_COUPE_BK2:
  return "HELLEN_154_HYUNDAI_COUPE_BK2";
case engine_type_e::HELLEN_154_VAG:
  return "HELLEN_154_VAG";
case engine_type_e::HELLEN_2CHAN_STIM_QC:
  return "HELLEN_2CHAN_STIM_QC";
case engine_type_e::HELLEN_4CHAN_STIM_QC:
  return "HELLEN_4CHAN_STIM_QC";
case engine_type_e::HELLEN_HONDA_BCM:
  return "HELLEN_HONDA_BCM";
case engine_type_e::HONDA_600:
  return "HONDA_600";
case engine_type_e::HONDA_K:
  return "HONDA_K";
case engine_type_e::HONDA_OBD1:
  return "HONDA_OBD1";
case engine_type_e::HONDA_OBD2A:
  return "HONDA_OBD2A";
case engine_type_e::HYUNDAI_PB:
  return "HYUNDAI_PB";
case engine_type_e::L9779_BENCH_ENGINE:
  return "L9779_BENCH_ENGINE";
case engine_type_e::MAVERICK_X3:
  return "MAVERICK_X3";
case engine_type_e::MAZDA_MIATA_NA6:
  return "MAZDA_MIATA_NA6";
case engine_type_e::MAZDA_MIATA_NA94:
  return "MAZDA_MIATA_NA94";
case engine_type_e::MAZDA_MIATA_NA96:
  return "MAZDA_MIATA_NA96";
case engine_type_e::MAZDA_MIATA_NB1:
  return "MAZDA_MIATA_NB1";
case engine_type_e::MAZDA_MIATA_NB2:
  return "MAZDA_MIATA_NB2";
case engine_type_e::MAZDA_MIATA_NB2_36:
  return "MAZDA_MIATA_NB2_36";
case engine_type_e::MAZDA_MIATA_NC:
  return "MAZDA_MIATA_NC";
case engine_type_e::ME17_9_MISC:
  return "ME17_9_MISC";
case engine_type_e::MERCEDES_M111:
  return "MERCEDES_M111";
case engine_type_e::MIATA_PROTEUS_TCU:
  return "MIATA_PROTEUS_TCU";
case engine_type_e::MINIMAL_PINS:
  return "MINIMAL_PINS";
case engine_type_e::MITSUBISHI_3A92:
  return "MITSUBISHI_3A92";
case engine_type_e::MITSUBISHI_4G93:
  return "MITSUBISHI_4G93";
case engine_type_e::MRE_BOARD_NEW_TEST:
  return "MRE_BOARD_NEW_TEST";
case engine_type_e::MRE_BODY_CONTROL:
  return "MRE_BODY_CONTROL";
case engine_type_e::MRE_SECONDARY_CAN:
  return "MRE_SECONDARY_CAN";
case engine_type_e::MRE_SUBARU_EJ18:
  return "MRE_SUBARU_EJ18";
case engine_type_e::MRE_VW_B6:
  return "MRE_VW_B6";
case engine_type_e::NISSAN_PRIMERA:
  return "NISSAN_PRIMERA";
case engine_type_e::POLARIS:
  return "POLARIS";
case engine_type_e::POLARIS_RZR:
  return "POLARIS_RZR";
case engine_type_e::PROTEUS_ANALOG_PWM_TEST:
  return "PROTEUS_ANALOG_PWM_TEST";
case engine_type_e::PROTEUS_BMW_M73:
  return "PROTEUS_BMW_M73";
case engine_type_e::PROTEUS_GM_LS_4:
  return "PROTEUS_GM_LS_4";
case engine_type_e::PROTEUS_LUA_DEMO:
  return "PROTEUS_LUA_DEMO";
case engine_type_e::PROTEUS_NISSAN_VQ35:
  return "PROTEUS_NISSAN_VQ35";
case engine_type_e::PROTEUS_QC_TEST_BOARD:
  return "PROTEUS_QC_TEST_BOARD";
case engine_type_e::PROTEUS_STIM_QC:
  return "PROTEUS_STIM_QC";
case engine_type_e::PROTEUS_VW_B6:
  return "PROTEUS_VW_B6";
case engine_type_e::SACHS:
  return "SACHS";
case engine_type_e::SIMULATOR_CONFIG:
  return "SIMULATOR_CONFIG";
case engine_type_e::SUBARU_EG33:
  return "SUBARU_EG33";
case engine_type_e::TCU_4R70W:
  return "TCU_4R70W";
case engine_type_e::TEST_100:
  return "TEST_100";
case engine_type_e::TEST_101:
  return "TEST_101";
case engine_type_e::TEST_33816:
  return "TEST_33816";
case engine_type_e::TEST_CRANK_ENGINE:
  return "TEST_CRANK_ENGINE";
case engine_type_e::TEST_DC_WASTEGATE_DISCOVERY:
  return "TEST_DC_WASTEGATE_DISCOVERY";
case engine_type_e::TEST_ENGINE:
  return "TEST_ENGINE";
case engine_type_e::TEST_ENGINE_VVT:
  return "TEST_ENGINE_VVT";
case engine_type_e::TEST_ISSUE_366_BOTH:
  return "TEST_ISSUE_366_BOTH";
case engine_type_e::TEST_ISSUE_366_RISE:
  return "TEST_ISSUE_366_RISE";
case engine_type_e::TEST_ISSUE_6451:
  return "TEST_ISSUE_6451";
case engine_type_e::TEST_ROTARY:
  return "TEST_ROTARY";
case engine_type_e::TOYOTA_1NZ_FE:
  return "TOYOTA_1NZ_FE";
case engine_type_e::TOYOTA_2JZ_GTE_VVTi:
  return "TOYOTA_2JZ_GTE_VVTi";
case engine_type_e::UNUSED102:
  return "UNUSED102";
case engine_type_e::UNUSED67:
  return "UNUSED67";
case engine_type_e::UNUSED94:
  return "UNUSED94";
case engine_type_e::UNUSED_65:
  return "UNUSED_65";
case engine_type_e::UNUSED_97:
  return "UNUSED_97";
case engine_type_e::VAG_5_CYL:
  return "VAG_5_CYL";
case engine_type_e::VW_ABA:
  return "VW_ABA";
case engine_type_e::WASTEGATE_PROTEUS_TEST:
  return "WASTEGATE_PROTEUS_TEST";
  }
 return NULL;
}
