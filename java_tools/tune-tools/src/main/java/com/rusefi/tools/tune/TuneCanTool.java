package com.rusefi.tools.tune;

import com.devexperts.logging.Logging;
import com.opensr5.ini.DialogModel;
import com.opensr5.ini.IniFileModel;
import com.rusefi.*;
import com.rusefi.config.generated.Fields;
import com.rusefi.core.preferences.storage.Node;
import com.rusefi.tune.xml.Constant;
import com.rusefi.tune.xml.Msq;
import org.jetbrains.annotations.NotNull;

import javax.xml.bind.JAXBException;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.Objects;

import static com.devexperts.logging.Logging.getLogging;
import static com.rusefi.ConfigFieldImpl.unquote;
import static com.rusefi.config.Field.niceToString;
import static com.rusefi.tools.tune.WriteSimulatorConfiguration.INI_FILE_FOR_SIMULATOR;

/**
 * this command line utility compares two TS calibration files and produces .md files with C++ source code of the difference between those two files.
 * <p>
 * Base 'default settings' file is msq generated by WriteSimulatorConfiguration.java with .xml extension but a real .msq could probably be used instead.
 * Second calibration file which contains desired base calibrations is a native TS calibration file.
 * <p>
 * [CannedTunes]
 * <p>
 * see <a href="https://github.com/rusefi/rusefi/wiki/Canned-Tune-Process">...</a>
 */
public class TuneCanTool {
    private static final Logging log = getLogging(TuneCanTool.class);
    private static final String REPORTS_OUTPUT_FOLDER = "tune_reports";

    public static final String SRC_TEST_RESOURCES = "src/test/resources/";
    private static final String FOLDER = "generated";
    public static final String SIMULATED_PREFIX = FOLDER + File.separator + "simulator_tune";
    public static final String SIMULATED_SUFFIX = ".xml";
    public static final String DEFAULT_TUNE = SIMULATED_PREFIX + SIMULATED_SUFFIX;
    private static final String workingFolder = "downloaded_tunes";

    private static IniFileModel ini;


    public static void main(String[] args) throws Exception {
        ini = new IniFileModel().readIniFile(INI_FILE_FOR_SIMULATOR);
        if (ini == null)
            throw new IllegalStateException("Not found " + INI_FILE_FOR_SIMULATOR);

        RootHolder.ROOT = "../firmware/";

//        writeDiffBetweenLocalTuneFileAndDefaultTune("example.msq");

//        writeDiffBetweenLocalTuneFileAndDefaultTune("vehicleName", getDefaultTuneName(Fields.engine_type_e_MAVERICK_X3),
//            "C:\\stuff\\i\\canam-2022-short\\canam-progress-pnp-dec-29.msq",  "comment");


//        processREOtune(1507, Fields.engine_type_e_HELLEN_154_HYUNDAI_COUPE_BK2, "BK2");
        processREOtune(1502, Fields.engine_type_e_HYUNDAI_PB, "PB");
//        processREOtune(1490, Fields.engine_type_e_MRE_M111, "m111-alex");
//        handle("Mitsubicha", 1258);
//        handle("Scion-1NZ-FE", 1448);
//        handle("4g93", 1425);
//        handle("BMW-mtmotorsport", 1479);
    }

  /**
   * @see WriteSimulatorConfiguration
   */
    private static void processREOtune(int tuneId, int engineType, String key) throws JAXBException, IOException {
        // compare specific internet tune to total global default
        handle(key, tuneId, TuneCanTool.DEFAULT_TUNE);
        // compare same internet tune to default tune of specified engine type
        handle(key + "-diff", tuneId, getDefaultTuneName(engineType));
    }

    @NotNull
    private static String getDefaultTuneName(int engineType) {
        return SIMULATED_PREFIX + "_" + engineType + SIMULATED_SUFFIX;
    }

    private static void handle(String vehicleName, int tuneId, String defaultTuneFileName) throws JAXBException, IOException {
        String customTuneFileName = workingFolder + File.separator + tuneId + ".msq";
        String url = "https://rusefi.com/online/view.php?msq=" + tuneId;

        downloadTune(tuneId, customTuneFileName);

        writeDiffBetweenLocalTuneFileAndDefaultTune(vehicleName, defaultTuneFileName, customTuneFileName, url);
    }

//    private static void writeDiffBetweenLocalTuneFileAndDefaultTune(String localFileName) throws JAXBException, IOException {
//        writeDiffBetweenLocalTuneFileAndDefaultTune("vehicleName", DEFAULT_TUNE,
//            localFileName,  "comment");
//    }
//
//    private static void writeDiffBetweenLocalTuneFileAndDefaultTune(int engineCode, String localFileName, String cannedComment) throws JAXBException, IOException {
//        writeDiffBetweenLocalTuneFileAndDefaultTune("vehicleName", getDefaultTuneName(engineCode),
//            localFileName,  cannedComment);
//    }

    private static void writeDiffBetweenLocalTuneFileAndDefaultTune(String vehicleName, String defaultTuneFileName, String customTuneFileName, String cannedComment) throws JAXBException, IOException {
        new File(REPORTS_OUTPUT_FOLDER).mkdir();

        Msq customTune = Msq.readTune(customTuneFileName);
        Msq defaultTune = Msq.readTune(defaultTuneFileName);

        StringBuilder methods = new StringBuilder();

        StringBuilder sb = TuneCanTool.getTunePatch(defaultTune, customTune, ini, customTuneFileName, methods);

        String fileNameMethods = REPORTS_OUTPUT_FOLDER + "/" + vehicleName + "_methods.md";
        try (FileWriter methodsWriter = new FileWriter(fileNameMethods)) {
            methodsWriter.append(methods);
        }

        String fileName = REPORTS_OUTPUT_FOLDER + "/" + vehicleName + ".md";
        File outputFile = new File(fileName);
        log.info("Writing to " + outputFile.getAbsolutePath());

        try (FileWriter w = new FileWriter(outputFile)) {
            w.append("# " + vehicleName + "\n\n");
            w.append("// canned tune " + cannedComment + "\n\n");

            w.append("```\n");
            w.append(sb);
            w.append("```\n");
        }
    }

  private static void downloadTune(int tuneId, String localFileName) throws IOException {
        new File(workingFolder).mkdirs();
        String downloadUrl = "https://rusefi.com/online/download.php?msq=" + tuneId;
        InputStream in = new URL(downloadUrl).openStream();
        Files.copy(in, Paths.get(localFileName), StandardCopyOption.REPLACE_EXISTING);
    }

    private static boolean isHardwareEnum(String type) {
        switch (type) {
            case "output_pin_e":
            case "brain_input_pin_e":
            case "adc_channel_e":
            case "Gpio":
            case "spi_device_e":
            case "pin_input_mode_e":
            case "pin_output_mode_e":
                return true;
        }
        return false;
    }

    private static Object simplerSpaces(String value) {
        if (value == null)
            return value;
        return value.replaceAll("\\s+", " ").trim();
    }

    @NotNull
    public static StringBuilder getTunePatch(Msq defaultTune, Msq customTune, IniFileModel ini, String currentTuneFileName, StringBuilder methods) throws IOException {
        ReaderStateImpl state = MetaHelper.getReaderState();

        StringBuilder invokeMethods = new StringBuilder();


        StringBuilder sb = new StringBuilder();
        for (DialogModel.Field f : ini.fieldsInUiOrder.values()) {
            String name = f.getKey();
            Constant customValue = customTune.getConstantsAsMap().get(name);
            Constant defaultValue = defaultTune.getConstantsAsMap().get(name);
            if (defaultValue == null) {
                // no longer present
                continue;
            }
            Objects.requireNonNull(defaultValue.getValue(), "d value");
            if (customValue == null) {
                log.info("Skipping " + name + " not present in tune");
                continue;
            }
            Objects.requireNonNull(customValue.getValue(), "c value");

            boolean isSameValue = simplerSpaces(defaultValue.getValue()).equals(simplerSpaces(customValue.getValue()));
            if (!isSameValue) {
                // todo: what about stuff outside of engine_configuration_s?
                StringBuffer context = new StringBuffer();

                ConfigField cf = MetaHelper.findField(state, name, context);
                if (cf == null) {
                    log.info("Not found " + name);
                    continue;
                }
                String cName = context + cf.getOriginalArrayName();

                if (cf.getType().equals("boolean")) {
                    sb.append(TuneTools.getAssignmentCode(defaultValue, cName, unquote(customValue.getValue())));
                    continue;
                }

                if (cf.isArray()) {
                    String parentReference;
                    if (cf.getParent().getName().equals("engine_configuration_s")) {
                        parentReference = "engineConfiguration->";
                    } else if (cf.getParent().getName().equals("persistent_config_s")) {
                        parentReference = "config->";
                    } else {
                        // todo: for instance map.samplingAngle
                        //throw new IllegalStateException("Unexpected " + cf.getParent());
                        System.out.println(" " + cf);
                        continue;
                    }


                    if (cf.getArraySizes().length == 2) {
                        TableData tableData = TableData.readTable(currentTuneFileName, name, ini);
                        if (tableData == null)
                            continue;
                        System.out.printf(" " + name);

                        methods.append(tableData.getCsourceMethod(parentReference));
                        invokeMethods.append(tableData.getCinvokeMethod());
                        continue;
                    }

                    CurveData data = CurveData.valueOf(currentTuneFileName, name, ini);
                    if (data == null)
                        continue;


                    methods.append(data.getCsourceMethod(parentReference));
                    invokeMethods.append(data.getCinvokeMethod());

                    continue;
                }


                if (!Node.isNumeric(customValue.getValue())) {
                    // todo: smarter logic for enums

                    String type = cf.getType();
                    if (isHardwareEnum(type)) {
                        continue;
                    }
                    EnumsReader.EnumState sourceCodeEnum = state.getEnumsReader().getEnums().get(type);
                    if (sourceCodeEnum == null) {
                        log.info("No info for " + type);
                        continue;
                    }
                    String customEnum = state.getTsCustomLine().get(type);

                    int ordinal;
                    try {
                        ordinal = TuneTools.resolveEnumByName(customEnum, unquote(customValue.getValue()));
                    } catch (IllegalStateException e) {
                        log.info("Looks like things were renamed: " + customValue.getValue() + " not found in " + customEnum);
                        continue;
                    }

                    log.info(cf + " " + sourceCodeEnum + " " + customEnum + " " + ordinal);

                    String sourceCodeValue = sourceCodeEnum.findByValue(ordinal);
                    sb.append(TuneTools.getAssignmentCode(defaultValue, cName, sourceCodeValue));

                    continue;
                }
                double doubleValue = Double.valueOf(customValue.getValue());
                int intValue = (int) doubleValue;
                boolean isInteger = intValue == doubleValue;
                if (isInteger) {
                    sb.append(TuneTools.getAssignmentCode(defaultValue, cName, Integer.toString(intValue)));
                } else {
                    sb.append(TuneTools.getAssignmentCode(defaultValue, cName, niceToString(doubleValue)));
                }
            }
        }
        sb.append("\n\n").append(invokeMethods);

        return sb;
    }

}
