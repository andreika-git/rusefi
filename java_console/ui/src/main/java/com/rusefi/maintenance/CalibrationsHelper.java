package com.rusefi.maintenance;

import com.devexperts.logging.Logging;
import com.opensr5.ConfigurationImage;
import com.opensr5.ConfigurationImageMetaVersion0_0;
import com.opensr5.ConfigurationImageWithMeta;
import com.opensr5.ini.IniFileModel;
import com.opensr5.ini.field.*;
import com.rusefi.SerialPortScanner.PortResult;
import com.rusefi.binaryprotocol.BinaryProtocol;
import com.rusefi.core.Pair;
import com.rusefi.io.UpdateOperationCallbacks;
import com.rusefi.tune.xml.Constant;
import com.rusefi.tune.xml.Msq;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

import static com.devexperts.logging.Logging.getLogging;
import static com.rusefi.binaryprotocol.BinaryProtocol.iniFileProvider;
import static com.rusefi.binaryprotocol.BinaryProtocol.saveConfigurationImageToFiles;
import static com.rusefi.maintenance.IniFieldsAnalizer.findValuesToUpdate;
import static java.nio.file.StandardCopyOption.REPLACE_EXISTING;

public class CalibrationsHelper {
    private static final Logging log = getLogging(CalibrationsHelper.class);

    static Optional<CalibrationsInfo> readCalibrationsInfo(
        final BinaryProtocol binaryProtocol,
        final UpdateOperationCallbacks callbacks
    ) {
        try {
            final String signature = BinaryProtocol.getSignature(binaryProtocol.getStream());
            callbacks.logLine(String.format("Received a signature %s", signature));
            final IniFileModel iniFile = iniFileProvider.provide(signature);
            final int pageSize = iniFile.getMetaInfo().getTotalSize();
            callbacks.logLine(String.format("Page size is %d", pageSize));
            final ConfigurationImageMetaVersion0_0 meta = new ConfigurationImageMetaVersion0_0(pageSize, signature);
            callbacks.logLine("Reading current calibrations...");
            final ConfigurationImageWithMeta image = binaryProtocol.readFullImageFromController(meta);
            return Optional.of(new CalibrationsInfo(iniFile, image));
        } catch (final IOException e) {
            log.error("Failed to read meta:", e);
            callbacks.logLine("Failed to read meta");
            return Optional.empty();
        }
    }

    static boolean backUpCalibrationsInfo(
        final CalibrationsInfo calibrationsInfo,
        final String fileName,
        final UpdateOperationCallbacks callbacks
    ) {
        try {
            final String iniFileName = String.format("%s.ini", fileName);
            final IniFileModel ini = calibrationsInfo.getIniFile();
            final Path iniFilePath = Paths.get(ini.getIniFilePath());
            callbacks.logLine(String.format("Backing up current ini-file `%s`...", iniFilePath));
            Files.copy(
                iniFilePath,
                Paths.get(iniFileName),
                REPLACE_EXISTING
            );
            callbacks.logLine(String.format(
                "`%s` ini-file is backed up as `%s`",
                iniFilePath.getFileName(),
                iniFileName
            ));
            final String zipFileName = String.format("%s.zip", fileName);
            final String msqFileName = String.format("%s.msq", fileName);
            callbacks.logLine(String.format(
                "Backing up calibrations to files `%s` and `%s`...",
                zipFileName,
                msqFileName
            ));
            saveConfigurationImageToFiles(calibrationsInfo.getImage(), ini, zipFileName, msqFileName);
            callbacks.logLine(String.format(
                "Calibrations are backed up to files `%s` and `%s`",
                zipFileName,
                msqFileName
            ));
            return true;
        } catch (final Exception e) {
            log.error("Backing up calibrations failed:", e);
            callbacks.logLine("Backing up current calibrations failed");
            return false;
        }
    }

    public static Optional<CalibrationsInfo> readAndBackupCurrentCalibrations(
        final PortResult ecuPort,
        final UpdateOperationCallbacks callbacks,
        final String backupFileName
    ) {
        return BinaryProtocolExecutor.executeWithSuspendedPortScanner(
            ecuPort.port,
            callbacks,
            (binaryProtocol) -> {
                try {
                    final Optional<CalibrationsInfo> calibrationsInfo = readCalibrationsInfo(binaryProtocol, callbacks);
                    if (calibrationsInfo.isPresent()) {
                        final CalibrationsInfo receivedCalibrations = calibrationsInfo.get();
                        if (backUpCalibrationsInfo(
                            receivedCalibrations,
                            backupFileName,
                            callbacks
                        )) {
                            return calibrationsInfo;
                        }
                    }
                    return Optional.empty();
                } catch (final Exception e) {
                    log.error("Back up current calibrations failed:", e);
                    callbacks.logLine("Back up current calibrations failed");
                    return Optional.empty();
                }
            },
            Optional.empty(),
            false
        );
    }

    public static CalibrationsInfo mergeCalibrations(
        final CalibrationsInfo prevCalibrations,
        final CalibrationsInfo newCalibrations,
        final UpdateOperationCallbacks callbacks
    ) {
        final IniFileModel prevIniFile = prevCalibrations.getIniFile();
        final Msq prevMsq = prevCalibrations.generateMsq();
        final IniFileModel newIniFile = newCalibrations.getIniFile();
        final Msq newMsq = newCalibrations.generateMsq();

        final List<Pair<IniField, Constant>> valuesToUpdate = findValuesToUpdate(
            prevIniFile,
            prevMsq.getConstantsAsMap(),
            newIniFile,
            newMsq.getConstantsAsMap()
        );

        final ConfigurationImage mergedImage = newCalibrations.getImage().getConfigurationImage().clone();
        for (final Pair<IniField, Constant> valueToUpdate: valuesToUpdate) {
            final IniField fieldToUpdate = valueToUpdate.first;
            final Constant value = valueToUpdate.second;
            fieldToUpdate.setValue(mergedImage, value);
            callbacks.logLine(String.format("Field `%s` is set to %s", fieldToUpdate.getName(), value.getValue()));
        }
        return new CalibrationsInfo(
            newIniFile,
            new ConfigurationImageWithMeta(newCalibrations.getImage().getMeta(), mergedImage.getContent())
        );
    }
}
