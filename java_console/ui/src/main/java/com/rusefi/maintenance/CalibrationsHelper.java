package com.rusefi.maintenance;

import com.devexperts.logging.Logging;
import com.opensr5.ConfigurationImage;
import com.opensr5.ConfigurationImageMetaVersion0_0;
import com.opensr5.ConfigurationImageWithMeta;
import com.opensr5.ini.IniFileModel;
import com.opensr5.ini.field.*;
import com.rusefi.SerialPortScanner.PortResult;
import com.rusefi.binaryprotocol.BinaryProtocol;
import com.rusefi.binaryprotocol.BinaryProtocolLocalCache;
import com.rusefi.core.Pair;
import com.rusefi.core.ui.AutoupdateUtil;
import com.rusefi.io.UpdateOperationCallbacks;
import com.rusefi.tune.xml.Constant;
import com.rusefi.tune.xml.Msq;

import javax.swing.*;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.function.Supplier;

import static com.devexperts.logging.Logging.getLogging;
import static com.rusefi.binaryprotocol.BinaryProtocol.iniFileProvider;
import static com.rusefi.binaryprotocol.BinaryProtocol.saveConfigurationImageToFiles;
import static com.rusefi.maintenance.IniFieldsAnalizer.findValuesToUpdate;
import static java.nio.file.StandardCopyOption.REPLACE_EXISTING;

public class CalibrationsHelper {
    private static final Logging log = getLogging(CalibrationsHelper.class);

    private static final String PREVIOUS_CALIBRATIONS_FILE_NAME = BinaryProtocolLocalCache.STATE_FOLDER + "prev_calibrations";
    private static final String UPDATED_CALIBRATIONS_FILE_NAME = BinaryProtocolLocalCache.STATE_FOLDER + "updated_calibrations";

    public static boolean updateFirmwareAndRestorePreviousCalibrations(
        final JComponent parent,
        final PortResult ecuPort,
        final UpdateOperationCallbacks callbacks,
        final Supplier<Boolean> updateFirmware
    ) {
        AutoupdateUtil.assertNotAwtThread();

        final Optional<CalibrationsInfo> prevCalibrations = readAndBackupCurrentCalibrations(
            ecuPort,
            callbacks,
            PREVIOUS_CALIBRATIONS_FILE_NAME
        );
        if (prevCalibrations.isEmpty()) {
            callbacks.logLine("Failed to back up current calibrations...");
            return false;
        }

        if (!updateFirmware.get()) {
            return false;
        }

        final Optional<CalibrationsInfo> updatedCalibrations = readAndBackupCurrentCalibrations(
            ecuPort,
            callbacks,
            UPDATED_CALIBRATIONS_FILE_NAME
        );
        if (updatedCalibrations.isEmpty()) {
            callbacks.logLine("Failed to back up updated calibrations...");
            return false;
        }
        final Optional<CalibrationsInfo> mergedCalibrations = mergeCalibrations(
            prevCalibrations.get(),
            updatedCalibrations.get(),
            callbacks
        );
        if (mergedCalibrations.isPresent() && (JOptionPane.showConfirmDialog(
            parent,
            "Some calibrations fields were overwritten with default values.\n" +
                "Would you like to restore previous calibrations?",
            "Restore previous calibrations",
            JOptionPane.YES_NO_OPTION
        ) == JOptionPane.YES_OPTION)) {
            if (!backUpCalibrationsInfo(mergedCalibrations.get(), "merged_calibrations", callbacks)) {
                callbacks.logLine("Failed to back up merged calibrations...");
                return false;
            }
            return CalibrationsUpdater.INSTANCE.updateCalibrations(
                ecuPort.port,
                mergedCalibrations.get().getImage().getConfigurationImage(),
                callbacks,
                false
            );
        } else {
            return true;
        }
    }

    private static Optional<CalibrationsInfo> readCalibrationsInfo(
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

    private static boolean backUpCalibrationsInfo(
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
            callbacks.logLine("Backing up current calibrations failed: " + e);
            return false;
        }
    }

    private static Optional<CalibrationsInfo> readAndBackupCurrentCalibrations(
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

    private static Optional<CalibrationsInfo> mergeCalibrations(
        final CalibrationsInfo prevCalibrations,
        final CalibrationsInfo newCalibrations,
        final UpdateOperationCallbacks callbacks
    ) {
        Optional<CalibrationsInfo> result = Optional.empty();
        final IniFileModel prevIniFile = prevCalibrations.getIniFile();
        final Msq prevMsq = prevCalibrations.generateMsq();
        final IniFileModel newIniFile = newCalibrations.getIniFile();
        final Msq newMsq = newCalibrations.generateMsq();

        final List<Pair<IniField, Constant>> valuesToUpdate = findValuesToUpdate(
            prevIniFile,
            prevMsq.getConstantsAsMap(),
            newIniFile,
            newMsq.getConstantsAsMap(),
            callbacks
        );
        if (!valuesToUpdate.isEmpty()) {
            final ConfigurationImage mergedImage = newCalibrations.getImage().getConfigurationImage().clone();
            for (final Pair<IniField, Constant> valueToUpdate : valuesToUpdate) {
                final IniField fieldToUpdate = valueToUpdate.first;
                final Constant value = valueToUpdate.second;
                fieldToUpdate.setValue(mergedImage, value);
                callbacks.logLine(String.format(
                    "To restore previous calibrations we are going to update the field `%s` with a value %s",
                    fieldToUpdate.getName(),
                    value.getValue()
                ));
            }
            result = Optional.of(new CalibrationsInfo(
                newIniFile,
                new ConfigurationImageWithMeta(newCalibrations.getImage().getMeta(), mergedImage.getContent())
            ));
        } else {
            callbacks.logLine("It looks like we do not need to update any fields to restore previous calibrations.");
        }
        return result;
    }
}
