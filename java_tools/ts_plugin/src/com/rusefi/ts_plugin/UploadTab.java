package com.rusefi.ts_plugin;

import com.efiAnalytics.plugin.ecu.ControllerAccess;
import com.efiAnalytics.plugin.ecu.ControllerException;
import com.efiAnalytics.plugin.ecu.ControllerParameterChangeListener;
import com.opensr5.ini.IniFileModel;
import com.opensr5.ini.field.IniField;
import com.rusefi.TsTuneReader;
import com.rusefi.autoupdate.AutoupdateUtil;
import com.rusefi.config.generated.Fields;
import com.rusefi.tools.online.Online;
import com.rusefi.tools.online.UploadResult;
import com.rusefi.ts_plugin.util.ManifestHelper;
import com.rusefi.tune.xml.Msq;
import com.rusefi.ui.AuthTokenPanel;
import com.rusefi.ui.util.URLLabel;
import org.apache.http.concurrent.FutureCallback;
import org.putgemin.VerticalFlowLayout;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.util.Date;
import java.util.Map;
import java.util.function.Supplier;

public class UploadTab {
    private final JComponent content = new JPanel(new VerticalFlowLayout());
    // 2 seconds aggregation by default
    private static final int AUTO_UPDATE_AGGREGATION = Integer.parseInt(System.getProperty("autoupload.aggregation", "2000"));

    private static final String REO_URL = "https://rusefi.com/online/";
    private final AuthTokenPanel tokenPanel = new AuthTokenPanel();

    private final Supplier<ControllerAccess> controllerAccessSupplier;

    private final UploadView uploadView = new UploadView();

    private final JButton upload = new JButton("Upload Current Tune");

    private String currentConfiguration;

    private UploaderStatus uploaderStatus = new UploaderStatus();

    private final Timer timer = new Timer(AUTO_UPDATE_AGGREGATION, new AbstractAction() {
        @Override
        public void actionPerformed(ActionEvent e) {
//            System.out.println("Timer! " + System.currentTimeMillis() + " " + timer + " " + e);
            if (UploadView.isAutoUpload()) {
                System.out.println(new Date() + ": enqueue tune");
                UploadQueue.enqueue(controllerAccessSupplier.get(), currentConfiguration);
            }
        }
    });

    private final ControllerParameterChangeListener listener;

    public UploadTab(Supplier<ControllerAccess> controllerAccessSupplier) {
        this.controllerAccessSupplier = controllerAccessSupplier;

        timer.stop();
        timer.setRepeats(false);
        UploadQueue.start();
        listener = parameterName -> {
            //            System.out.println("Parameter value changed " + parameterName);
            timer.restart();
        };
        upload.setBackground(new Color(0x90EE90));

        new Thread(new Runnable() {
            @Override
            public void run() {
                while (true) {
                    String configurationName = getConfigurationName();
                    if ((currentConfiguration == null && configurationName != null)
                            || !currentConfiguration.equals(configurationName)) {
                        handleConfigurationChange(configurationName, controllerAccessSupplier.get());
                    }

                    boolean isProjectActive = configurationName != null;
                    uploaderStatus.updateProjectStatus(configurationName, isProjectActive);

                    SwingUtilities.invokeLater(new Runnable() {
                        @Override
                        public void run() {

                            updateUploadEnabled();
                        }
                    });

                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }).start();

        upload.addActionListener(new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent e) {
                if (!tokenPanel.hasToken()) {
                    tokenPanel.showError(content);
                    return;
                }
                String configurationName = getConfigurationName();
                if (configurationName == null) {
                    JOptionPane.showMessageDialog(content, "No project opened");
                    return;
                }
                uploadView.uploadState.setVisible(true);
                uploadView.uploadState.setText("Uploading...");

                Msq tune = TuneUploder.writeCurrentTune(controllerAccessSupplier.get(), configurationName);
                Online.uploadTune(tune, tokenPanel, content, new FutureCallback<UploadResult>() {
                    @Override
                    public void completed(UploadResult array) {
                        SwingUtilities.invokeLater(new Runnable() {
                            @Override
                            public void run() {
                                uploadView.setResult(array);
                            }
                        });
                    }

                    @Override
                    public void failed(Exception e) {
                    }

                    @Override
                    public void cancelled() {
                    }
                });
            }
        });

        content.add(new JLabel("Version " + ManifestHelper.getBuildTimestamp()));
//        content.add(new JLabel("Active project: " + getConfigurationName()));


        content.add(uploadView.getContent());
        content.add(upload);

        ImageIcon LOGO = AutoupdateUtil.loadIcon("/rusefi_online_color_300.png");
        content.add(new JLabel(LOGO));
        content.add(tokenPanel.getContent());
        content.add(new URLLabel(REO_URL));

    }

    /**
     * This method is invoked every time we defect a switch between projects
     */
    private void handleConfigurationChange(String configurationName, ControllerAccess controllerAccess) {
        uploaderStatus.readTuneState(configurationName);

        if (configurationName != null) {
            subscribeToUpdates(configurationName, controllerAccess);
        }

        updateUploadEnabled();

        currentConfiguration = configurationName;
    }

    private void updateUploadEnabled() {
        uploadView.update(uploaderStatus);

        upload.setEnabled(uploaderStatus.isTuneOk() && uploaderStatus.isProjectIsOk());
    }

    private void subscribeToUpdates(String configurationName, ControllerAccess controllerAccess) {
        IniFileModel model = new IniFileModel().readIniFile(TsTuneReader.getProjectModeFileName(configurationName));
        Map<String, IniField> allIniFields = model.allIniFields;
        if (model.allIniFields == null)
            return;
        for (Map.Entry<String, IniField> field : allIniFields.entrySet()) {
            boolean isOnlineTuneField = field.getValue().getOffset() >= Fields.engine_configuration_s_size;
            if (!isOnlineTuneField) {
                try {
                    controllerAccess.getControllerParameterServer().subscribe(configurationName, field.getKey(), listener);
                } catch (ControllerException e) {
                    throw new IllegalStateException(e);
                }
            }
        }
    }

    private String getConfigurationName() {
        ControllerAccess controllerAccess = controllerAccessSupplier.get();
        if (controllerAccess == null) {
            System.out.println("No ControllerAccess");
            return null;
        }
        String[] configurationNames = controllerAccess.getEcuConfigurationNames();
        if (configurationNames.length == 0)
            return null;
        return configurationNames[0];
    }

    public JComponent getContent() {
        return content;
    }
}
