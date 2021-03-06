package com.rusefi.io;

import com.opensr5.ConfigurationImage;
import com.rusefi.FileLog;
import com.rusefi.binaryprotocol.BinaryProtocolState;
import com.rusefi.config.generated.Fields;
import com.rusefi.io.LinkConnector;
import com.rusefi.io.LinkManager;
import com.rusefi.io.tcp.BinaryProtocolServer;

class BinaryProtocolServerSandbox {
    public static void main(String[] args) {
        BinaryProtocolState state = new BinaryProtocolState();
        state.setController(new ConfigurationImage(new byte[Fields.TOTAL_CONFIG_SIZE]));
        state.setCurrentOutputs(new byte[1 + Fields.TS_OUTPUT_SIZE]);

        LinkManager linkManager = new LinkManager(FileLog.LOGGER);
        linkManager.setConnector(LinkConnector.getDetachedConnector(state));
        new BinaryProtocolServer(FileLog.LOGGER).start(linkManager);
    }
}