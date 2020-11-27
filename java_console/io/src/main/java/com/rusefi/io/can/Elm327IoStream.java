package com.rusefi.io.can;

import com.opensr5.io.DataListener;
import com.rusefi.binaryprotocol.IncomingDataBuffer;
import com.rusefi.io.ByteReader;
import com.rusefi.io.serial.AbstractIoStream;
import com.rusefi.shared.FileUtil;
import org.jetbrains.annotations.NotNull;

import java.io.*;
import java.nio.ByteBuffer;
import java.util.Arrays;

public class Elm327IoStream extends AbstractIoStream {
    private final String loggingPrefix;
    private final Elm327Connector con;
    private final DisconnectListener disconnectListener;
    @NotNull
    private final IncomingDataBuffer dataBuffer;

    // the buffer size is limited by CAN-TP protocol
    private final static int OUT_BUFFER_SIZE = 4095;
    private ByteBuffer outBuf;

    public Elm327IoStream(Elm327Connector con, String loggingPrefix) throws IOException {
        this(con, loggingPrefix, DisconnectListener.VOID);
    }

    public Elm327IoStream(Elm327Connector con, String loggingPrefix, DisconnectListener disconnectListener) throws IOException {
        this.con = con;
        this.loggingPrefix = loggingPrefix;
        this.disconnectListener = disconnectListener;
        this.dataBuffer = IncomingDataBuffer.createDataBuffer(loggingPrefix, this);

        outBuf = ByteBuffer.allocate(OUT_BUFFER_SIZE);
    }

    @Override
    public void close() {
        // we need to guarantee only one onDisconnect invocation for retry logic to be healthy
        synchronized (this) {
            if (!isClosed()) {
                super.close();
                disconnectListener.onDisconnect("on close");
            }
        }
    }

    @Override
    public String getLoggingPrefix() {
        return loggingPrefix;
    }

    @Override
    public IncomingDataBuffer getDataBuffer() {
        return dataBuffer;
    }

    @Override
    public void write(byte[] bytes) throws IOException {
        super.write(bytes);

    	/*log.info("-------write "+bytes.length+" bytes:");
    	for (int i = 0; i < bytes.length; i++) {
        	log.info("["+i+"] " + (int)bytes[i]);
    	}*/

    	int offset = 0;
    	int numBytes = bytes.length;
    	// split the data if it doesn't fit in our out-buffer
    	while (numBytes > outBuf.remaining()) {
    		int remaining = outBuf.remaining();
    		//log.info("* remaining= "+remaining + " numBytes="+numBytes+" offset="+offset);
    		outBuf.put(bytes, offset, remaining);
    		numBytes -= remaining;
    		offset += remaining;
			flush();
    	}
    	//log.info("* rest numBytes="+numBytes+" offset="+offset);
        outBuf.put(bytes, offset, numBytes);
    }

    @Override
    public void flush() throws IOException {
        super.flush();
        con.sendBytesToSerial(Arrays.copyOf(outBuf.array(), outBuf.position()));
        outBuf.clear();
    }

    @Override
    public void setInputListener(final DataListener listener) {
    }

    public interface DisconnectListener {
        DisconnectListener VOID = (String message) -> {

        };
        void onDisconnect(String message);
    }
}
