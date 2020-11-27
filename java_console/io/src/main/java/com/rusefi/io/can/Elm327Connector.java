package com.rusefi.io.can;

import com.devexperts.logging.Logging;
import com.opensr5.io.DataListener;
import com.rusefi.io.IoStream;
import com.rusefi.io.serial.BaudRateHolder;
import com.rusefi.io.serial.SerialIoStreamJSerialComm;
import com.rusefi.io.tcp.BinaryProtocolProxy;
import com.rusefi.io.tcp.ServerSocketReference;
import com.rusefi.io.tcp.TcpConnector;

import java.io.Closeable;
import java.io.IOException;
import java.lang.Thread;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class Elm327Connector implements Closeable, DataListener {
	private final static Logging log = Logging.getLogging(Elm327Connector.class);
	private static final byte[] HEX_ARRAY = "0123456789ABCDEF".getBytes();

	private final static int ELM327_DEFAULT_BAUDRATE = 38400;
	private final static int BIG_TIMEOUT = 2000;
	private final static int TIMEOUT = 70;

	private final static int ISO_TP_FRAME_SINGLE = 0;
	private final static int ISO_TP_FRAME_FIRST = 1;
	private final static int ISO_TP_FRAME_CONSECUTIVE = 2;
	private final static int ISO_TP_FRAME_FLOW_CONTROL = 3;
    
    private IoStream stream = null;
	private String partialLine = new String();
	private List<String> completeLines = new ArrayList<String>();
	private boolean waitForEcho = true;

	private ServerSocketReference serverHolder;
	private Elm327IoStream elmStream;

	public static boolean checkConnection(String serialPort) {
		Elm327Connector con = new Elm327Connector();
		boolean found = con.initConnection(serialPort);
		con.close();
		return found;
	}

    public void start(String serialPort) {
    	log.info("* Elm327.start()");

        if (initConnection(serialPort)) {
        	// reset to defaults
        	sendCommand("ATD", "OK");

        	// Echo off
        	sendCommand("ATE0", "OK");
        	waitForEcho = false;
        	
        	// protocol #6 - ISO 15765-4 CAN (11 bit ID, 500 kbaud)
			sendCommand("ATSP6", "OK");
			
			// rx ID = 0x102
			sendCommand("ATCF 102", "OK");

			// rx ID mask = "all bits set"
			sendCommand("ATCM FFF", "OK");

			// tx ID = 0x100
			sendCommand("ATSH 100", "OK");

			// disable data auto-formatting
			sendCommand("ATCAF0", "OK");
			
			// read the ignition voltage
			String voltage = sendCommand("ATRV", "([0-9\\.]+)V");
			log.info("* Ignition voltage = " + voltage);
        }
        
        //startNetworkConnector(TcpConnector.DEFAULT_PORT);
		sendBytesToSerial(new byte[] { 0, 1, 83, 32, 96, (byte)239, (byte)195 });
    }

    @Override
    public void close() {
    	log.info("* Elm327.close()");
    	if (stream != null)
    		stream.close();
    }

    @Override
    public void onDataArrived(byte freshData[]) {
    	// ELM327 uses a text protocol, so we convert the data to a string
    	String freshStr = new String(freshData);
    	while (true) {
	    	int newL = freshStr.indexOf('\r');
    	    //log.info("* onData: " + newL + " [" + freshStr + "]");
        
	        // split the stream into separate lines
    	    if (newL >= 0) {
        		String curLine = this.partialLine;
	        	this.partialLine = new String();
    	    	if (newL > 0)
        		    curLine += freshStr.substring(0, newL);
    	        if (curLine.length() > 0)
        	    	processLine(curLine);
            	freshStr = freshStr.substring(newL + 1);
            	continue;
	        }
			this.partialLine += freshStr;
			break;
		}
    }

	public void sendBytesToSerial(byte [] bytes) {
    	log.info("-------sendBytesToSerial "+bytes.length+" bytes:");
    	for (int i = 0; i < bytes.length; i++) {
        	log.info("["+i+"] " + ((int)bytes[i] & 0xff));
    	}

    	// 1 frame
    	if (bytes.length <= 7) {
    		sendFrame((ISO_TP_FRAME_SINGLE << 4) | bytes.length, bytes, 0, bytes.length);
    		return;
    	}

    	// multiple frames
    	// send the first header frame
    	sendFrame((ISO_TP_FRAME_FIRST << 4) | ((bytes.length >> 8) & 0x0f), bytes.length & 0xff, bytes, 0, 6);
    	// get a flow control frame
    	byte[] fc = receiveData();

    	// send the rest of the data
    	int idx = 1, offset = 6;
    	int remaining = bytes.length - 6;
    	while (remaining > 0) {
    		int len = Math.min(remaining, 7);
    		// send the consecutive frames
    		sendFrame((ISO_TP_FRAME_CONSECUTIVE << 4) | ((idx++) & 0x0f), bytes, offset, len);
    		offset += len;
    		remaining -= len;
    	}

	}

	///////////////////////////////////////////////////////

    private boolean initConnection(String serialPort) {
        // todo: this seems like a hack-ish way? Shouldn't be openPort(port, baudrate)?
        BaudRateHolder.INSTANCE.baudRate = ELM327_DEFAULT_BAUDRATE;
    	
    	this.stream = SerialIoStreamJSerialComm.openPort(serialPort);
        
        this.stream.setInputListener(this);
        if (sendCommand("ATZ", "ELM327 v[0-9]+\\.[0-9]+", BIG_TIMEOUT) != null) {
        	log.info("ELM DETECTED on " + serialPort + "!");
        	return true;
        }
		log.info("ELM NOT FOUND on " + serialPort + "!");
		return false;
    }

    private String sendCommand(String command, String responseFilter) {
    	return sendCommand(command, responseFilter, TIMEOUT);
    }

    private String sendCommand(String command, String responseFilter, int timeout) {
    	log.info("* Elm327.cmd: " + command);
    	this.completeLines.clear();
       	try {
       		this.stream.write((command + "\r").getBytes());
       		waitForResponse(timeout);
	    } catch (IOException ignore) {
	        return null;
	    } catch (InterruptedException ignore) {
	        return null;
	    }

	    if (this.completeLines.size() < 1)
	    	return null;

        int responseIdx = 0;
		// the first line can repeat the command (if echo is on)
        if (this.completeLines.get(0).equals(command)) {
        	// then the response is in the 2nd line
        	responseIdx = 1;
        }
        // return the response only if it matches the given regexp
        Pattern pattern = Pattern.compile(responseFilter);
		Matcher matcher = pattern.matcher(this.completeLines.get(responseIdx));
		if (matcher.find()) {
        	// store the echo mode
        	this.waitForEcho = responseIdx != 0;
			
        	return (matcher.groupCount() > 0) ? matcher.group(1) : matcher.group();
        }
        return null;
    }

    private void sendFrame(int hdr0, byte [] data, int offset, int len) {
    	sendData(new byte[] { (byte)hdr0 }, data, offset, len);
    }
    
    private void sendFrame(int hdr0, int hdr1, byte [] data, int offset, int len) {
    	sendData(new byte[] { (byte)hdr0, (byte)hdr1 }, data, offset, len);
    }

    private void sendData(byte [] hdr, byte [] data, int offset, int len) {
    	len += hdr.length;
    	byte [] hexData = new byte [len * 2 + 1];
   		for (int i = 0, j = 1; i < len; i++, j += 2) {
      		int v = ((i < hdr.length) ? hdr[i] : data[i - hdr.length + offset]) & 0xFF;
        	hexData[j] = HEX_ARRAY[v >>> 4];
        	hexData[j + 1] = HEX_ARRAY[v & 0x0F];
      	}
      	hexData[len * 2] = '\r';

   		//!!!!!!!
   		log.info("* Elm327.data: " + (new String(hexData)));

		try {
       		this.stream.write(hexData);
	    } catch (IOException ignore) {
	        return;
	    }
    }

    private synchronized byte[] receiveData() {
		try {
       		waitForResponse(TIMEOUT);
       		// decode text lines
       		// todo:
       		//this.completeLines.size()
       		return null;
	    } catch (InterruptedException ignore) {
	        return null;
	    }
    }

    private synchronized void waitForResponse(int timeout) throws InterruptedException {
    	// multiple lines can be sent, we need to wait for them all
    	while (true) {
       		int numLines = this.completeLines.size();
	        wait(timeout);
	        // if nothing changed
	        if (this.completeLines.size() == numLines)
	        	break;
	    }
    }

    private synchronized void processLine(String line) {
    	log.info("Elm327Connector.processLine(): {" + line + "}");
    	
    	// remove the 'cursor'
    	if (line.charAt(0) == '>')
    		line = line.substring(1);
    	
    	this.completeLines.add(line);
    	notifyAll();
    }

    private boolean startNetworkConnector(int controllerPort) {
        try {
	        elmStream = new Elm327IoStream(this, "elm327Stream");
	        serverHolder = BinaryProtocolProxy.createProxy(elmStream, controllerPort, new AtomicInteger());
	    } catch (IOException ignore) {
	        return false;
	    }

        return true;
	}

}
