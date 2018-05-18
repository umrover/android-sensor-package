package org.mrover.sensorpackage;

import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;
import android.os.ParcelFileDescriptor;
import android.system.ErrnoException;
import android.system.OsConstants;
import android.util.Log;

import java.io.Closeable;
import java.io.FileDescriptor;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;

public class AccessoryProtocol {
    private static final String TAG = "AccessoryProtocol";
    private static final int MESSAGE_SIZE = 32;

    private final UsbManager service_;

    private UsbAccessory accessory_;
    private ParcelFileDescriptor fd_;
    private FileOutputStream tx_;
    private boolean is_connected_;

    private final ByteBuffer buf_;
    private final byte[] size_bytes_ = new byte[2];

    public AccessoryProtocol(final UsbManager service) {
        this.service_ = service;

        this.buf_ = ByteBuffer.allocateDirect(MESSAGE_SIZE); // TODO check the size
        this.size_bytes_[0] = (byte) ((MESSAGE_SIZE & 0xff00) >> 8);
        this.size_bytes_[1] = (byte) (MESSAGE_SIZE & 0x00ff);
    }

    public boolean connect(final UsbAccessory accessory) {
        this.accessory_ = accessory;

        this.fd_ = this.service_.openAccessory(this.accessory_);
        if (this.fd_ == null) {
            return false;
        }

        FileDescriptor fd = this.fd_.getFileDescriptor();
        this.tx_ = new FileOutputStream(fd);
        this.is_connected_ = true;

        Log.d(TAG, "connected to accessory");

        return true;
    }

    public boolean connected() {
        return this.is_connected_;
    }

    public boolean transmit(int lat_deg, float lat_min,
                            int lon_deg, float lon_min,
                            float bearing_deg, int num_sats,
                            boolean valid) throws IOException {
        // Encode
        this.buf_.rewind();
        this.buf_.putInt(lat_deg);
        this.buf_.putFloat(lat_min);
        this.buf_.putInt(lon_deg);
        this.buf_.putFloat(lon_min);
        this.buf_.putFloat(bearing_deg);
        this.buf_.putInt(num_sats);
        this.buf_.putInt(valid ? 1 : 0);

        // Transmit
        try {
            this.tx_.write(this.size_bytes_);
            this.tx_.write(this.buf_.array(), 0, MESSAGE_SIZE);
            this.tx_.flush();
            return true;
        } catch (IOException e) {
            if (is_no_such_device(e)) {
                throw e;
            }

            Log.d(TAG, "IOException in writing", e);
            return false;
        }
    }

    public void disconnect() {
        if (this.is_connected_) {
            this.is_connected_ = false;
        }

        if (this.tx_ != null) {
            closeQuietly(this.tx_);
            this.tx_ = null;
        }

        if (this.fd_ != null) {
            closeQuietly(this.fd_);
            this.fd_ = null;
        }
    }

    private void closeQuietly(Closeable c) {
        try {
            c.close();
        } catch (IOException e) {
            // quiet it
        }
    }

    private boolean is_no_such_device(IOException e) {
        final Throwable cause = e.getCause();
        if (cause instanceof ErrnoException) {
            return ((ErrnoException) cause).errno == OsConstants.ENODEV;
        }
        return false;
    }
}
