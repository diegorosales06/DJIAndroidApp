package com.dji.sdk.sample.demo.drop;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.util.Log;

import java.io.IOException;
import java.io.OutputStream;
import java.util.Set;
import java.util.UUID;

public class PayloadDropController {

    private static final String TAG = "PayloadDropController";

    // This name must match the ESP32 Bluetooth name
    private static final String ESP32_DEVICE_NAME = "ESP32_DROP";

    // Standard Bluetooth Serial UUID
    private static final UUID SPP_UUID =
            UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    private boolean armed = false;
    private boolean alreadyDropped = false;

    private BluetoothAdapter bluetoothAdapter;
    private BluetoothSocket bluetoothSocket;
    private OutputStream outputStream;

    public PayloadDropController() {
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
    }

    public void armDrop() {
        armed = true;
    }

    public void dropPayload() {
        if (!armed) {
            Log.d(TAG, "Drop ignored: system is not armed.");
            return;
        }

        if (alreadyDropped) {
            Log.d(TAG, "Drop ignored: payload already dropped.");
            return;
        }

        boolean sent = sendDropCommand();

        if (sent) {
            alreadyDropped = true;
            Log.d(TAG, "Drop command sent to ESP32.");
        } else {
            Log.e(TAG, "Drop command failed.");
        }
    }

    private boolean sendDropCommand() {
        try {
            if (!connectToESP32()) {
                return false;
            }

            outputStream.write("DROP\n".getBytes());
            outputStream.flush();
            return true;

        } catch (IOException e) {
            Log.e(TAG, "Error sending DROP command: " + e.getMessage());
            closeConnection();
            return false;
        }
    }

    private boolean connectToESP32() {
        if (bluetoothAdapter == null) {
            Log.e(TAG, "Bluetooth not supported on this device.");
            return false;
        }

        if (!bluetoothAdapter.isEnabled()) {
            Log.e(TAG, "Bluetooth is not enabled.");
            return false;
        }

        if (bluetoothSocket != null && bluetoothSocket.isConnected() && outputStream != null) {
            return true;
        }

        BluetoothDevice esp32Device = findPairedESP32();

        if (esp32Device == null) {
            Log.e(TAG, "ESP32 device not paired. Pair with ESP32_DROP first.");
            return false;
        }

        try {
            bluetoothSocket = esp32Device.createRfcommSocketToServiceRecord(SPP_UUID);
            bluetoothSocket.connect();
            outputStream = bluetoothSocket.getOutputStream();

            Log.d(TAG, "Connected to ESP32.");
            return true;

        } catch (IOException e) {
            Log.e(TAG, "Failed to connect to ESP32: " + e.getMessage());
            closeConnection();
            return false;
        }
    }

    private BluetoothDevice findPairedESP32() {
        Set<BluetoothDevice> pairedDevices = bluetoothAdapter.getBondedDevices();

        if (pairedDevices == null) {
            return null;
        }

        for (BluetoothDevice device : pairedDevices) {
            if (ESP32_DEVICE_NAME.equals(device.getName())) {
                return device;
            }
        }

        return null;
    }

    public void resetDropSystem() {
        armed = false;
        alreadyDropped = false;
    }

    public boolean isArmed() {
        return armed;
    }

    public boolean hasDropped() {
        return alreadyDropped;
    }

    public void closeConnection() {
        try {
            if (outputStream != null) {
                outputStream.close();
            }
        } catch (IOException ignored) {}

        try {
            if (bluetoothSocket != null) {
                bluetoothSocket.close();
            }
        } catch (IOException ignored) {}

        outputStream = null;
        bluetoothSocket = null;
    }
}