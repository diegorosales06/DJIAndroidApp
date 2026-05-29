package com.dji.sdk.sample.demo.drop;

import android.content.Context;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;

import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.view.PresentableView;
import com.dji.sdk.sample.demo.geofencing.FlightLogger;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;

public class PayloadDropMissionView extends LinearLayout implements PresentableView {

    private static final String TAG = "DropMissionView";

    private FlightLogger flightLogger;
    private PayloadDropController payloadDropController = new PayloadDropController();

    private static final long CONTROL_LOOP_INTERVAL_MS = 50;
    private static final double DROP_RADIUS_M = 2.0;
    private static final float MIN_DROP_ALTITUDE_M = 6.1f;
    private static final float Kp_HORIZONTAL = 0.4f;
    private static final float Kp_VERTICAL = 0.6f;
    private static final float MAX_HORIZONTAL_SPEED = 5.0f;
    private static final float MIN_HORIZONTAL_SPEED = 0.3f;
    private static final float MAX_VERTICAL_SPEED = 2.0f;
    private static final double EARTH_RADIUS_M = 6_371_000.0;
    private static final double ACCEPTANCE_RADIUS_M = 2.0;
    private static final double ALTITUDE_ACCEPTANCE_M = 0.5;

    // Default drop target coordinates
    private static final double DEFAULT_DROP_LAT = 34.0272525;
    private static final double DEFAULT_DROP_LNG = -117.8511957;
    private static final float DEFAULT_DROP_ALT = 8.0f;

    private boolean missionRunning = false;
    private double dropTargetLat = DEFAULT_DROP_LAT;
    private double dropTargetLng = DEFAULT_DROP_LNG;
    private float dropTargetAlt = DEFAULT_DROP_ALT;
    private boolean dropTargetSet = true;

    private double currentLat = 0.0;
    private double currentLng = 0.0;
    private float currentAlt = 0.0f;
    private boolean hasValidGPS = false;
    private boolean hasDropped = false;

    private FlightController flightController;
    private Timer controlTimer;

    // UI
    private TextView tvStatus;
    private TextView tvDronePos;
    private EditText etDropLat;
    private EditText etDropLng;
    private EditText etDropAlt;
    private Button btnStart;
    private Button btnStop;
    private ScrollView scrollLog;
    private TextView tvLog;

    public PayloadDropMissionView(Context context) {
        super(context);
        init(context);
    }

    public PayloadDropMissionView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    @Override
    public int getDescription() { return 0; }

    @NonNull
    @Override
    public String getHint() { return this.getClass().getSimpleName() + ".java"; }

    private void init(Context context) {
        setOrientation(VERTICAL);
        setPadding(24, 24, 24, 24);

        tvStatus = new TextView(context);
        tvStatus.setText("Mission: IDLE");
        tvStatus.setTextSize(16f);
        tvStatus.setPadding(0, 0, 0, 6);
        addView(tvStatus);

        tvDronePos = new TextView(context);
        tvDronePos.setText("Drone: unknown");
        tvDronePos.setTextSize(13f);
        tvDronePos.setPadding(0, 0, 0, 14);
        addView(tvDronePos);

        TextView dropLabel = new TextView(context);
        dropLabel.setText("Drop Target Coordinates:");
        dropLabel.setTextSize(14f);
        dropLabel.setPadding(0, 0, 0, 4);
        addView(dropLabel);

        LinearLayout dropInputRow = new LinearLayout(context);
        dropInputRow.setOrientation(HORIZONTAL);
        dropInputRow.setPadding(0, 8, 0, 8);

        etDropLat = new EditText(context);
        etDropLat.setHint("Drop Lat");
        etDropLat.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                | android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);
        LinearLayout.LayoutParams p1 = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        p1.setMarginEnd(6);
        etDropLat.setLayoutParams(p1);
        dropInputRow.addView(etDropLat);

        etDropLng = new EditText(context);
        etDropLng.setHint("Drop Lng");
        etDropLng.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                | android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);
        LinearLayout.LayoutParams p2 = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        p2.setMarginEnd(6);
        etDropLng.setLayoutParams(p2);
        dropInputRow.addView(etDropLng);

        etDropAlt = new EditText(context);
        etDropAlt.setHint("Alt (m)");
        etDropAlt.setText("8");
        etDropAlt.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL);
        LinearLayout.LayoutParams p3 = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.6f);
        etDropAlt.setLayoutParams(p3);
        dropInputRow.addView(etDropAlt);

        // Auto-load default drop target into the UI
        etDropLat.setText(String.valueOf(DEFAULT_DROP_LAT));
        etDropLng.setText(String.valueOf(DEFAULT_DROP_LNG));
        etDropAlt.setText(String.valueOf(DEFAULT_DROP_ALT));

        addView(dropInputRow);

        LinearLayout btnRow = new LinearLayout(context);
        btnRow.setOrientation(HORIZONTAL);
        btnRow.setPadding(0, 8, 0, 16);

        btnStart = new Button(context);
        btnStart.setText("Start Mission");
        LinearLayout.LayoutParams bp1 = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bp1.setMarginEnd(8);
        btnStart.setLayoutParams(bp1);
        btnStart.setOnClickListener(v -> onStartMission());
        btnRow.addView(btnStart);

        btnStop = new Button(context);
        btnStop.setText("Stop Mission");
        LinearLayout.LayoutParams bp2 = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        btnStop.setLayoutParams(bp2);
        btnStop.setEnabled(false);
        btnStop.setOnClickListener(v -> onStopMission());
        btnRow.addView(btnStop);

        addView(btnRow);

        tvLog = new TextView(context);
        tvLog.setText("Log:\n");
        tvLog.setTextSize(11f);

        scrollLog = new ScrollView(context);
        scrollLog.setLayoutParams(new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 500));
        scrollLog.addView(tvLog);
        addView(scrollLog);

        initFlightController();
    }

    private void initFlightController() {
        if (DJISampleApplication.getProductInstance() == null) {
            appendLog("No aircraft connected.");
            return;
        }
        if (DJISampleApplication.getProductInstance() instanceof Aircraft) {
            Aircraft aircraft = (Aircraft) DJISampleApplication.getProductInstance();
            flightController = aircraft.getFlightController();
            if (flightController != null) {
                flightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                flightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
                flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
                flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);
                flightController.setStateCallback(this::onFlightControllerState);
                appendLog("FlightController ready.");
            } else {
                appendLog("FlightController unavailable.");
            }
        } else {
            appendLog("No aircraft connected.");
        }
    }

    private void onFlightControllerState(FlightControllerState state) {
        LocationCoordinate3D loc = state.getAircraftLocation();
        if (loc == null) return;
        double lat = loc.getLatitude();
        double lng = loc.getLongitude();
        float alt = loc.getAltitude();
        if (lat == 0.0 && lng == 0.0) { hasValidGPS = false; return; }
        currentLat = lat;
        currentLng = lng;
        currentAlt = alt;
        hasValidGPS = true;
        post(() -> tvDronePos.setText(String.format("Drone: (%.6f, %.6f)  alt: %.1fm", lat, lng, alt)));
    }

    private void onStartMission() {
        if (flightController == null) {
            initFlightController();
            if (flightController == null) { showToast("Flight controller not available."); return; }
        }
        if (!hasValidGPS) { showToast("Waiting for GPS fix."); return; }

        String dropLatStr = etDropLat.getText().toString().trim();
        String dropLngStr = etDropLng.getText().toString().trim();
        String dropAltStr = etDropAlt.getText().toString().trim();

        if (dropLatStr.isEmpty() || dropLngStr.isEmpty()) {
            showToast("Enter drop target latitude and longitude.");
            return;
        }

        try {
            dropTargetLat = Double.parseDouble(dropLatStr);
            dropTargetLng = Double.parseDouble(dropLngStr);
            dropTargetAlt = dropAltStr.isEmpty() ? 8.0f : Float.parseFloat(dropAltStr);
        } catch (NumberFormatException e) {
            showToast("Invalid coordinates.");
            return;
        }

        if (dropTargetLat < -90 || dropTargetLat > 90 || dropTargetLng < -180 || dropTargetLng > 180) {
            showToast("Coordinates out of range.");
            return;
        }

        if (dropTargetAlt < MIN_DROP_ALTITUDE_M) {
            showToast("Drop altitude must be at least 6.1 m / 20 ft.");
            return;
        }

        dropTargetSet = true;
        flightController.setVirtualStickModeEnabled(true, error -> {
            if (error != null) { appendLog("Failed to enable Virtual Stick: " + error.getDescription()); return; }
            appendLog("Virtual Stick mode enabled.");
            missionRunning = true;
            hasDropped = false;
            payloadDropController.reset();
            appendLog("Payload drop system armed.");
            appendLog(String.format("Flying to drop target: (%.6f, %.6f) @ %.1fm", dropTargetLat, dropTargetLng, dropTargetAlt));

            flightLogger = new FlightLogger(getContext());
            flightLogger.start();

            post(() -> {
                tvStatus.setText("Mission: RUNNING");
                btnStart.setEnabled(false);
                btnStop.setEnabled(true);
            });

            startControlLoop();
        });
    }

    private void onStopMission() {
        missionRunning = false;
        stopControlLoop();
        if (flightLogger != null) { flightLogger.stop(); }
        sendVelocityCommand(0, 0, 0);
        if (flightController != null) {
            flightController.setVirtualStickModeEnabled(false, error -> {
                if (error == null) appendLog("Virtual Stick disabled — manual control restored.");
            });
        }
        post(() -> {
            tvStatus.setText("Mission: STOPPED");
            btnStart.setEnabled(true);
            btnStop.setEnabled(false);
        });
        appendLog("Mission stopped.");
    }

    private void startControlLoop() {
        controlTimer = new Timer();
        controlTimer.scheduleAtFixedRate(new TimerTask() {
            @Override public void run() { runControlStep(); }
        }, 0, CONTROL_LOOP_INTERVAL_MS);
    }

    private void stopControlLoop() {
        if (controlTimer != null) { controlTimer.cancel(); controlTimer = null; }
    }

    private void runControlStep() {
        if (!missionRunning || !hasValidGPS) return;

        double distanceM = haversineDistance(currentLat, currentLng, dropTargetLat, dropTargetLng);
        float altError = dropTargetAlt - currentAlt;

        if (flightLogger != null) flightLogger.log(currentLat, currentLng, true);

        if (dropTargetSet && !payloadDropController.hasDropped()) {
            boolean closeToDropTarget = distanceM <= DROP_RADIUS_M;
            boolean highEnoughToDrop = currentAlt >= MIN_DROP_ALTITUDE_M;

            if (closeToDropTarget && highEnoughToDrop) {
                appendLog(String.format("Drop target reached! Distance: %.2fm", distanceM));
                sendVelocityCommand(0, 0, 0);
                payloadDropController.dropPayload(flightController);
                hasDropped = true;
                appendLog("Payload dropped! Initiating RTH.");
                missionRunning = false;
                stopControlLoop();
                if (flightLogger != null) flightLogger.stop();
                triggerRTH();
                return;
            }
        }

        float horizontalSpeed = (float)(Kp_HORIZONTAL * distanceM);
        horizontalSpeed = Math.max(MIN_HORIZONTAL_SPEED, Math.min(MAX_HORIZONTAL_SPEED, horizontalSpeed));

        double bearingRad = bearing(currentLat, currentLng, dropTargetLat, dropTargetLng);
        float northVelocity = (float)(horizontalSpeed * Math.cos(bearingRad));
        float eastVelocity  = (float)(horizontalSpeed * Math.sin(bearingRad));

        float verticalVelocity = Kp_VERTICAL * altError;
        verticalVelocity = Math.max(-MAX_VERTICAL_SPEED, Math.min(MAX_VERTICAL_SPEED, verticalVelocity));

        sendVelocityCommand(eastVelocity, northVelocity, verticalVelocity);
    }

    private void sendVelocityCommand(float pitch, float roll, float vertical) {
        if (flightController == null) return;
        FlightControlData data = new FlightControlData(pitch, roll, 0f, vertical);
        flightController.sendVirtualStickFlightControlData(data, error -> {
            if (error != null) Log.e(TAG, "Virtual stick error: " + error.getDescription());
        });
    }

    private void triggerRTH() {
        if (flightController == null) return;
        flightController.setVirtualStickModeEnabled(false, error -> {
            flightController.startGoHome(rthError -> {
                if (rthError == null) {
                    post(() -> { tvStatus.setText("Mission: RTH"); btnStart.setEnabled(true); btnStop.setEnabled(false); });
                    appendLog("RTH initiated.");
                } else {
                    appendLog("RTH failed: " + rthError.getDescription());
                }
            });
        });
    }

    static double haversineDistance(double lat1, double lng1, double lat2, double lng2) {
        double dLat = Math.toRadians(lat2 - lat1);
        double dLng = Math.toRadians(lng2 - lng1);
        double a = Math.sin(dLat/2)*Math.sin(dLat/2)
                + Math.cos(Math.toRadians(lat1))*Math.cos(Math.toRadians(lat2))
                * Math.sin(dLng/2)*Math.sin(dLng/2);
        return EARTH_RADIUS_M * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    }

    static double bearing(double lat1, double lng1, double lat2, double lng2) {
        double lat1R = Math.toRadians(lat1), lat2R = Math.toRadians(lat2);
        double dLng = Math.toRadians(lng2 - lng1);
        double y = Math.sin(dLng)*Math.cos(lat2R);
        double x = Math.cos(lat1R)*Math.sin(lat2R) - Math.sin(lat1R)*Math.cos(lat2R)*Math.cos(dLng);
        return (Math.atan2(y, x) + 2*Math.PI) % (2*Math.PI);
    }

    private void appendLog(String message) {
        post(() -> { tvLog.append(message + "\n"); scrollLog.post(() -> scrollLog.fullScroll(View.FOCUS_DOWN)); });
    }

    private void showToast(String message) {
        post(() -> Toast.makeText(getContext(), message, Toast.LENGTH_SHORT).show());
    }

    @Override
    protected void onDetachedFromWindow() {
        super.onDetachedFromWindow();
        if (missionRunning) { missionRunning = false; stopControlLoop(); sendVelocityCommand(0,0,0); if (flightLogger != null) flightLogger.stop(); }
        if (flightController != null) { flightController.setStateCallback(null); flightController.setVirtualStickModeEnabled(false, null); }
    }
}