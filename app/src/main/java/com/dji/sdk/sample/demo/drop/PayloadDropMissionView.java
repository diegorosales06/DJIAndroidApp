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

/**
* PayloadDropMissionView
 *
 * Autonomous GPS waypoint navigation for the DJI Mavic Air 2 (and other drones
 * that do NOT support WaypointMission) using Virtual Stick commands.
 *
 * How it works:
 *  1. User defines a list of GPS waypoints (lat, lng, altitude).
 *  2. On "Start Mission", the drone takes off and climbs to the first waypoint's altitude.
 *  3. A control loop runs at 20Hz via a Timer:
 *       a. Read current drone GPS position and altitude from FlightController state.
 *       b. Compute distance and bearing to the current target waypoint using
 *          the Haversine formula.
 *       c. Compute altitude error (target alt - current alt).
 *       d. Convert bearing into North/South (pitch) and East/West (roll) velocity
 *          commands using trigonometry.
 *       e. Scale speed proportionally to distance (P-controller) so the drone
 *          slows down as it approaches the waypoint.
 *       f. Send FlightControlData to the drone via Virtual Stick.
 *  4. When the drone is within ACCEPTANCE_RADIUS_M of the target waypoint AND
 *     within ALTITUDE_ACCEPTANCE_M of the target altitude, it advances to the
 *     next waypoint.
 *  5. After the last waypoint, the drone initiates RTH.
 *
 * Control law (proportional controller):
 *   horizontalSpeed = Kp_horizontal * distanceToWaypoint
 *   verticalSpeed   = Kp_vertical   * altitudeError
 *   Both are clamped to safe max values.
 *
 * Coordinate system used: GROUND (NED — North/East/Down body frame).
 *   pitch > 0 = fly North
 *   roll  > 0 = fly East
 *   verticalThrottle > 0 = climb
 *   yaw = 0 (heading held constant; drone nose stays fixed)
 *
 * To register in the sample app menu:
 * Add an entry in DemoListView.java pointing to PayloadDropMissionView.class
 *
* Package: com.dji.sdk.sample.demo.drop
 */
public class PayloadDropMissionView extends LinearLayout implements PresentableView {

    private static final String TAG = "VSWaypointView";

    // FlightLogger — writes lat/lng/waypoint data to CSV during mission
    private FlightLogger flightLogger;
    private PayloadDropController payloadDropController = new PayloadDropController();

    // ---------------------------------------------------------------------------------
    // Control loop timing
    // ---------------------------------------------------------------------------------

    // How often the control loop runs (ms). 20Hz = 50ms interval.
    // DJI requires Virtual Stick commands at minimum every 200ms or the drone
    // will exit virtual stick mode — 50ms gives us plenty of margin.
    private static final long CONTROL_LOOP_INTERVAL_MS = 50;

    // ---------------------------------------------------------------------------------
    // Acceptance criteria
    // ---------------------------------------------------------------------------------

    // Horizontal distance (meters) within which a waypoint is considered "reached"
    private static final double ACCEPTANCE_RADIUS_M = 2.0;
    private static final double DROP_RADIUS_M = 2.0;
    private static final float MIN_DROP_ALTITUDE_M = 6.1f; // 20ft AGL

    // Altitude error (meters) within which altitude is considered "reached"
    private static final double ALTITUDE_ACCEPTANCE_M = 0.5;

    // ---------------------------------------------------------------------------------
    // Proportional controller gains (Kp)
    // These scale how aggressively the drone responds to position error.
    // Increase to fly faster, decrease if the drone overshoots.
    // ---------------------------------------------------------------------------------

    // Horizontal: maps distance (meters) → speed (m/s)
    // Example: 10m away → 10 * 0.4 = 4 m/s
    private static final float Kp_HORIZONTAL = 0.4f;

    // Vertical: maps altitude error (meters) → climb/descend speed (m/s)
    private static final float Kp_VERTICAL = 0.6f;

    // ---------------------------------------------------------------------------------
    // Speed limits — clamp output of proportional controller to safe values
    // ---------------------------------------------------------------------------------

    // Max horizontal cruise speed in m/s (Mavic Air 2 max is ~19 m/s, keep conservative)
    private static final float MAX_HORIZONTAL_SPEED = 5.0f;

    // Min speed so the drone doesn't stall when very close to waypoint
    private static final float MIN_HORIZONTAL_SPEED = 0.3f;

    // Max climb/descend speed in m/s
    private static final float MAX_VERTICAL_SPEED = 2.0f;

    // ---------------------------------------------------------------------------------
    // Earth radius constant for Haversine distance calculation
    // ---------------------------------------------------------------------------------
    private static final double EARTH_RADIUS_M = 6_371_000.0;

    // ---------------------------------------------------------------------------------
    // Waypoint storage
    // Each entry is a double[3]: { latitude, longitude, altitudeMeters }
    // ---------------------------------------------------------------------------------
    private final List<double[]> waypointList = new ArrayList<>();

    // Index of the waypoint the drone is currently flying toward
    private int currentWaypointIndex = 0;

    // ---------------------------------------------------------------------------------
    // Mission state
    // ---------------------------------------------------------------------------------
    private boolean missionRunning = false;
    private double dropTargetLat = 0.0;
    private double dropTargetLng = 0.0;
    private float dropTargetAlt = 8.0f;
    private boolean dropTargetSet = false;

    // Latest drone position — updated by the FlightController state callback
    private double currentLat = 0.0;
    private double currentLng = 0.0;
    private float  currentAlt = 0.0f;  // altitude above takeoff point (meters)
    private boolean hasValidGPS = false;

    // ---------------------------------------------------------------------------------
    // DJI
    // ---------------------------------------------------------------------------------
    private FlightController flightController;

    // Timer that drives the 20Hz control loop
    private Timer controlTimer;

    // ---------------------------------------------------------------------------------
    // UI components
    // ---------------------------------------------------------------------------------
    private TextView   tvStatus;
    private TextView   tvCurrentWaypoint;
    private TextView   tvDronePos;
    private TextView   tvWaypointList;
    private EditText   etLat;
    private EditText   etLng;
    private EditText   etAlt;
    private EditText   etDropLat;
    private EditText   etDropLng;
    private EditText   etDropAlt;
    private Button     btnAddWaypoint;
    private Button     btnClearWaypoints;
    private Button     btnStart;
    private Button     btnStop;
    private ScrollView scrollLog;
    private TextView   tvLog;

    // ---------------------------------------------------------------------------------
    // Constructors
    // ---------------------------------------------------------------------------------

   public PayloadDropMissionView(Context context) {
    super(context);
    init(context);
}

public PayloadDropMissionView(Context context, AttributeSet attrs) {
    super(context, attrs);
    init(context);
}

    // ---------------------------------------------------------------------------------
    // PresentableView
    // ---------------------------------------------------------------------------------

    @Override
    public int getDescription() {
        return 0;
    }

    @NonNull
    @Override
    public String getHint() {
        return this.getClass().getSimpleName() + ".java";
    }

    // ---------------------------------------------------------------------------------
    // UI Construction
    // ---------------------------------------------------------------------------------

    private void init(Context context) {

        setOrientation(VERTICAL);
        setPadding(24, 24, 24, 24);

        // Status label
        tvStatus = new TextView(context);
        tvStatus.setText("Mission: IDLE");
        tvStatus.setTextSize(16f);
        tvStatus.setPadding(0, 0, 0, 6);
        addView(tvStatus);

        // Current waypoint target label
        tvCurrentWaypoint = new TextView(context);
        tvCurrentWaypoint.setText("Target: —");
        tvCurrentWaypoint.setTextSize(13f);
        tvCurrentWaypoint.setPadding(0, 0, 0, 6);
        addView(tvCurrentWaypoint);

        // Drone position label
        tvDronePos = new TextView(context);
        tvDronePos.setText("Drone: unknown");
        tvDronePos.setTextSize(13f);
        tvDronePos.setPadding(0, 0, 0, 14);
        addView(tvDronePos);

        // Lat / Lng / Alt input row
        LinearLayout inputRow = new LinearLayout(context);
        inputRow.setOrientation(HORIZONTAL);

        etLat = new EditText(context);
        etLat.setHint("Latitude");
        etLat.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                | android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);
        LinearLayout.LayoutParams etP1 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        etP1.setMarginEnd(6);
        etLat.setLayoutParams(etP1);
        inputRow.addView(etLat);

        etLng = new EditText(context);
        etLng.setHint("Longitude");
        etLng.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                | android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);
        LinearLayout.LayoutParams etP2 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        etP2.setMarginEnd(6);
        etLng.setLayoutParams(etP2);
        inputRow.addView(etLng);

        etAlt = new EditText(context);
        etAlt.setHint("Alt (m)");
        etAlt.setText("10");
        etAlt.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL);
        LinearLayout.LayoutParams etP3 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.6f);
        etAlt.setLayoutParams(etP3);
        inputRow.addView(etAlt);

        addView(inputRow);
        LinearLayout dropInputRow = new LinearLayout(context);
        dropInputRow.setOrientation(HORIZONTAL);
        dropInputRow.setPadding(0, 8, 0, 8);

        etDropLat = new EditText(context);
        etDropLat.setHint("Drop Lat");
        etDropLat.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                | android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);
        LinearLayout.LayoutParams dropLatParams = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        dropLatParams.setMarginEnd(6);
        etDropLat.setLayoutParams(dropLatParams);
        dropInputRow.addView(etDropLat);

        etDropLng = new EditText(context);
        etDropLng.setHint("Drop Lng");
        etDropLng.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                | android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);
        LinearLayout.LayoutParams dropLngParams = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        dropLngParams.setMarginEnd(6);
        etDropLng.setLayoutParams(dropLngParams);
        dropInputRow.addView(etDropLng);

        etDropAlt = new EditText(context);
        etDropAlt.setHint("Drop Alt");
        etDropAlt.setText("8");
        etDropAlt.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL);
        LinearLayout.LayoutParams dropAltParams = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.6f);
        etDropAlt.setLayoutParams(dropAltParams);
        dropInputRow.addView(etDropAlt);

        addView(dropInputRow);

        // Add / Clear buttons
        LinearLayout btnRow1 = new LinearLayout(context);
        btnRow1.setOrientation(HORIZONTAL);
        btnRow1.setPadding(0, 8, 0, 8);

        btnAddWaypoint = new Button(context);
        btnAddWaypoint.setText("Add Waypoint");
        LinearLayout.LayoutParams bP1 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bP1.setMarginEnd(8);
        btnAddWaypoint.setLayoutParams(bP1);
        btnAddWaypoint.setOnClickListener(v -> onAddWaypoint());
        btnRow1.addView(btnAddWaypoint);

        btnClearWaypoints = new Button(context);
        btnClearWaypoints.setText("Clear All");
        LinearLayout.LayoutParams bP2 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        btnClearWaypoints.setLayoutParams(bP2);
        btnClearWaypoints.setOnClickListener(v -> onClearWaypoints());
        btnRow1.addView(btnClearWaypoints);

        addView(btnRow1);

        // Waypoint list display
        tvWaypointList = new TextView(context);
        tvWaypointList.setText("Waypoints: (none)");
        tvWaypointList.setTextSize(12f);
        tvWaypointList.setPadding(0, 0, 0, 12);
        addView(tvWaypointList);

        // add default waypoints only after tvWaypointList exists
        waypointList.add(new double[]{34.04625892060497, -117.84532693990677, 8});
        waypointList.add(new double[]{34.04642926281088, -117.8454831769448,  8});
        waypointList.add(new double[]{34.04628913461136, -117.84535578331251, 8});
        refreshWaypointList();

        // Start / Stop buttons
        LinearLayout btnRow2 = new LinearLayout(context);
        btnRow2.setOrientation(HORIZONTAL);
        btnRow2.setPadding(0, 0, 0, 16);

        btnStart = new Button(context);
        btnStart.setText("Start Mission");
        LinearLayout.LayoutParams bP3 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bP3.setMarginEnd(8);
        btnStart.setLayoutParams(bP3);
        btnStart.setOnClickListener(v -> onStartMission());
        btnRow2.addView(btnStart);

        btnStop = new Button(context);
        btnStop.setText("Stop Mission");
        LinearLayout.LayoutParams bP4 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        btnStop.setLayoutParams(bP4);
        btnStop.setEnabled(false);
        btnStop.setOnClickListener(v -> onStopMission());
        btnRow2.addView(btnStop);

        addView(btnRow2);

        // Scrollable log
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

    // ---------------------------------------------------------------------------------
    // DJI initialisation
    // ---------------------------------------------------------------------------------

    private void initFlightController() {
        if (DJISampleApplication.getProductInstance() == null) {
            appendLog("No aircraft connected.");
            return;  // exit safely instead of crashing
        }
        if (DJISampleApplication.getProductInstance() instanceof Aircraft) {
            Aircraft aircraft = (Aircraft) DJISampleApplication.getProductInstance();
            flightController = aircraft.getFlightController();
            if (flightController != null) {
                // Configure Virtual Stick control modes before enabling.
                // VELOCITY mode means our pitch/roll values are in m/s, not degrees.
                // GROUND coordinate system means pitch/roll are relative to true
                // North/East, not the drone's nose heading — much easier to work with.
                flightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                flightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
                flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
                flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);

                // Attach the state callback to continuously read GPS position
                flightController.setStateCallback(this::onFlightControllerState);

                appendLog("FlightController ready.");
            } else {
                appendLog("FlightController unavailable.");
            }
        } else {
            appendLog("No aircraft connected.");
        }
    }

    // ---------------------------------------------------------------------------------
    // Flight Controller State Callback
    // Fires ~10Hz — keeps currentLat/Lng/Alt up to date for the control loop
    // ---------------------------------------------------------------------------------

    private void onFlightControllerState(FlightControllerState state) {
        LocationCoordinate3D loc = state.getAircraftLocation();
        if (loc == null) return;

        double lat = loc.getLatitude();
        double lng = loc.getLongitude();
        float  alt = loc.getAltitude();

        // 0,0 means no GPS fix — ignore
        if (lat == 0.0 && lng == 0.0) {
            hasValidGPS = false;
            return;
        }

        currentLat = lat;
        currentLng = lng;
        currentAlt = alt;
        hasValidGPS = true;

        // Update position display on UI thread
        post(() -> tvDronePos.setText(String.format(
                "Drone: (%.6f, %.6f)  alt: %.1fm", lat, lng, alt)));
    }

    // ---------------------------------------------------------------------------------
    // Button handlers
    // ---------------------------------------------------------------------------------

    private void onAddWaypoint() {
        String latStr = etLat.getText().toString().trim();
        String lngStr = etLng.getText().toString().trim();
        String altStr = etAlt.getText().toString().trim();

        if (latStr.isEmpty() || lngStr.isEmpty()) {
            showToast("Enter latitude and longitude.");
            return;
        }

        double lat, lng;
        float  alt;
        try {
            lat = Double.parseDouble(latStr);
            lng = Double.parseDouble(lngStr);
            alt = altStr.isEmpty() ? 10.0f : Float.parseFloat(altStr);
        } catch (NumberFormatException e) {
            showToast("Invalid coordinates.");
            return;
        }

        if (lat < -90 || lat > 90 || lng < -180 || lng > 180) {
            showToast("Coordinates out of valid GPS range.");
            return;
        }

        waypointList.add(new double[]{lat, lng, alt});
        etLat.setText("");
        etLng.setText("");
        refreshWaypointList();
        appendLog(String.format("Added waypoint %d: (%.6f, %.6f) @ %.1fm",
                waypointList.size(), lat, lng, alt));
    }

    private void onClearWaypoints() {
        if (missionRunning) {
            showToast("Stop the mission before clearing waypoints.");
            return;
        }
        waypointList.clear();
        refreshWaypointList();
        appendLog("Waypoints cleared.");
    }

    /**
     * Starts the waypoint mission.
     *
     * Steps:
     *  1. Validate we have waypoints and a flight controller.
     *  2. Enable Virtual Stick mode on the flight controller.
     *  3. Reset waypoint index to 0.
     *  4. Start the 20Hz control loop timer.
     */
    private void onStartMission() {
        if (waypointList.size() < 1) {
            showToast("Add at least 1 waypoint.");
            return;
        }
        if (flightController == null) {
            initFlightController();
            if (flightController == null) {
                showToast("Flight controller not available.");
                return;
            }
        }
        if (!hasValidGPS) {
            showToast("Waiting for GPS fix.");
            return;
        }
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
            showToast("Invalid drop target coordinates.");
            return;
        }

        if (dropTargetLat < -90 || dropTargetLat > 90 || dropTargetLng < -180 || dropTargetLng > 180) {
            showToast("Drop target coordinates out of range.");
            return;
        }

        if (dropTargetAlt < MIN_DROP_ALTITUDE_M) {
            showToast("Drop altitude must be at least 6.1 m / 20 ft.");
            return;
        }

        dropTargetSet = true;
        // Enable Virtual Stick mode — required before sending any FlightControlData.
        // The drone will reject commands if this isn't enabled first.
        flightController.setVirtualStickModeEnabled(true, error -> {
            if (error != null) {
                appendLog("Failed to enable Virtual Stick: " + error.getDescription());
                return;
            }
            appendLog("Virtual Stick mode enabled.");
            currentWaypointIndex = 0;
            missionRunning = true;

            payloadDropController.resetDropSystem();
            payloadDropController.armDrop();
            appendLog("Payload drop system armed.");

            appendLog(String.format("Drop target set: (%.6f, %.6f) @ %.1fm",
                    dropTargetLat, dropTargetLng, dropTargetAlt));

            // Start flight logger — creates a new timestamped CSV file
            flightLogger = new FlightLogger(getContext());
            flightLogger.start();
            appendLog("Logging to: " + flightLogger.getLogFilePath());

            post(() -> {
                tvStatus.setText("Mission: RUNNING");
                btnStart.setEnabled(false);
                btnStop.setEnabled(true);
                btnAddWaypoint.setEnabled(false);
                btnClearWaypoints.setEnabled(false);
                updateTargetLabel();
            });

            appendLog("Mission started. Flying to waypoint 1.");
            startControlLoop();
        });
    }

    /**
     * Stops the mission immediately.
     * Sends a zero-velocity command to halt the drone in place, then
     * disables Virtual Stick mode so the pilot can take back manual control.
     */
    private void onStopMission() {
        missionRunning = false;
        stopControlLoop();

        // Stop the logger and flush to disk
        if (flightLogger != null) {
            flightLogger.stop();
            appendLog("Log saved to: " + flightLogger.getLogFilePath());
        }

        // Send one final zero command to stop all movement
        sendVelocityCommand(0, 0, 0);

        // Disable Virtual Stick — returns control to the remote controller
        if (flightController != null) {
            flightController.setVirtualStickModeEnabled(false, error -> {
                if (error == null) {
                    appendLog("Virtual Stick disabled — manual control restored.");
                } else {
                    appendLog("Error disabling Virtual Stick: " + error.getDescription());
                }
            });
        }

        post(() -> {
            tvStatus.setText("Mission: STOPPED");
            btnStart.setEnabled(true);
            btnStop.setEnabled(false);
            btnAddWaypoint.setEnabled(true);
            btnClearWaypoints.setEnabled(true);
            tvCurrentWaypoint.setText("Target: —");
        });

        appendLog("Mission stopped by user.");
    }

    // ---------------------------------------------------------------------------------
    // Control Loop — runs at 20Hz via a Timer
    // ---------------------------------------------------------------------------------

    private void startControlLoop() {
        controlTimer = new Timer();
        controlTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                // Each tick: read position, compute commands, send to drone
                runControlStep();
            }
        }, 0, CONTROL_LOOP_INTERVAL_MS);
    }

    private void stopControlLoop() {
        if (controlTimer != null) {
            controlTimer.cancel();
            controlTimer = null;
        }
    }

    /**
     * One iteration of the control loop.
     *
     * This is the core of the waypoint navigation logic:
     *  1. Get the current target waypoint coordinates.
     *  2. Compute horizontal distance and bearing to it using Haversine.
     *  3. Compute altitude error.
     *  4. If within acceptance radius AND altitude is correct → advance to next waypoint.
     *  5. Otherwise → compute velocity commands and send them.
     */
    private void runControlStep() {
        if (!missionRunning || !hasValidGPS) return;
        if (currentWaypointIndex >= waypointList.size()) return;

        // Pull the current target waypoint
        double[] target    = waypointList.get(currentWaypointIndex);
        double targetLat   = target[0];
        double targetLng   = target[1];
        float  targetAlt   = (float) target[2];

        // --- Compute horizontal distance to target (Haversine formula) ---
        // Haversine gives great-circle distance between two GPS points in meters.
        // This is more accurate than simple Euclidean distance for GPS coords.
        double distanceM = haversineDistance(currentLat, currentLng, targetLat, targetLng);

        // --- Compute altitude error ---
        float altError = targetAlt - currentAlt;

        // Log every tick to Logcat for debugging
        Log.d(TAG, String.format("WP%d dist=%.2fm altErr=%.2fm lat=%.6f lng=%.6f",
                currentWaypointIndex + 1, distanceM, altError, currentLat, currentLng));

        // Write to CSV file
        if (flightLogger != null) {
            flightLogger.log(currentLat, currentLng, true);
        }

        // --- Check if waypoint is reached ---
        // Both horizontal AND vertical acceptance must be met before advancing
        boolean horizontalReached = distanceM <= ACCEPTANCE_RADIUS_M;
        boolean verticalReached   = Math.abs(altError) <= ALTITUDE_ACCEPTANCE_M;

        if (dropTargetSet && !payloadDropController.hasDropped()) {
            double dropDistanceM = haversineDistance(currentLat, currentLng, dropTargetLat, dropTargetLng);
            boolean closeToDropTarget = dropDistanceM <= DROP_RADIUS_M;
            boolean highEnoughToDrop = currentAlt >= MIN_DROP_ALTITUDE_M;

            if (closeToDropTarget && highEnoughToDrop) {
                appendLog(String.format("Drop target reached! Distance: %.2fm", dropDistanceM));
                sendVelocityCommand(0, 0, 0);
                payloadDropController.dropPayload();
                appendLog("Payload drop command sent.");
            }
        }

        if (horizontalReached && verticalReached) {
            appendLog(String.format("Waypoint %d reached! (dist=%.2fm altErr=%.2fm)",
                    currentWaypointIndex + 1, distanceM, altError));


            currentWaypointIndex++;

            if (currentWaypointIndex >= waypointList.size()) {
                // All waypoints completed — initiate RTH
                appendLog("All waypoints complete. Initiating RTH.");
                missionRunning = false;
                stopControlLoop();
                sendVelocityCommand(0, 0, 0); // stop movement first

                // Stop the logger before handing off to RTH
                if (flightLogger != null) {
                    flightLogger.stop();
                    appendLog("Log saved to: " + flightLogger.getLogFilePath());
                }

                triggerRTH();
                return;
            }

            // Advance to next waypoint
            final int nextIdx = currentWaypointIndex;
            post(() -> {
                appendLog("Flying to waypoint " + (nextIdx + 1) + ".");
                updateTargetLabel();
            });
            return;
        }

        // --- Compute velocity commands ---

        // Proportional horizontal speed: faster when far, slower when close
// Proportional horizontal speed: faster when far, slower when close
        float horizontalSpeed = (float)(Kp_HORIZONTAL * distanceM);
        horizontalSpeed = Math.max(MIN_HORIZONTAL_SPEED,
                Math.min(MAX_HORIZONTAL_SPEED, horizontalSpeed));

// Bearing from current position to target (radians, clockwise from North)
        double bearingRad = bearing(currentLat, currentLng, targetLat, targetLng);

// Resolve desired ground velocity into North/East components
        float northVelocity = (float)(horizontalSpeed * Math.cos(bearingRad));
        float eastVelocity  = (float)(horizontalSpeed * Math.sin(bearingRad));

// DJI GROUND + VELOCITY mapping:
//   pitch = east/west
//   roll  = north/south
        float pitchVelocity = eastVelocity;
        float rollVelocity  = northVelocity;

// Proportional vertical speed: climb/descend based on altitude error
        float verticalVelocity = Kp_VERTICAL * altError;
        verticalVelocity = Math.max(-MAX_VERTICAL_SPEED,
                Math.min(MAX_VERTICAL_SPEED, verticalVelocity));

// Send the computed velocities to the drone
        sendVelocityCommand(pitchVelocity, rollVelocity, verticalVelocity);
    }

    // ---------------------------------------------------------------------------------
    // Virtual Stick Command
    // ---------------------------------------------------------------------------------

    /**
     * Sends a FlightControlData packet to the drone via Virtual Stick.
     *
     * Parameters (all in m/s in VELOCITY + GROUND mode):
     *  @param pitch    North/South velocity — positive = fly North
     *  @param roll     East/West velocity  — positive = fly East
     *  @param vertical Climb/descend rate  — positive = climb
     *
     * Yaw is held at 0 (no rotation). The drone's nose stays fixed.
     * To make the nose follow the flight direction, replace 0f with a
     * computed yaw rate based on bearingRad.
     */
    private void sendVelocityCommand(float pitch, float roll, float vertical) {
        if (flightController == null) return;

        // FlightControlData constructor: (pitch, roll, yaw, vertical)
        FlightControlData data = new FlightControlData(pitch, roll, 0f, vertical);

        flightController.sendVirtualStickFlightControlData(data, error -> {
            if (error != null) {
                Log.e(TAG, "Virtual stick send error: " + error.getDescription());
            }
        });
    }

    // ---------------------------------------------------------------------------------
    // Return to Home
    // ---------------------------------------------------------------------------------

    private void triggerRTH() {
        if (flightController == null) return;

        // Disable Virtual Stick first so RTH can take over
        flightController.setVirtualStickModeEnabled(false, error -> {
            flightController.startGoHome(rthError -> {
                if (rthError == null) {
                    post(() -> {
                        appendLog("RTH initiated successfully.");
                        tvStatus.setText("Mission: RTH");
                        btnStart.setEnabled(true);
                        btnStop.setEnabled(false);
                        btnAddWaypoint.setEnabled(true);
                        btnClearWaypoints.setEnabled(true);
                    });
                } else {
                    post(() -> appendLog("RTH failed: " + rthError.getDescription()));
                }
            });
        });
    }

    // ---------------------------------------------------------------------------------
    // Navigation Math
    // ---------------------------------------------------------------------------------

    /**
     * Haversine formula — computes the great-circle distance in meters between
     * two GPS coordinates.
     *
     * This is the standard formula for GPS distance calculation. It accounts for
     * the curvature of the Earth, though at drone-scale distances (~100s of meters)
     * a flat-Earth approximation would also work. Haversine is used here for correctness.
     *
     * @param lat1 current latitude  (degrees)
     * @param lng1 current longitude (degrees)
     * @param lat2 target latitude   (degrees)
     * @param lng2 target longitude  (degrees)
     * @return distance in meters
     */
    static double haversineDistance(double lat1, double lng1,
                                    double lat2, double lng2) {
        double dLat = Math.toRadians(lat2 - lat1);
        double dLng = Math.toRadians(lng2 - lng1);

        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2)
                + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2))
                * Math.sin(dLng / 2) * Math.sin(dLng / 2);

        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return EARTH_RADIUS_M * c;
    }

    /**
     * Computes the initial bearing (heading) from point 1 to point 2.
     *
     * Returns a value in radians, measured clockwise from true North (0 = North,
     * π/2 = East, π = South, 3π/2 = West).
     *
     * This bearing is used to decompose horizontal speed into North (pitch)
     * and East (roll) components via cos() and sin() respectively.
     *
     * @return bearing in radians [0, 2π)
     */
    static double bearing(double lat1, double lng1, double lat2, double lng2) {
        double lat1Rad = Math.toRadians(lat1);
        double lat2Rad = Math.toRadians(lat2);
        double dLng    = Math.toRadians(lng2 - lng1);

        double y = Math.sin(dLng) * Math.cos(lat2Rad);
        double x = Math.cos(lat1Rad) * Math.sin(lat2Rad)
                - Math.sin(lat1Rad) * Math.cos(lat2Rad) * Math.cos(dLng);

        // atan2 returns [-π, π]; normalize to [0, 2π]
        return (Math.atan2(y, x) + 2 * Math.PI) % (2 * Math.PI);
    }

    // ---------------------------------------------------------------------------------
    // UI helpers
    // ---------------------------------------------------------------------------------

    private void updateTargetLabel() {
        if (currentWaypointIndex < waypointList.size()) {
            double[] wp = waypointList.get(currentWaypointIndex);
            tvCurrentWaypoint.setText(String.format(
                    "Target WP%d: (%.6f, %.6f) @ %.1fm",
                    currentWaypointIndex + 1, wp[0], wp[1], wp[2]));
        } else {
            tvCurrentWaypoint.setText("Target: —");
        }
    }

    private void refreshWaypointList() {
        if (waypointList.isEmpty()) {
            tvWaypointList.setText("Waypoints: (none)");
            return;
        }
        StringBuilder sb = new StringBuilder("Waypoints:\n");
        for (int i = 0; i < waypointList.size(); i++) {
            double[] wp = waypointList.get(i);
            sb.append(String.format("  %d: (%.6f, %.6f) @ %.1fm\n",
                    i + 1, wp[0], wp[1], wp[2]));
        }
        tvWaypointList.setText(sb.toString());
    }

    private void appendLog(String message) {
        post(() -> {
            tvLog.append(message + "\n");
            scrollLog.post(() -> scrollLog.fullScroll(View.FOCUS_DOWN));
        });
    }

    private void showToast(String message) {
        post(() -> Toast.makeText(getContext(), message, Toast.LENGTH_SHORT).show());
    }

    // ---------------------------------------------------------------------------------
    // Lifecycle
    // ---------------------------------------------------------------------------------

    @Override
    protected void onDetachedFromWindow() {
        super.onDetachedFromWindow();
        // If mission is running when user navigates away, stop everything safely
        if (missionRunning) {
            missionRunning = false;
            stopControlLoop();
            sendVelocityCommand(0, 0, 0);
            if (flightLogger != null) flightLogger.stop();
        }
        // Always detach state callback and disable virtual stick on exit
        if (flightController != null) {
            flightController.setStateCallback(null);
            flightController.setVirtualStickModeEnabled(false, null);
        }
    }
}