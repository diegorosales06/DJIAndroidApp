package com.dji.sdk.sample.demo.waypoint;

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

import java.util.ArrayList;
import java.util.List;

import dji.common.error.DJIError;
import dji.common.mission.waypoint.Waypoint;
import dji.common.mission.waypoint.WaypointAction;
import dji.common.mission.waypoint.WaypointActionType;
import dji.common.mission.waypoint.WaypointMission;
import dji.common.mission.waypoint.WaypointMissionDownloadEvent;
import dji.common.mission.waypoint.WaypointMissionExecutionEvent;
import dji.common.mission.waypoint.WaypointMissionFinishedAction;
import dji.common.mission.waypoint.WaypointMissionFlightPathMode;
import dji.common.mission.waypoint.WaypointMissionHeadingMode;
import dji.common.mission.waypoint.WaypointMissionState;
import dji.common.mission.waypoint.WaypointMissionUploadEvent;
import dji.common.util.CommonCallbacks;
import dji.sdk.mission.waypoint.WaypointMissionOperator;
import dji.sdk.mission.waypoint.WaypointMissionOperatorListener;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;

/**
 * WaypointMissionView
 *
 * Demonstrates autonomous GPS waypoint navigation using the DJI Mobile SDK V4
 * WaypointMission system.
 *
 * How it works at a high level:
 *  1. User inputs a list of GPS waypoints (lat, lng, altitude).
 *  2. A WaypointMission object is built from those waypoints.
 *  3. The mission is uploaded to the drone's onboard flight controller.
 *  4. Once uploaded, the mission is started — the drone flies to each waypoint
 *     in order, confirms its position at each one, then moves to the next.
 *  5. Mission events (upload progress, waypoint reached, finished) are reported
 *     via a listener callback and displayed in the on-screen log.
 *
 * The drone's internal cascaded PID controller handles all the actual flight
 * control (position → velocity → attitude loops). We only provide GPS targets.
 *
 * To register this view in the sample app menu:
 *  - Add an entry in DemoListView.java pointing to WaypointMissionView.class
 *  - Add a string resource: <string name="waypoint_mission_title">Waypoint Mission</string>
 *
 * Package: com.dji.sdk.sample.demo.waypoint
 */
public class WaypointMissionView extends LinearLayout implements PresentableView {

    private static final String TAG = "WaypointMissionView";

    // Default cruise altitude in meters (above takeoff point, not sea level)
    // All waypoints use this altitude unless overridden per-waypoint
    private static final float DEFAULT_ALTITUDE_M = 10.0f;

    // Cruise speed in m/s — DJI SDK V4 range is 2.0 to 15.0 m/s
    private static final float CRUISE_SPEED_MS = 5.0f;

    // How close the drone must get to a waypoint before it counts as "reached" (meters)
    // Smaller = more precise but slower; larger = faster but less accurate
    private static final float WAYPOINT_RADIUS_M = 0.2f;

    // ---------------------------------------------------------------------------------
    // UI components
    // ---------------------------------------------------------------------------------
    private TextView   tvStatus;         // shows current mission state
    private TextView   tvWaypointList;   // shows the list of entered waypoints
    private EditText   etLat;            // latitude input field
    private EditText   etLng;            // longitude input field
    private EditText   etAlt;            // altitude input field (optional override)
    private Button     btnAddWaypoint;   // adds a waypoint to the list
    private Button     btnClearWaypoints;// clears the waypoint list
    private Button     btnUpload;        // uploads the mission to the drone
    private Button     btnStart;         // starts the uploaded mission
    private Button     btnStop;          // stops/cancels the mission mid-flight
    private ScrollView scrollLog;
    private TextView   tvLog;

    // ---------------------------------------------------------------------------------
    // Waypoint list — holds the user-defined GPS targets for this mission.
    // Pre-populated with one default waypoint at (120, 240) @ 360m altitude.
    // ---------------------------------------------------------------------------------
    private final List<Waypoint> waypointList = new ArrayList<Waypoint>() {{
        // Default hardcoded waypoint: lat=120, lng=240, alt=360m
        // Replace these with real coordinates before flying
        Waypoint defaultWp = new Waypoint(120.0, 240.0, 360.0f);
        defaultWp.addAction(new WaypointAction(WaypointActionType.STAY, 2000)); // hover 2s
        add(defaultWp);
    }};

    // ---------------------------------------------------------------------------------
    // DJI Mission Operator
    // The WaypointMissionOperator is the SDK object that manages the full
    // mission lifecycle: build → upload → start → monitor → stop
    // ---------------------------------------------------------------------------------
    private WaypointMissionOperator missionOperator;

    // Listener that receives all mission state change events from the SDK
    private WaypointMissionOperatorListener missionListener;

    // ---------------------------------------------------------------------------------
    // Constructors
    // ---------------------------------------------------------------------------------

    public WaypointMissionView(Context context) {
        super(context);
        init(context);
    }

    public WaypointMissionView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    // ---------------------------------------------------------------------------------
    // PresentableView interface
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
    // UI Construction — built programmatically, no XML layout needed
    // ---------------------------------------------------------------------------------

    private void init(Context context) {
        setOrientation(VERTICAL);
        setPadding(24, 24, 24, 24);

        // --- Mission status label ---
        tvStatus = new TextView(context);
        tvStatus.setText("Mission: IDLE");
        tvStatus.setTextSize(16f);
        tvStatus.setPadding(0, 0, 0, 12);
        addView(tvStatus);

        // --- Lat / Lng / Alt input row ---
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
        etAlt.setText(String.valueOf(DEFAULT_ALTITUDE_M));
        etAlt.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL);
        LinearLayout.LayoutParams etP3 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.6f);
        etLng.setLayoutParams(etP3);
        etAlt.setLayoutParams(etP3);
        inputRow.addView(etAlt);

        addView(inputRow);

        // --- Add / Clear waypoint buttons ---
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

        // --- Waypoint list display ---
        tvWaypointList = new TextView(context);
        tvWaypointList.setText("Waypoints: (none)");
        tvWaypointList.setTextSize(12f);
        tvWaypointList.setPadding(0, 0, 0, 12);
        addView(tvWaypointList);

        // --- Upload / Start / Stop mission buttons ---
        LinearLayout btnRow2 = new LinearLayout(context);
        btnRow2.setOrientation(HORIZONTAL);
        btnRow2.setPadding(0, 0, 0, 16);

        btnUpload = new Button(context);
        btnUpload.setText("Upload");
        LinearLayout.LayoutParams bP3 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bP3.setMarginEnd(6);
        btnUpload.setLayoutParams(bP3);
        btnUpload.setOnClickListener(v -> onUploadMission());
        btnRow2.addView(btnUpload);

        btnStart = new Button(context);
        btnStart.setText("Start");
        LinearLayout.LayoutParams bP4 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bP4.setMarginEnd(6);
        btnStart.setLayoutParams(bP4);
        btnStart.setEnabled(false); // disabled until mission is uploaded
        btnStart.setOnClickListener(v -> onStartMission());
        btnRow2.addView(btnStart);

        btnStop = new Button(context);
        btnStop.setText("Stop");
        LinearLayout.LayoutParams bP5 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        btnStop.setLayoutParams(bP5);
        btnStop.setEnabled(false); // disabled until mission is running
        btnStop.setOnClickListener(v -> onStopMission());
        btnRow2.addView(btnStop);

        addView(btnRow2);

        // --- Scrollable event log ---
        tvLog = new TextView(context);
        tvLog.setText("Log:\n");
        tvLog.setTextSize(11f);

        scrollLog = new ScrollView(context);
        scrollLog.setLayoutParams(new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 500));
        scrollLog.addView(tvLog);
        addView(scrollLog);

        // Get the WaypointMissionOperator from the SDK and register our listener
        initMissionOperator();

        // Populate the waypoint display with whatever is already in the list
        // (including the hardcoded default waypoint added at field initialization)
        refreshWaypointList();
    }

    // ---------------------------------------------------------------------------------
    // DJI Mission Operator initialisation
    // ---------------------------------------------------------------------------------

    /**
     * Retrieves the WaypointMissionOperator singleton from the DJI SDK manager.
     * This operator is the single interface for all waypoint mission operations.
     * We also register a listener here so we receive all state change callbacks
     * for the duration of this view's life.
     */
    private void initMissionOperator() {
        // The mission manager is accessed through DJISDKManager, not through the aircraft
        // directly — it persists as a singleton across the session
        if (DJISDKManager.getInstance() == null
                || DJISDKManager.getInstance().getMissionControl() == null) {
            appendLog("SDK not ready — mission operator unavailable.");
            return;
        }

        missionOperator = DJISDKManager.getInstance()
                .getMissionControl()
                .getWaypointMissionOperator();

        if (missionOperator == null) {
            appendLog("WaypointMissionOperator unavailable.");
            return;
        }

        appendLog("WaypointMissionOperator ready.");
        registerMissionListener();
    }

    // ---------------------------------------------------------------------------------
    // Mission Listener
    // ---------------------------------------------------------------------------------

    /**
     * Registers a WaypointMissionOperatorListener with the SDK.
     *
     * The listener has four callbacks:
     *  - onDownloadUpdate:  fired when the drone downloads a previously stored mission
     *  - onUploadUpdate:    fired repeatedly during mission upload, reporting progress
     *  - onExecutionUpdate: fired repeatedly during flight, reporting which waypoint
     *                       the drone is currently targeting and its progress
     *  - onExecutionFinish: fired once when the mission completes or is interrupted
     */
    private void registerMissionListener() {
        missionListener = new WaypointMissionOperatorListener() {

            // Called if the app retrieves a mission already stored on the drone
            // (e.g. from a previous session). Not commonly used but good to handle.
            @Override
            public void onDownloadUpdate(@NonNull WaypointMissionDownloadEvent event) {
                appendLog("Download update: " + event.getProgress());
            }

            // Called the moment the drone begins executing the mission (takes off / starts moving).
            // Required by the WaypointMissionOperatorListener interface.
            @Override
            public void onExecutionStart() {
                appendLog("Mission execution started.");
                post(() -> tvStatus.setText("Mission: EXECUTING"));
            }

            // Called repeatedly as the mission uploads to the drone.
            // progress.uploadedWaypointIndex tells you which waypoint just finished uploading.
            @Override
            public void onUploadUpdate(@NonNull WaypointMissionUploadEvent event) {
                if (event.getProgress() != null) {
                    int uploaded = event.getProgress().uploadedWaypointIndex + 1;
                    int total    = event.getProgress().totalWaypointCount;
                    appendLog("Uploading waypoints: " + uploaded + " / " + total);
                }

                // When upload is complete, the mission state transitions to READY_TO_EXECUTE
                // At that point we enable the Start button
                if (missionOperator.getCurrentState()
                        == WaypointMissionState.READY_TO_EXECUTE) {
                    appendLog("Upload complete — ready to start.");
                    post(() -> {
                        btnStart.setEnabled(true);
                        btnUpload.setEnabled(false);
                        tvStatus.setText("Mission: READY");
                    });
                }
            }

            // Called ~10Hz during active flight execution.
            // event.getProgress().targetWaypointIndex = which waypoint the drone is flying to
            // event.getProgress().isWaypointReached   = true the moment the drone arrives
            @Override
            public void onExecutionUpdate(@NonNull WaypointMissionExecutionEvent event) {
                if (event.getProgress() != null) {
                    int targetIdx = event.getProgress().targetWaypointIndex;
                    boolean reached = event.getProgress().isWaypointReached;

                    // Only log the moment a waypoint is confirmed reached to avoid
                    // flooding the log at 10Hz during transit
                    if (reached) {
                        appendLog("Waypoint " + (targetIdx + 1) + " reached.");
                        post(() -> tvStatus.setText(
                                "Mission: flying to waypoint " + (targetIdx + 2)));
                    }

                    Log.d(TAG, "Target waypoint: " + targetIdx + "  reached: " + reached);
                }
            }

            // Called once when the mission ends — either successfully completed,
            // interrupted by Stop, or failed due to an error
            @Override
            public void onExecutionFinish(DJIError error) {
                if (error == null) {
                    appendLog("Mission completed successfully.");
                } else {
                    appendLog("Mission ended with error: " + error.getDescription());
                }
                post(() -> {
                    tvStatus.setText("Mission: FINISHED");
                    btnStop.setEnabled(false);
                    btnUpload.setEnabled(true);
                    btnStart.setEnabled(false);
                });
            }
        };

        // Register the listener with the operator
        missionOperator.addListener(missionListener);
        appendLog("Mission listener registered.");
    }

    // ---------------------------------------------------------------------------------
    // Button handlers
    // ---------------------------------------------------------------------------------

    /**
     * Parses the lat/lng/alt fields and adds a new Waypoint to the list.
     *
     * Each DJI Waypoint object holds:
     *  - coordinate (lat, lng)
     *  - altitude above takeoff point in meters
     *  - optional WaypointActions (hover, take photo, rotate gimbal, etc.)
     */
    private void onAddWaypoint() {
        String latStr = etLat.getText().toString().trim();
        String lngStr = etLng.getText().toString().trim();
        String altStr = etAlt.getText().toString().trim();

        if (latStr.isEmpty() || lngStr.isEmpty()) {
            showToast("Please enter latitude and longitude.");
            return;
        }

        double lat, lng;
        float  alt;
        try {
            lat = Double.parseDouble(latStr);
            lng = Double.parseDouble(lngStr);
            alt = altStr.isEmpty() ? DEFAULT_ALTITUDE_M : Float.parseFloat(altStr);
        } catch (NumberFormatException e) {
            showToast("Invalid coordinates.");
            return;
        }

        if (lat < -90 || lat > 90 || lng < -180 || lng > 180) {
            showToast("Coordinates out of valid range.");
            return;
        }

        // Create the DJI Waypoint object at the given coordinate and altitude
        Waypoint wp = new Waypoint(lat, lng, alt);

        // Optional: add a hover action at each waypoint (hovering for 2 seconds).
        // WaypointActionType options include:
        //   STAY             — hover for N milliseconds
        //   START_TAKE_PHOTO — trigger camera shutter
        //   START_RECORD     — start video recording
        //   STOP_RECORD      — stop video recording
        //   ROTATE_AIRCRAFT  — yaw to a heading
        //   GIMBAL_PITCH     — tilt the gimbal
        wp.addAction(new WaypointAction(WaypointActionType.STAY, 2000)); // hover 2s

        waypointList.add(wp);
        etLat.setText("");
        etLng.setText("");
        refreshWaypointList();
        appendLog(String.format("Added waypoint %d: (%.6f, %.6f) @ %.1fm",
                waypointList.size(), lat, lng, alt));
    }

    /** Clears the waypoint list. Can only be done when no mission is active. */
    private void onClearWaypoints() {
        waypointList.clear();
        refreshWaypointList();
        btnUpload.setEnabled(true);
        btnStart.setEnabled(false);
        appendLog("Waypoint list cleared.");
    }

    /**
     * Builds a WaypointMission from the current waypoint list and uploads it
     * to the drone's onboard flight controller.
     *
     * Upload must complete before the mission can be started. The onUploadUpdate
     * callback will report progress and enable the Start button when done.
     *
     * Key WaypointMission settings:
     *  - maxFlightSpeed:    hard cap on speed (2–15 m/s)
     *  - autoFlightSpeed:   cruise speed during transit between waypoints
     *  - finishedAction:    what to do when the last waypoint is reached
     *      NO_ACTION        — hover in place
     *      GO_HOME          — auto RTH
     *      AUTO_LAND        — land immediately at last waypoint
     *      GO_FIRST_WAYPOINT— loop back to waypoint 0
     *  - headingMode:       how the drone's nose is oriented during flight
     *      AUTO             — nose always points in the direction of travel
     *      USING_WAYPOINT_HEADING — use per-waypoint heading if set
     *  - flightPathMode:
     *      NORMAL           — fly straight lines between waypoints
     *      CURVED           — smooth curved path (requires cornerRadiusInMeters)
     */
    private void onUploadMission() {
        if (waypointList.size() < 2) {
            showToast("Need at least 2 waypoints to run a mission.");
            return;
        }

        if (missionOperator == null) {
            initMissionOperator();
            if (missionOperator == null) {
                showToast("Mission operator not available.");
                return;
            }
        }

        // Build the WaypointMission using the builder pattern
        WaypointMission.Builder builder = new WaypointMission.Builder();

        builder.waypointList(waypointList)              // the GPS targets
                .waypointCount(waypointList.size())      // must match list size
                .maxFlightSpeed(15.0f)                   // hard speed cap (m/s)
                .autoFlightSpeed(CRUISE_SPEED_MS)        // cruise speed (m/s)
                .finishedAction(WaypointMissionFinishedAction.GO_HOME)   // RTH when done
                .headingMode(WaypointMissionHeadingMode.AUTO)            // nose follows path
                .flightPathMode(WaypointMissionFlightPathMode.NORMAL);   // straight lines
        // Note: isGimbalPitchRotationEnabled() and exitMissionOnRCSignalLostEnabled()
        // are not available as builder setters in this SDK version — omitted

        // Validate the mission before uploading — the SDK checks for things like
        // waypoints being too close together, invalid altitudes, etc.
        DJIError validationError = builder.checkParameters();
        if (validationError != null) {
            appendLog("Mission validation failed: " + validationError.getDescription());
            showToast("Mission invalid — check log.");
            return;
        }

        WaypointMission mission = builder.build();

        appendLog("Uploading mission (" + waypointList.size() + " waypoints)...");
        tvStatus.setText("Mission: UPLOADING");
        btnUpload.setEnabled(false);

        // Upload the mission to the drone.
        // NOTE: uploadMission() takes only the mission object — no callback here.
        // Upload results come through the listener's onUploadUpdate() method above.
        DJIError uploadError = missionOperator.loadMission(mission);
        if (uploadError != null) {
            appendLog("Upload failed: " + uploadError.getDescription());
            btnUpload.setEnabled(true);
            tvStatus.setText("Mission: UPLOAD FAILED");
            return;
        }
        missionOperator.uploadMission(error -> {
            if (error != null) {
                appendLog("Upload transmission error: " + error.getDescription());
                post(() -> {
                    btnUpload.setEnabled(true);
                    tvStatus.setText("Mission: UPLOAD FAILED");
                });
            }
        });
    }

    /**
     * Starts the previously uploaded mission.
     * The drone will take off (if on the ground), fly to each waypoint in order,
     * execute any actions at each waypoint, and then perform the finishedAction.
     *
     * IMPORTANT: The drone must have a valid GPS fix and be armed before this works.
     * The SDK will return an error if conditions are not met.
     */
    private void onStartMission() {
        if (missionOperator == null) {
            showToast("Mission operator not available.");
            return;
        }

        appendLog("Starting mission...");
        tvStatus.setText("Mission: STARTING");

        missionOperator.startMission(error -> {
            if (error == null) {
                appendLog("Mission started — drone is flying.");
                post(() -> {
                    tvStatus.setText("Mission: EXECUTING");
                    btnStart.setEnabled(false);
                    btnStop.setEnabled(true);  // enable Stop once mission is running
                });
            } else {
                appendLog("Failed to start mission: " + error.getDescription());
                post(() -> tvStatus.setText("Mission: START FAILED"));
            }
        });
    }

    /**
     * Stops the mission mid-flight.
     * The drone will halt and hover in place at its current position.
     * After stopping, you can resume with startMission() or take manual control.
     */
    private void onStopMission() {
        if (missionOperator == null) return;

        appendLog("Stopping mission...");

        missionOperator.stopMission(error -> {
            if (error == null) {
                appendLog("Mission stopped. Drone hovering.");
                post(() -> {
                    tvStatus.setText("Mission: STOPPED");
                    btnStop.setEnabled(false);
                    btnUpload.setEnabled(true);
                    btnStart.setEnabled(false);
                });
            } else {
                appendLog("Failed to stop mission: " + error.getDescription());
            }
        });
    }

    // ---------------------------------------------------------------------------------
    // UI helpers
    // ---------------------------------------------------------------------------------

    /** Rebuilds the waypoint list display from the current waypointList. */
    private void refreshWaypointList() {
        if (waypointList.isEmpty()) {
            tvWaypointList.setText("Waypoints: (none)");
            return;
        }
        StringBuilder sb = new StringBuilder("Waypoints:\n");
        for (int i = 0; i < waypointList.size(); i++) {
            Waypoint wp = waypointList.get(i);
            sb.append(String.format("  %d: (%.6f, %.6f) @ %.1fm\n",
                    i + 1, wp.coordinate.getLatitude(),
                    wp.coordinate.getLongitude(), wp.altitude));
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

    /**
     * Clean up the mission listener when the view is destroyed.
     * Failing to remove the listener would cause a memory leak since the
     * WaypointMissionOperator is a singleton that outlives this view.
     */
    @Override
    protected void onDetachedFromWindow() {
        super.onDetachedFromWindow();
        if (missionOperator != null && missionListener != null) {
            missionOperator.removeListener(missionListener);
        }
    }
}