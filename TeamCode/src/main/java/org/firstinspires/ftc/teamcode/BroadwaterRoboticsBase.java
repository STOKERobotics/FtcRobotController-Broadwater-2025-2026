package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * BroadwaterRobotBase - Shared functionality for TeleOp and Autonomous
 *
 * This base class contains all common robot hardware, constants, and methods.
 * Changes here automatically propagate to both TeleOp and Autonomous modes.
 */
public abstract class BroadwaterRoboticsBase extends LinearOpMode {

    // ==================== HARDWARE ====================
    protected DcMotor motor0, motor1, motor2, motor3;           // Drive motors
    protected DcMotor motor0b, motor1b, motor2b;                 // Shooter + Intake
    protected Servo servo0, servo2;                              // Kicker, Shooter Angle
    protected CRServo servo1;                                    // Merry Go Round Tray
    protected BNO055IMU imu1;
    //    protected BNO055IMU imu2;
    protected DigitalChannel blueLED, redLED;
    protected DigitalChannel mag0, mag1, mag2, mag3;
    protected NormalizedColorSensor ballColor;
    protected Limelight3A limelight;
    protected AnalogInput laser;
    protected IMU imu2;   // REV 9-Axis IMU (new SDK IMU)

    // ==================== CACHED STATES ====================
    protected boolean mag0State, mag1State, mag2State, mag3State;
    protected String ballColorValue;
    protected String[] slots = new String[3];
    protected final boolean[] slotFired = new boolean[3];

    // Magnet timing for robust detection (tracks when each magnet was last seen)
    protected long intakeTopSeenTime = 0;    // mag0
    protected long intakeBottomSeenTime = 0; // mag1
    protected long shootTopSeenTime = 0;     // mag2
    protected long shootBottomSeenTime = 0;  // mag3
    protected static final long MAGNET_WINDOW_MS = 250; // Detection window for timing tolerance

    // ==================== CONSTANTS ====================
    // Alignment
    protected static final double ALIGN_KP = 0.02;
    protected static final double ALIGN_TOLERANCE = 1.0;
    protected static final double ALIGN_MAX_POWER = 0.25;
    protected static final long ALIGN_STABLE_MS = 200;

    // Shooter
    protected static final double KICK_EXTEND_POS = 0.0;
    protected static final double KICK_RETRACT_POS = 1.0;
    protected static final long KICK_DURATION_MS = 300;   // extend time
    protected static final long KICK_RETRACT_WAIT_MS = 200; // retract settle

    // Merry-go-round servo
    protected static final double MGR_FAST_POWER = .75;
    protected static final double MGR_CRAWL_POWER = 0.08;
    protected static final long MGR_BRAKE_MS = 0;
    protected static final double MGR_BRAKE_POWER = -0.0;
    protected static final long MGR_MOVE_TIMEOUT_MS = 10000;
    protected static final int INTAKE_SLOT_STEP = 1;
    protected static final int INTAKE_TO_SHOOT_OFFSET = 0;
    protected String lastShootRotateStatus = "none";
    protected long lastShootRotateStatusMs = 0;


    // Color sensor thresholds
    protected static final float COLOR_SAT_THRESHOLD = 0.05f;
    protected static final float COLOR_VAL_THRESHOLD = 0.009f;
    protected static final float HUE_GREEN_MIN = 120f;
    protected static final float HUE_GREEN_MAX = 180f;
    protected static final float HUE_PURPLE_MIN = 210f;
    protected static final float HUE_PURPLE_MAX = 255f;

    // Telemetry
    protected static final long TELEM_PERIOD_MS = 500;

    // Camera offset
    protected static final double CAMERA_X_OFFSET_IN = 5.0;
    protected static final double IN_TO_M = 0.0254;

    // Movement constants (for autonomous)
    protected static final double WHEEL_DIAMETER_M = 0.096;
    protected static final double WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M;
    protected static final double TICKS_PER_WHEEL_REV = 537.7;
    protected static final double HEADING_SIGN = +1.0;
    protected static final double HEADING_DEADBAND_DEG = 1.0;

    // ==================== STATE VARIABLES ====================
    protected enum INTAKEState { INIT_TO_SLOT0, WAIT_COLOR_0, MOVE_TO_SLOT1, WAIT_COLOR_1, MOVE_TO_SLOT2, WAIT_COLOR_2, DONE }
    protected INTAKEState intakeState = INTAKEState.INIT_TO_SLOT0;

    protected int currentSlot = 0;
    protected boolean colorLatched = false;
    protected boolean shootingBusy = false;
    protected boolean mgrRetractDone = false;

    // Motif tracking
    protected boolean motifLatched = false;
    protected int latchedTagId = -1;
    protected String latchedMotif = "NONE";
    protected double lastTagDistanceIn = -1;
    protected boolean motifListenEnabled = false;

    // Step shoot
    protected int stepShotIndex = 0;
    protected boolean stepModeActive = false;

    // Servo2 timing
    protected double servo2Pos = 1;
    protected double lastServo2Time = 0.0;

    // LED states
    protected boolean isRedOn = false;
    protected boolean isBlueOn = false;

    // Telemetry throttle
    protected long lastTelemMs = 0;

    // Shooter slot detection stability
    protected static final long SHOOT_STABLE_MS = 120;      // must hold pattern this long
    protected static final long SHOOT_MIN_SPIN_MS = 250;    // ignore early false hits
    protected static final long SHOOT_LEAVE_MS = 250;       // move off if starting on target

    // ==================== INITIALIZATION ====================
    protected void initializeHardware() {
        // Initialize Limelight first
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        // Drive motors
        motor0 = initMotor("motor0", DcMotor.Direction.FORWARD);
        motor1 = initMotor("motor1", DcMotor.Direction.REVERSE);
        motor2 = initMotor("motor2", DcMotor.Direction.FORWARD);
        motor3 = initMotor("motor3", DcMotor.Direction.FORWARD);

        // Shooter and intake motors
        motor0b = initMotor("motor0b", DcMotor.Direction.REVERSE);
        motor1b = initMotor("motor1b", DcMotor.Direction.FORWARD);
        motor2b = initMotor("motor2b", DcMotorSimple.Direction.FORWARD);

        // Servos
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo0.setPosition(KICK_RETRACT_POS);
        servo1.setDirection(DcMotorSimple.Direction.REVERSE);
        servo2.setPosition(servo2Pos);
        lastServo2Time = getRuntime();

        // Sensors
        laser = hardwareMap.get(AnalogInput.class, "laser");
        ballColor = hardwareMap.get(NormalizedColorSensor.class, "ballColor");
        if (ballColor instanceof SwitchableLight) {
            ((SwitchableLight)ballColor).enableLight(true);
        }

        // Magnets
        mag0 = initDigitalChannel("mag0");
        mag1 = initDigitalChannel("mag1");
        mag2 = initDigitalChannel("mag2");
        mag3 = initDigitalChannel("mag3");

        // LEDs
        redLED = hardwareMap.get(DigitalChannel.class, "redLED");
        blueLED = hardwareMap.get(DigitalChannel.class, "blueLED");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        blueLED.setMode(DigitalChannel.Mode.OUTPUT);

        // IMU
        imu1 = hardwareMap.get(BNO055IMU.class, "imu 1");
        initIMU((BNO055IMU) imu1, false);

        // IMU #2 (REV 9-Axis IMU)
        imu2 = hardwareMap.get(IMU.class, "imu2");

        // IMPORTANT: Set orientation of the Control/Expansion Hub on the robot.
        // These defaults are common if the REV Hub logo faces up and USB faces forward.
        // If your hub is mounted differently, update these two.
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usb  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logo, usb);
        IMU.Parameters imu2Params = new IMU.Parameters(orientation);

        imu2.initialize(imu2Params);
        imu2.resetYaw();  // optional: zero yaw at init
        // ---- AUTO ZERO ARM ANGLE ----
        // Assumes servo2 is already at 0.5 (neutral arm position)
        sleep(150);           // allow IMUs + servo to settle
        zeroArmAngle();       // capture imu1 yaw + imu2 pitch as zero
        updateArmAngle();     // initialize armAngleDeg = 0
    }

    protected DcMotor initMotor(String name, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }

    protected DigitalChannel initDigitalChannel(String name) {
        DigitalChannel channel = hardwareMap.get(DigitalChannel.class, name);
        channel.setMode(DigitalChannel.Mode.INPUT);
        return channel;
    }

    protected void initIMU(BNO055IMU imu, boolean logging) {
        Parameters params = new Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = logging;
        imu.initialize(params);
    }

    // ==================== MAGNET STATE MANAGEMENT ====================
    protected void updateMagnetStates() {
        mag0State = mag0.getState();
        mag1State = mag1.getState();
        mag2State = mag2.getState();
        mag3State = mag3.getState();

        // Track when each magnet was last detected for robust slot detection
        long now = System.currentTimeMillis();
        if (!mag0State) intakeTopSeenTime = now;    // mag0 = intake top
        if (!mag1State) intakeBottomSeenTime = now; // mag1 = intake bottom
        if (!mag2State) shootTopSeenTime = now;     // mag2 = shoot top
        if (!mag3State) shootBottomSeenTime = now;  // mag3 = shoot bottom
    }

    // Reset timing when starting a new rotation to avoid false positives
    protected void resetMagnetTiming() {
        intakeTopSeenTime = 0;
        intakeBottomSeenTime = 0;
        shootTopSeenTime = 0;
        shootBottomSeenTime = 0;
    }

    // Intake slot detectors (use cached states with timing window for robustness)
    // Slot 0 = BOTH magnets (divider to right of slot 0)
    protected boolean atSlot0() {
        // Immediate detection: both magnets currently detected
        if (!mag0State && !mag1State) return true;

        // Robust detection: both magnets seen within timing window
        // This handles cases where magnets don't trigger simultaneously
        long now = System.currentTimeMillis();
        boolean topRecent = (now - intakeTopSeenTime) < MAGNET_WINDOW_MS;
        boolean bottomRecent = (now - intakeBottomSeenTime) < MAGNET_WINDOW_MS;
        return topRecent && bottomRecent;
    }

    // Slot 1 = TOP magnet only (divider between slot 0 and 1)
    protected boolean atSlot1() {
        // Immediate detection
        if (!mag0State && mag1State) return true;

        // Robust detection: top seen recently, bottom NOT currently detected
        long now = System.currentTimeMillis();
        boolean topRecent = (now - intakeTopSeenTime) < MAGNET_WINDOW_MS;
        return topRecent && mag1State;
    }

    // Slot 2 = BOTTOM magnet only (divider between slot 1 and 2)
    protected boolean atSlot2() {
        // Immediate detection
        if (mag0State && !mag1State) return true;

        // Robust detection: bottom seen recently, top NOT currently detected
        long now = System.currentTimeMillis();
        boolean bottomRecent = (now - intakeBottomSeenTime) < MAGNET_WINDOW_MS;
        return bottomRecent && mag0State;
    }

    // ==================== SHOOTER SLOT DETECTION (SIMPLE) ====================
    // mag2 = top sensor, mag3 = bottom sensor
    // magXState = true means NO magnet, false means magnet detected

    // Slot 0: BOTTOM magnet only (top=no magnet, bottom=magnet)
    protected boolean atShootSlot0() {
        return mag2State && !mag3State;
    }

    // Slot 1: TOP magnet only (top=magnet, bottom=no magnet)
    protected boolean atShootSlot1() {
        return !mag2State && mag3State;
    }

    // Slot 2: BOTH magnets (top=magnet, bottom=magnet)
    protected boolean atShootSlot2() {
        return !mag2State && !mag3State;
    }

    protected boolean atIntakeSlot(int slot) {
        switch (slot) {
            case 0: return atSlot0();
            case 1: return atSlot1();
            case 2: return atSlot2();
            default: return false;
        }
    }

    protected boolean atShootSlot(int slot) {
        switch (slot) {
            case 0: return atShootSlot0();
            case 1: return atShootSlot1();
            case 2: return atShootSlot2();
            default: return false;
        }
    }

    // Last known slots - never returns -1, holds previous value
    protected int lastKnownIntakeSlot = 0;
    protected int lastKnownShootSlot = 0;

    // Use IMMEDIATE detection only for current slot queries (telemetry, next-slot calc)
    // This avoids false positives from timing-based robust detection
    protected int getCurrentIntakeSlot() {
        // Immediate pattern matching only - no timing
        if (!mag0State && !mag1State) { lastKnownIntakeSlot = 0; return 0; } // Both
        if (!mag0State && mag1State) { lastKnownIntakeSlot = 1; return 1; }  // Top only
        if (mag0State && !mag1State) { lastKnownIntakeSlot = 2; return 2; }  // Bottom only
        return lastKnownIntakeSlot;
    }

    protected int getCurrentShootSlot() {
        // Immediate pattern matching only - no timing
        if (mag2State && !mag3State) { lastKnownShootSlot = 0; return 0; }   // Bottom only
        else if (!mag2State && mag3State) { lastKnownShootSlot = 1; return 1; }   // Top only
        else if (!mag2State && !mag3State) { lastKnownShootSlot = 2; return 2; }  // Both
        return lastKnownShootSlot;
    }

    // ==================== COLOR SENSOR ====================
    protected void updateBallColor() {
        NormalizedRGBA c = ballColor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.RGBToHSV((int)(c.red * 255), (int)(c.green * 255), (int)(c.blue * 255), hsv);

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        boolean confident = (sat > COLOR_SAT_THRESHOLD) && (val > COLOR_VAL_THRESHOLD);

        if (confident && (hue >= HUE_GREEN_MIN && hue <= HUE_GREEN_MAX)) {
            ballColorValue = "g";
        } else if (confident && (hue >= HUE_PURPLE_MIN && hue <= HUE_PURPLE_MAX)) {
            ballColorValue = "p";
        } else {
            ballColorValue = null;
        }
    }

    protected boolean colorSeen() {
        return ballColorValue != null && !ballColorValue.isEmpty();
    }

    // ==================== SERVO CONTROL ====================
    // Stores last kick timing for telemetry review
    protected long lastKickTotalMs = 0;
    protected long lastKickExtendMs = 0;
    protected long lastKickRetractMs = 0;

    protected void kickOnce() {
        long start = System.currentTimeMillis();

        // EXTEND (snap)
        servo0.setPosition(KICK_EXTEND_POS);
        sleep(KICK_DURATION_MS);

        // RETRACT immediately
        servo0.setPosition(KICK_RETRACT_POS);
        sleep(KICK_RETRACT_WAIT_MS);

        // Timing for telemetry
        long end = System.currentTimeMillis();
        lastKickTotalMs = end - start;
        lastKickExtendMs = KICK_DURATION_MS;
        lastKickRetractMs = KICK_RETRACT_WAIT_MS;
    }

    protected void ensureKickerRetracted() {
        servo0.setPosition(KICK_RETRACT_POS);
        sleep(KICK_RETRACT_WAIT_MS);
    }

    // ==================== SHOOTER ROTATION (SIMPLE) ====================
    // Rotate to a specific shooter slot by number (0, 1, or 2)
    protected boolean rotateToShootSlot(int targetSlot) {
        shootingBusy = true;

        final long TIMEOUT_MS = 2500;   // shorter so it doesn't "feel endless"
        final long HOLD_MS = 80;        // stable window
        final long IGNORE_MS = 150;     // ignore first bit after starting

        int seen00 = 0, seen01 = 0, seen10 = 0, seen11 = 0;

        try {
            ensureKickerRetracted();
            updateMagnetStates();

            // If already at target, move off briefly
            if (atShootSlot(targetSlot)) {
                servo1.setPower(MGR_CRAWL_POWER);
                long leaveStart = System.currentTimeMillis();
                while (opModeIsActive() && System.currentTimeMillis() - leaveStart < 200) {
                    updateMagnetStates();
                    if (!atShootSlot(targetSlot)) break;
                }
                servo1.setPower(0);
                sleep(50);
            }

            long start = System.currentTimeMillis();
            long deadline = start + TIMEOUT_MS;

            servo1.setPower(MGR_FAST_POWER);

            boolean hit = false;

            while (opModeIsActive() && System.currentTimeMillis() < deadline) {
                updateMagnetStates();

                // Track what we're seeing (mag2/mag3 as 0/1)
                int b2 = mag2State ? 1 : 0;
                int b3 = mag3State ? 1 : 0;
                if (b2 == 0 && b3 == 0) seen00++;
                else if (b2 == 0 && b3 == 1) seen01++;
                else if (b2 == 1 && b3 == 0) seen10++;
                else seen11++;

                long now = System.currentTimeMillis();
                if (now - start < IGNORE_MS) continue;

                if (atShootSlot(targetSlot)) {
                    long holdStart = now;
                    while (opModeIsActive() && System.currentTimeMillis() < deadline) {
                        updateMagnetStates();
                        if (!atShootSlot(targetSlot)) break;
                        if (System.currentTimeMillis() - holdStart >= HOLD_MS) {
                            hit = true;
                            break;
                        }
                    }
                    if (hit) break;
                }
            }

            servo1.setPower(0);

            if (hit) {
                setShootRotateStatus("OK -> slot " + targetSlot +
                        " raw=" + (mag2State ? "1" : "0") + (mag3State ? "1" : "0"));
                return true;
            } else {
                setShootRotateStatus("FAIL slot " + targetSlot +
                        " last=" + (mag2State ? "1" : "0") + (mag3State ? "1" : "0") +
                        " seen 00:" + seen00 + " 01:" + seen01 + " 10:" + seen10 + " 11:" + seen11);
                return false;
            }

        } finally {
            servo1.setPower(0);
            shootingBusy = false;
        }
    }


    // Keep intake rotation separate (uses timing-based detection which works)
    protected void rotateToSlotBlocking(int targetSlot, boolean useShootMagnets) {
        if (useShootMagnets) {
            rotateToShootSlot(targetSlot);
            return;
        }

        // Intake rotation (unchanged - this works)
        shootingBusy = true;
        try {
            ensureKickerRetracted();
            resetMagnetTiming();

            updateMagnetStates();
            boolean atTarget = atIntakeSlot(targetSlot);
            if (atTarget) {
                long leaveStart = System.currentTimeMillis();
                servo1.setPower(MGR_CRAWL_POWER);
                while (opModeIsActive() && (System.currentTimeMillis() - leaveStart) < 400) {
                    updateMagnetStates();
                    atTarget = atIntakeSlot(targetSlot);
                    if (!atTarget) break;
                }
                servo1.setPower(0);
                resetMagnetTiming();
            }

            long start = System.currentTimeMillis();
            servo1.setPower(MGR_FAST_POWER);

            while (opModeIsActive() && (System.currentTimeMillis() - start) < MGR_MOVE_TIMEOUT_MS) {
                updateMagnetStates();
                atTarget = atIntakeSlot(targetSlot);
                if (atTarget) {
                    servo1.setPower(0);
                    break;
                }
            }

            servo1.setPower(0);

            if (atTarget && MGR_BRAKE_MS > 0) {
                servo1.setPower(MGR_BRAKE_POWER);
                sleep(MGR_BRAKE_MS);
                servo1.setPower(0);
            }

        } finally {
            servo1.setPower(0);
            shootingBusy = false;
        }
    }

    // ==================== INTAKE STATE MACHINE ====================
    protected INTAKEState waitStateForSlot(int slot) {
        switch (slot) {
            case 0: return INTAKEState.WAIT_COLOR_0;
            case 1: return INTAKEState.WAIT_COLOR_1;
            case 2: return INTAKEState.WAIT_COLOR_2;
            default: return INTAKEState.WAIT_COLOR_0;
        }
    }

    protected void runIntakeStateMachine() {
        if (shootingBusy) {
            servo1.setPower(0);
            return;
        }

        // Update color sensor
        if (intakeState != INTAKEState.DONE) {
            updateBallColor();
        }

        boolean seen = colorSeen();
        boolean newColorEvent = seen && !colorLatched;
        colorLatched = seen;

        switch (intakeState) {
            case INIT_TO_SLOT0:
                if (!mgrRetractDone) {
                    ensureKickerRetracted();
                    resetMagnetTiming(); // Reset timing before starting rotation
                    mgrRetractDone = true;
                }
                servo1.setPower(MGR_FAST_POWER);
                if (atSlot0()) {
                    servo1.setPower(0);
                    currentSlot = 0;
                    intakeState = INTAKEState.WAIT_COLOR_0;
                    colorLatched = false;
                    mgrRetractDone = false;
                }
                break;

            case WAIT_COLOR_0:
                servo1.setPower(0);
                if (newColorEvent) {
                    slots[0] = ballColorValue;
                    intakeState = INTAKEState.MOVE_TO_SLOT1;
                }
                break;

            case MOVE_TO_SLOT1:
                if (!mgrRetractDone) {
                    ensureKickerRetracted();
                    resetMagnetTiming(); // Reset timing before starting rotation
                    mgrRetractDone = true;
                }
                servo1.setPower(MGR_FAST_POWER);
                if (atSlot1()) {
                    servo1.setPower(0);
                    currentSlot = 1;
                    intakeState = INTAKEState.WAIT_COLOR_1;
                    colorLatched = false;
                    mgrRetractDone = false;
                }
                break;

            case WAIT_COLOR_1:
                servo1.setPower(0);
                if (newColorEvent) {
                    slots[1] = ballColorValue;
                    intakeState = INTAKEState.MOVE_TO_SLOT2;
                }
                break;

            case MOVE_TO_SLOT2:
                if (!mgrRetractDone) {
                    ensureKickerRetracted();
                    resetMagnetTiming(); // Reset timing before starting rotation
                    mgrRetractDone = true;
                }
                servo1.setPower(MGR_FAST_POWER);
                if (atSlot2()) {
                    servo1.setPower(0);
                    currentSlot = 2;
                    intakeState = INTAKEState.WAIT_COLOR_2;
                    colorLatched = false;
                    mgrRetractDone = false;
                }
                break;

            case WAIT_COLOR_2:
                servo1.setPower(0);
                if (newColorEvent) {
                    slots[2] = ballColorValue;
                    intakeState = INTAKEState.DONE;
                }
                break;

            case DONE:
                servo1.setPower(0);
                break;
        }
    }

    // ==================== ALIGNMENT ====================
    protected boolean alignToTarget() {
        LLResult result = limelight.getLatestResult();
        double distTag = getBestTagDistanceInches(result);
        lastTagDistanceIn = distTag; // keep your existing variable updated

        telemetry.addData("distTag (in)", "%.1f", distTag);

        if (result == null || !result.isValid()) {
            stopDrive();
            return false;
        }

        // Find best alignment tag (20 or 24)
        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult t : result.getFiducialResults()) {
            int id = t.getFiducialId();
            if (id == 20 || id == 24) {
                if (best == null || t.getTargetArea() > best.getTargetArea()) {
                    best = t;
                }
            }
        }

        if (best == null) {
            stopDrive();
            return false;
        }

        // Compute alignment error
        Pose3D pose = best.getTargetPoseCameraSpace();
        if (pose == null) {
            stopDrive();
            return false;
        }

        double xCam = pose.getPosition().x;
        double zCam = pose.getPosition().z;
        double xCorrected = xCam + CAMERA_X_OFFSET_IN * IN_TO_M;
        double tx = Math.toDegrees(Math.atan2(xCorrected, zCam));

        // Check if aligned
        if (Math.abs(tx) <= ALIGN_TOLERANCE) {
            stopDrive();
            return true;
        }

        // Apply turning correction
        double turn = clamp(-tx * ALIGN_KP, -ALIGN_MAX_POWER, ALIGN_MAX_POWER);
        driveRobotCentric(0, 0, turn);

        return false;
    }

    protected void adjustShooterAndFire() {
        if (!motifLatched || latchedMotif == null || latchedMotif.length() != 3 || "UNKNOWN".equals(latchedMotif)) {
            return;
        }
        if (!allSlotsLoaded() || lastTagDistanceIn <= 0) {
            return;
        }

        // Set shooter angle
        servo2.setPosition(servo2Pos);

        // Reset fired tracking
        for (int i = 0; i < 3; i++) slotFired[i] = false;

        // Fire sequence
        char[] order = latchedMotif.toCharArray();
        for (int shotIndex = 0; shotIndex < 3 && opModeIsActive(); shotIndex++) {
            char wanted = order[shotIndex];
            int slotToShoot = findSlotForColor(wanted);

            if (slotToShoot < 0) return;

            int shootFrameSlot = intakeSlotToShootSlot(slotToShoot);

            // Rotate to position (kicker already retracted)
            rotateToSlotBlocking(shootFrameSlot, true);
            kickOnce(); // This extends, waits, then retracts kicker

            slotFired[slotToShoot] = true;
            slots[slotToShoot] = null;
            sleep(120);
        }
    }

    protected boolean allSlotsLoaded() {
        return slots[0] != null && slots[1] != null && slots[2] != null;
    }

    protected int findSlotForColor(char wanted) {
        String w = String.valueOf(wanted);
        for (int i = 0; i < 3; i++) {
            if (!slotFired[i] && w.equals(slots[i])) return i;
        }
        return -1;
    }

    protected int intakeSlotToShootSlot(int intakeSlot) {
        return (intakeSlot + INTAKE_TO_SHOOT_OFFSET + 3) % 3;
    }
    protected String intakePattern() {
        return (mag0State ? "1" : "0") + (mag1State ? "1" : "0");
    }

    protected boolean isIntakeSlotPattern() {
        // Any pattern with at least one magnet detected (00,01,10) is "slot-ish"
        return (!mag0State) || (!mag1State);   // 00,01,10
    }
    // Robust intake slot classification using timing window (handles 01->11->10 during slot0)
    protected int classifyIntakeSlotRobust() {
        long now = System.currentTimeMillis();
        boolean topRecent    = (now - intakeTopSeenTime) < MAGNET_WINDOW_MS;
        boolean bottomRecent = (now - intakeBottomSeenTime) < MAGNET_WINDOW_MS;

        if (topRecent && bottomRecent) return 0; // slot0 (both seen close together)
        if (topRecent) return 1;
        if (bottomRecent) return 2;
        return -1;
    }


    protected int classifyIntakeSlotImmediate() {
        // This MUST match your physical mapping:
        // Slot0 = BOTH magnets -> 00
        // Slot1 = TOP only     -> 01? or 10? (depends on which magnet is "top")
        // Slot2 = BOTTOM only  -> 10? or 01?
        //
        // Based on your current atSlot*() mapping:
        //  - Slot0: !mag0 && !mag1  => "00"
        //  - Slot1: !mag0 &&  mag1  => "01"
        //  - Slot2:  mag0 && !mag1  => "10"

        if (!mag0State && !mag1State) return 0; // 00 both
        if (!mag0State &&  mag1State) return 1; // 01 top only
        if ( mag0State && !mag1State) return 2; // 10 bottom only
        return -1;                               // 11 between/no magnet
    }
    protected int spinToNextIntakeSlotAndStop(double power) {
        shootingBusy = true;
        try {
            ensureKickerRetracted();

            // IMPORTANT: clear old sightings so we don't "start" with stale times
            resetMagnetTiming();
            updateMagnetStates();

            final long TIMEOUT_MS = 6000;
            final long STABLE_MS  = 120;
            long deadline = System.currentTimeMillis() + TIMEOUT_MS;

            int startSlot = classifyIntakeSlotRobust();

            // Start spinning
            servo1.setPower(power);

            int found = -1;
            while (opModeIsActive() && System.currentTimeMillis() < deadline) {
                updateMagnetStates();

                int s = classifyIntakeSlotRobust();

                // Candidate = valid slot and different than where we started
                boolean candidate = (s != -1) && (startSlot == -1 || s != startSlot);

                if (candidate) {
                    long holdStart = System.currentTimeMillis();
                    boolean ok = true;

                    while (opModeIsActive() && System.currentTimeMillis() < deadline) {
                        updateMagnetStates();
                        if (classifyIntakeSlotRobust() != s) { ok = false; break; }
                        if (System.currentTimeMillis() - holdStart >= STABLE_MS) break;
                    }

                    if (ok && classifyIntakeSlotRobust() == s) {
                        found = s;
                        break;
                    }
                }

                telemetryUpdateThrottled();
            }

            servo1.setPower(0);

            if (found == -1) {
                setShootRotateStatus("INTAKE NEXT FAIL start=" + startSlot + " patt=" + intakePattern());
            } else {
                setShootRotateStatus("INTAKE NEXT -> " + found + " start=" + startSlot + " patt=" + intakePattern());
            }

            return found;

        } finally {
            servo1.setPower(0);
            shootingBusy = false;
        }
    }




    // Returns a readable pattern like "10" (mag2=1 mag3=0)
    protected String shootPattern() {
        return (mag2State ? "1" : "0") + (mag3State ? "1" : "0");
    }
    protected void debugStopOnShootMagSightings(double crawlPower, long maxRunMs) {
        // Start from current cached state
        updateMagnetStates();
        boolean last2 = mag2State;
        boolean last3 = mag3State;

        long start = System.currentTimeMillis();
        long deadline = start + maxRunMs;

        setShootRotateStatus("DEBUG: running...");

        // Slow crawl so we don't blow past patterns
        servo1.setPower(crawlPower);

        while (opModeIsActive() && System.currentTimeMillis() < deadline) {
            updateMagnetStates();

            boolean changed = (mag2State != last2) || (mag3State != last3);

            if (changed) {
                // STOP immediately on any change
                servo1.setPower(0);

                String patt = shootPattern();
                String cls = classifyShootPattern();

                // Persist status so it stays visible even if your TeleOp clears telemetry
                setShootRotateStatus("SIGHTING patt=" + patt + " " + cls);

                // Show a dedicated telemetry page
                telemetry.clearAll();
                telemetry.addLine("=== SHOOT MAG SIGHTING ===");
                telemetry.addData("mag2/mag3", patt);
                telemetry.addData("classified", cls);
                telemetry.addData("raw", "mag2=%s mag3=%s", mag2State ? "1" : "0", mag3State ? "1" : "0");
                telemetry.addLine("Press RIGHT_BUMPER to continue, or B to exit.");
                telemetry.update();

                // Wait for user to continue or exit
                while (opModeIsActive()) {
                    // Exit
                    if (gamepad2.b) {
                        setShootRotateStatus("DEBUG: exit by B");
                        return;
                    }

                    // Continue
                    if (gamepad2.right_bumper) {
                        // Debounce: wait for release
                        while (opModeIsActive() && gamepad2.right_bumper) {
                            telemetryUpdateThrottled();
                        }
                        break;
                    }

                    telemetryUpdateThrottled();
                }

                // Resume crawl
                updateMagnetStates();
                last2 = mag2State;
                last3 = mag3State;
                servo1.setPower(crawlPower);
            } else {
                // Update last states occasionally even if no change (optional)
                last2 = mag2State;
                last3 = mag3State;
            }

            telemetryUpdateThrottled();
        }

        servo1.setPower(0);
        setShootRotateStatus("DEBUG: timeout");
    }
    // Raw classification from immediate pattern (NO timing)
    protected String classifyShootPattern() {
        String p = shootPattern();
        switch (p) {
            case "10": return "Slot0? (mag2=1 mag3=0)";
            case "01": return "Slot1? (mag2=0 mag3=1)";
            case "00": return "Slot2? (mag2=0 mag3=0)";
            case "11": return "Between/No magnet (11)";
            default:   return "Unknown";
        }
    }

    // ==================== MOTIF LISTENER ====================
    protected void updateLatchedMotif(LLResult result) {
        if (motifLatched) return;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;

        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult t : tags) {
            int id = t.getFiducialId();
            if (id >= 21 && id <= 23) {
                if (best == null || t.getTargetArea() > best.getTargetArea()) {
                    best = t;
                }
            }
        }

        if (best == null) return;

        int id = best.getFiducialId();
        String motif = decodeMotifFromTagId(id);

        if (!"UNKNOWN".equals(motif)) {
            motifLatched = true;
            latchedTagId = id;
            latchedMotif = motif;

            // Update distance
            Pose3D pose = best.getTargetPoseCameraSpace();
            if (pose != null) {
                double xIn = pose.getPosition().x * 39.37;
                double yIn = pose.getPosition().y * 39.37;
                double zIn = pose.getPosition().z * 39.37;
                lastTagDistanceIn = Math.sqrt(xIn*xIn + yIn*yIn + zIn*zIn);
            }
        }
    }

    protected String decodeMotifFromTagId(int tagId) {
        switch (tagId) {
            case 21: return "gpp";
            case 22: return "pgp";
            case 23: return "ppg";
            default: return "UNKNOWN";
        }
    }
    // Field-centric wrapper around driveRobotCentric()
    // y = forward/back on stick, x = strafe on stick, rx = rotate stick
    protected void driveFieldCentric(double y, double x, double rx) {
        // Get robot heading (degrees). If your yaw is reversed, flip the sign.
        double headingDeg = getYawDeg(); // imu1 yaw
        double headingRad = Math.toRadians(headingDeg);

        // Rotate the joystick vector by -heading to convert field->robot
        double cosA = Math.cos(-headingRad);
        double sinA = Math.sin(-headingRad);

        double yRobot = y * cosA - x * sinA;
        double xRobot = y * sinA + x * cosA;

        driveRobotCentric(yRobot, xRobot, rx);
    }


    // ==================== DRIVE ====================
    protected void driveRobotCentric(double y, double x, double rx) {
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        motor2.setPower(fl / max);
        motor0.setPower(fr / max);
        motor1.setPower(bl / max);
        motor3.setPower(br / max);
    }

    protected void stopDrive() {
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }

    // ==================== IMU UTILITIES ====================
    protected double getYawDeg() {
        return imu1.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;

    }
    protected double getImu2YawDeg() {
        if (imu2 == null) return 0.0;
        YawPitchRollAngles ypr = imu2.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.DEGREES);
    }

    protected static final double ARM_SIGN = +1.0;

    protected double imu2PitchZeroDeg = 0.0;
    protected double armAngleDeg = 0.0;

    protected double getImu2PitchDeg() {
        if (imu2 == null) return 0.0;
        YawPitchRollAngles ypr = imu2.getRobotYawPitchRollAngles();
        return ypr.getPitch(AngleUnit.DEGREES);
    }

    protected void zeroArmAngle() {
        imu2PitchZeroDeg = getImu2PitchDeg();
    }

    protected void updateArmAngle() {
        armAngleDeg = ARM_SIGN * (getImu2PitchDeg() - imu2PitchZeroDeg);
    }

    protected double getArmAngleDeg() {
        return armAngleDeg;
    }




    protected static double angleWrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
    protected boolean isShootSlotPattern() {
        // Any pattern that includes at least one magnet detected
        // (magXState=false means magnet detected)
        return (!mag2State) || (!mag3State);   // patterns 10,01,00 are "slot-ish"
    }

    protected int classifyShootSlotImmediate() {
        // return 0/1/2 for your current mapping, or -1 for between/no magnet
        if (mag2State && !mag3State) return 0;        // 10
        if (!mag2State && mag3State) return 1;        // 01
        if (!mag2State && !mag3State) return 2;       // 00
        return -1;                                    // 11
    }
    protected int spinToNextShootSlotAndStop(double power) {
        shootingBusy = true;
        try {
            ensureKickerRetracted();
            updateMagnetStates();

            final long TIMEOUT_MS = 2500;
            final long STABLE_MS = 40;     // small stability window helps a lot
            long deadline = System.currentTimeMillis() + TIMEOUT_MS;

            // Step 1: if we’re currently ON a slot, leave it first
            if (isShootSlotPattern()) {
                servo1.setPower(power);
                long leaveDeadline = System.currentTimeMillis() + 250;
                while (opModeIsActive() && System.currentTimeMillis() < leaveDeadline) {
                    updateMagnetStates();
                    if (!isShootSlotPattern()) break; // now in "11"
                }
            }

            // Step 2: now spin until we ENTER a new slot pattern and it stays stable briefly
            servo1.setPower(power);

            int found = -1;
            while (opModeIsActive() && System.currentTimeMillis() < deadline) {
                updateMagnetStates();

                int s = classifyShootSlotImmediate();
                if (s != -1) {
                    // Hold check for stability
                    long holdStart = System.currentTimeMillis();
                    boolean ok = true;

                    while (opModeIsActive() && System.currentTimeMillis() < deadline) {
                        updateMagnetStates();
                        if (classifyShootSlotImmediate() != s) { ok = false; break; }
                        if (System.currentTimeMillis() - holdStart >= STABLE_MS) break;
                    }

                    if (ok && classifyShootSlotImmediate() == s) {
                        found = s;
                        break;
                    }
                }
            }

            servo1.setPower(0);

            // Optional brake OFF for now; it can bounce you out
            // servo1.setPower(MGR_BRAKE_POWER); sleep(MGR_BRAKE_MS); servo1.setPower(0);

            // Record status for telemetry
            if (found == -1) {
                setShootRotateStatus("NEXT SLOT FAIL (timeout) last=" + (mag2State ? "1":"0") + (mag3State ? "1":"0"));
            } else {
                setShootRotateStatus("NEXT SLOT -> " + found + " patt=" + (mag2State ? "1":"0") + (mag3State ? "1":"0"));
            }

            return found;

        } finally {
            servo1.setPower(0);
            shootingBusy = false;
        }
    }
    // ==================== TELEMETRY ====================
    protected void telemetryUpdateThrottled() {
        long now = System.currentTimeMillis();
        if (now - lastTelemMs >= TELEM_PERIOD_MS) {
            telemetry.update();
            lastTelemMs = now;
        }
    }

    // ==================== UTILITY ====================
    protected double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    protected static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    protected void setShootRotateStatus(String s) {
        lastShootRotateStatus = s;
        lastShootRotateStatusMs = System.currentTimeMillis();
    }
    protected boolean isAtShootSlotStable(int targetSlot, long stableMs) {
        long startHold = 0;
        long now;

        while (opModeIsActive()) {
            updateMagnetStates();
            now = System.currentTimeMillis();

            boolean at = atShootSlot(targetSlot);

            if (at) {
                if (startHold == 0) startHold = now;
                if (now - startHold >= stableMs) return true;
            } else {
                startHold = 0; // reset hold timer if pattern breaks
            }

            // allow other loops to run
            telemetryUpdateThrottled();
        }
        return false;
    }
    // Returns distance (inches) from camera to the "best" visible tag,
    // or -1 if none.
    protected double getBestTagDistanceInches(LLResult result) {
        if (result == null || !result.isValid()) return -1;

        Pose3D targetCam = getBestTagPoseCameraSpace(result);
        if (targetCam == null) return -1;

        double xIn = targetCam.getPosition().x * 39.37;
        double yIn = targetCam.getPosition().y * 39.37;
        double zIn = targetCam.getPosition().z * 39.37;

        return Math.sqrt(xIn * xIn + yIn * yIn + zIn * zIn);
    }
    // Picks the biggest-area fiducial and returns its camera-space pose.
    protected Pose3D getBestTagPoseCameraSpace(LLResult result) {
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;

        LLResultTypes.FiducialResult best = tags.get(0);
        for (LLResultTypes.FiducialResult t : tags) {
            if (t.getTargetArea() > best.getTargetArea()) best = t;
        }
        return best.getTargetPoseCameraSpace();
    }
    // Turn robot using IMU2 yaw to the next "step" heading (e.g., +45° each press)
    protected void stepTurnIMU2(double stepDeg, double maxPower) {
        final double TOL_DEG = 2.0;
        final double KP = 0.015;
        final double MIN_OUT = 0.10;
        final long TIMEOUT_MS = 2500;

        double startYaw = getImu2YawDeg();
        double targetYaw = angleWrapDeg(startYaw + stepDeg);

        long startMs = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startMs) < TIMEOUT_MS) {
            double yaw = getImu2YawDeg();
            double err = angleWrapDeg(targetYaw - yaw);

            if (Math.abs(err) <= TOL_DEG) break;

            double out = KP * err;
            // add minimum to overcome stiction
            if (out > 0) out = Math.max(out, MIN_OUT);
            else         out = Math.min(out, -MIN_OUT);

            out = clip(out, -Math.abs(maxPower), Math.abs(maxPower));

            // robot-centric rotate only
            driveRobotCentric(0, 0, out);

            telemetry.addData("IMU2 Turn", "start=%.1f target=%.1f yaw=%.1f err=%.1f out=%.2f",
                    startYaw, targetYaw, yaw, err, out);
            telemetryUpdateThrottled();
        }

        stopDrive();
    }
    protected int spinToNextIntakeSlotAndStopFastCapture(double fastPower, double crawlPower) {
        shootingBusy = true;
        try {
            ensureKickerRetracted();

            // Clear old sightings so the "robust" window is fresh
            resetMagnetTiming();
            updateMagnetStates();

            final long TIMEOUT_MS = 6000;
            final long STABLE_MS  = 70;   // stable confirm time once crawling
            final long CRAWL_MAX_MS = 1200;

            long deadline = System.currentTimeMillis() + TIMEOUT_MS;

            // Where are we now? (robust so we don't skip slot0)
            int startSlot = classifyIntakeSlotRobust();

            // 1) FAST: leave current slot region first if we're "on a slot"
            servo1.setPower(fastPower);
            long leaveDeadline = System.currentTimeMillis() + 350;
            while (opModeIsActive() && System.currentTimeMillis() < leaveDeadline) {
                updateMagnetStates();
                int s = classifyIntakeSlotRobust();
                if (s == -1) break; // now in between
            }

            // Refresh timing after leaving so the next slot is clean
            resetMagnetTiming();

            // 2) FAST SEARCH until we see *any* magnet hint
            servo1.setPower(fastPower);
            while (opModeIsActive() && System.currentTimeMillis() < deadline) {
                updateMagnetStates();

                // any hint we’re entering a slot
                if (isIntakeSlotPattern() || classifyIntakeSlotRobust() != -1) {
                    break;
                }
                telemetryUpdateThrottled();
            }

            // 3) CRAWL + STABLE HOLD to land precisely
            servo1.setPower(crawlPower);
            long crawlDeadline = Math.min(deadline, System.currentTimeMillis() + CRAWL_MAX_MS);

            int found = -1;
            while (opModeIsActive() && System.currentTimeMillis() < crawlDeadline) {
                updateMagnetStates();

                int s = classifyIntakeSlotRobust();
                boolean candidate = (s != -1) && (startSlot == -1 || s != startSlot);

                if (candidate) {
                    long holdStart = System.currentTimeMillis();
                    boolean ok = true;

                    while (opModeIsActive() && System.currentTimeMillis() < crawlDeadline) {
                        updateMagnetStates();
                        if (classifyIntakeSlotRobust() != s) { ok = false; break; }
                        if (System.currentTimeMillis() - holdStart >= STABLE_MS) break;
                    }

                    if (ok && classifyIntakeSlotRobust() == s) {
                        found = s;
                        break;
                    }
                }

                telemetryUpdateThrottled();
            }

            servo1.setPower(0);

            if (found == -1) {
                setShootRotateStatus("INTAKE FASTCAP FAIL start=" + startSlot + " patt=" + intakePattern());
            } else {
                setShootRotateStatus("INTAKE FASTCAP -> " + found + " patt=" + intakePattern());
            }

            return found;

        } finally {
            servo1.setPower(0);
            shootingBusy = false;
        }
    }
    protected int spinToNextShootSlotAndStopFastOnly(double fastPower) {
        shootingBusy = true;
        try {
            ensureKickerRetracted();
            updateMagnetStates();

            final long TIMEOUT_MS = 3500;
            final long STABLE_MS  = 70;   // confirm we’re really on the slot
            long deadline = System.currentTimeMillis() + TIMEOUT_MS;

            int startSlot = classifyShootSlotImmediate();

            // Must see BETWEEN (11) at least once before we accept the next slot
            boolean seenBetween = !isShootSlotPattern(); // if we start in 11, we’re good already

            // 1) Leave current slot region until we see 11 (between)
            if (!seenBetween) {
                servo1.setPower(fastPower);
                long leaveDeadline = Math.min(deadline, System.currentTimeMillis() + 700);
                while (opModeIsActive() && System.currentTimeMillis() < leaveDeadline) {
                    updateMagnetStates();
                    if (!isShootSlotPattern()) { // 11
                        seenBetween = true;
                        break;
                    }
                    telemetryUpdateThrottled();
                }
            }

            // 2) Keep spinning FAST until we hit a DIFFERENT slot, and hold it stable
            servo1.setPower(fastPower);

            int found = -1;
            while (opModeIsActive() && System.currentTimeMillis() < deadline) {
                updateMagnetStates();

                int s = classifyShootSlotImmediate(); // 0/1/2 or -1 for 11
                boolean candidate = seenBetween && (s != -1) && (startSlot == -1 || s != startSlot);

                if (candidate) {
                    long holdStart = System.currentTimeMillis();
                    boolean ok = true;

                    while (opModeIsActive() && System.currentTimeMillis() < deadline) {
                        updateMagnetStates();
                        if (classifyShootSlotImmediate() != s) { ok = false; break; }
                        if (System.currentTimeMillis() - holdStart >= STABLE_MS) break;
                        telemetryUpdateThrottled();
                    }

                    if (ok && classifyShootSlotImmediate() == s) {
                        found = s;
                        break;
                    }
                }

                telemetryUpdateThrottled();
            }

            servo1.setPower(0);

            if (found == -1) {
                setShootRotateStatus("SHOOT FASTONLY FAIL start=" + startSlot + " patt=" + shootPattern());
            } else {
                setShootRotateStatus("SHOOT FASTONLY -> " + found + " patt=" + shootPattern());
            }

            return found;

        } finally {
            servo1.setPower(0);
            shootingBusy = false;
        }
    }




}