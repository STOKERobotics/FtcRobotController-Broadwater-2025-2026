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
    protected static final long MAGNET_WINDOW_MS = 150; // Detection window for timing tolerance

    // ==================== CONSTANTS ====================
    // Alignment
    protected static final double ALIGN_KP = 0.02;
    protected static final double ALIGN_TOLERANCE = 1.0;
    protected static final double ALIGN_MAX_POWER = 0.25;
    protected static final long ALIGN_STABLE_MS = 200;

    // Shooter
    protected static final double SHOOTER_MIN_DIST = 50.0;
    protected static final double SHOOTER_MAX_DIST = 120.0;
    protected static final double SERVO_MIN_ANGLE = 0.25;
    protected static final double SERVO_MAX_ANGLE = 0.85;
    protected static final double KICK_EXTEND_POS = 0.0;
    protected static final double KICK_RETRACT_POS = 1.0;
    protected static final long KICK_DURATION_MS = 1000;
    protected static final long KICK_RETRACT_WAIT_MS = 500;

    // Merry-go-round servo
    protected static final double MGR_FAST_POWER = 1;
    protected static final double MGR_CRAWL_POWER = 0.08;
    protected static final long MGR_BRAKE_MS = 100;
    protected static final double MGR_BRAKE_POWER = -0.60;
    protected static final long MGR_MOVE_TIMEOUT_MS = 10000;
    protected static final int INTAKE_SLOT_STEP = 1;
    protected static final int SHOOT_SLOT_STEP = 1;
    protected static final int INTAKE_TO_SHOOT_OFFSET = 0;

    // Servo2 (shooter angle)
    protected static final double SERVO2_MIN = 0.0;
    protected static final double SERVO2_MAX = 1.0;
    protected static final double SERVO2_RATE = 0.1;

    // Color sensor thresholds
    protected static final float COLOR_SAT_THRESHOLD = 0.05f;
    protected static final float COLOR_VAL_THRESHOLD = 0.009f;
    protected static final float HUE_GREEN_MIN = 120f;
    protected static final float HUE_GREEN_MAX = 180f;
    protected static final float HUE_PURPLE_MIN = 210f;
    protected static final float HUE_PURPLE_MAX = 255f;

    // Telemetry
    protected static final long TELEM_PERIOD_MS = 100;

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
//        imu2 = hardwareMap.get(BNO055IMU.class, "imu 2");
//        initIMU(imu2, false);
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

    // Shooter slot detectors (use cached states with timing window for robustness)
    // Shoot Slot 0 = BOTTOM magnet only (divider between slot 0 and 1)
    protected boolean atShootSlot0() {
        // Immediate detection: bottom detected, top not detected
        if (mag2State && !mag3State) return true;

        // Robust detection: bottom seen recently, top NOT seen recently
        // Note: Do NOT use mag2State here - it's true when between slots (no magnets)
        long now = System.currentTimeMillis();
        boolean bottomRecent = (now - shootBottomSeenTime) < MAGNET_WINDOW_MS;
        boolean topNotRecent = (shootTopSeenTime == 0) || (now - shootTopSeenTime) >= MAGNET_WINDOW_MS;
        return bottomRecent && topNotRecent;
    }

    // Shoot Slot 1 = TOP magnet only (mag2 detected, mag3 not)
    protected boolean atShootSlot1() {
        // Immediate detection: top detected, bottom not detected
        if (!mag2State && mag3State) return true;

        // Robust detection: top seen recently, bottom NOT seen recently
        // Note: Do NOT use mag3State here - it's true when between slots (no magnets)
        long now = System.currentTimeMillis();
        boolean topRecent = (now - shootTopSeenTime) < MAGNET_WINDOW_MS;
        boolean bottomNotRecent = (shootBottomSeenTime == 0) || (now - shootBottomSeenTime) >= MAGNET_WINDOW_MS;
        return topRecent && bottomNotRecent;
    }

    // Shoot Slot 2 = BOTH magnets (both detected)
    protected boolean atShootSlot2() {
        // Immediate detection: both magnets currently detected
        if (!mag2State && !mag3State) return true;

        // Robust detection: both magnets seen within timing window
        long now = System.currentTimeMillis();
        boolean topRecent = (now - shootTopSeenTime) < MAGNET_WINDOW_MS;
        boolean bottomRecent = (now - shootBottomSeenTime) < MAGNET_WINDOW_MS;
        return topRecent && bottomRecent;
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
        long kickStart = System.currentTimeMillis();

        // Phase 1: Extend
        telemetry.clear();
        telemetry.addData("KICKER", "EXTENDING...");
        telemetry.addData("Phase", "1/3 - Extend");
        telemetry.update();

        servo0.setPosition(KICK_EXTEND_POS);
        sleep(KICK_DURATION_MS);
        long afterExtend = System.currentTimeMillis();
        lastKickExtendMs = afterExtend - kickStart;

        // Phase 2: Retract command
        telemetry.clear();
        telemetry.addData("KICKER", "RETRACTING...");
        telemetry.addData("Phase", "2/3 - Retract");
        telemetry.addData("Extend took", "%d ms", lastKickExtendMs);
        telemetry.update();

        servo0.setPosition(KICK_RETRACT_POS);

        // Phase 3: Wait for retract to complete (using KICK_RETRACT_WAIT_MS)
        sleep(KICK_RETRACT_WAIT_MS);
        long afterRetract = System.currentTimeMillis();
        lastKickRetractMs = afterRetract - afterExtend;
        lastKickTotalMs = afterRetract - kickStart;

        // Show final timing summary briefly
        telemetry.clear();
        telemetry.addData("KICKER", "COMPLETE");
        telemetry.addData("Extend", "%d ms", lastKickExtendMs);
        telemetry.addData("Retract", "%d ms", lastKickRetractMs);
        telemetry.addData("TOTAL", "%d ms", lastKickTotalMs);
        telemetry.update();
    }

    protected void ensureKickerRetracted() {
        servo0.setPosition(KICK_RETRACT_POS);
        sleep(KICK_RETRACT_WAIT_MS);
    }

    protected void rotateToSlotBlocking(int targetSlot, boolean useShootMagnets) {
        shootingBusy = true;
        try {
            ensureKickerRetracted();

            // Reset magnet timing to avoid false positives from previous detections
            resetMagnetTiming();

            // Check if already at target and leave if so
            updateMagnetStates(); // Fresh read after reset
            boolean atTarget = useShootMagnets ? atShootSlot(targetSlot) : atIntakeSlot(targetSlot);
            if (atTarget) {
                long leaveStart = System.currentTimeMillis();
                servo1.setPower(MGR_CRAWL_POWER);
                while (opModeIsActive() && (System.currentTimeMillis() - leaveStart) < 400) {
                    updateMagnetStates();
                    atTarget = useShootMagnets ? atShootSlot(targetSlot) : atIntakeSlot(targetSlot);
                    if (!atTarget) break;
                }
                servo1.setPower(0);
                // Reset timing again after leaving the current position
                resetMagnetTiming();
            }

            // Rotate to target
            long start = System.currentTimeMillis();
            servo1.setPower(MGR_FAST_POWER);

            while (opModeIsActive() && (System.currentTimeMillis() - start) < MGR_MOVE_TIMEOUT_MS) {
                updateMagnetStates();
                atTarget = useShootMagnets ? atShootSlot(targetSlot) : atIntakeSlot(targetSlot);
                if (atTarget) {
                    servo1.setPower(0); // INSTANT STOP
                    break;
                }
            }

            servo1.setPower(0);

            // Brake
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

    protected static double angleWrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
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
}