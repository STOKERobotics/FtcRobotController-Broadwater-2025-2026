package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.AnalogInput;

import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous(name = "Broadwater Auto: 1m + Turn35 + SpinUp")
public class broadwater_robotics_25_26_auto extends LinearOpMode {

    private DcMotor motor0; // Drive FR
    private DcMotor motor1; // Drive BL
    private DcMotor motor2; // Drive FL
    private DcMotor motor3; // Drive BR
    private DcMotor motor0b; // Shooter 1
    private DcMotor motor1b; // Shooter 2
    private DcMotor motor2b; // Intake
    private Servo servo0; // Kicker
    private CRServo servo1; // Merry Go Round Tray
    private Servo servo2; // Shooter Angle
    private BNO055IMU imu1;
    private DigitalChannel blueLED;
    private DigitalChannel redLED;
    private AnalogInput laser;
    private DigitalChannel mag0; // Intake Magnet 0
    private DigitalChannel mag1; // Intake Magnet 1
    private DigitalChannel mag2; // Shooter Magnet 2
    private DigitalChannel mag3; // Shooter Magnet 3



    // Limelight alignment control
    // --- Alignment tuning ---
    private static final double ALIGN_KP = 0.02;            // start smaller than before
    private static final double ALIGN_TOLERANCE = 1.0;      // deg
    private static final double ALIGN_MAX_POWER = 0.25;
    private static final double ALIGN_MIN_POWER = 0.08;     // helps overcome friction
    private static final long   ALIGN_STABLE_MS = 200;      // must stay within tolerance this long

    // --- Emergency stop (drive) ---
    private boolean driveEStop = false;
    private boolean wasEStopCombo = false;
    private boolean wasClearEStop = false;

    private boolean alignStableStarted = false;
    private long alignStableStartMs = 0;

    // Optional smoothing (prevents jitter)
    private double txFiltered = 0.0;
    private static final double TX_ALPHA = 0.35; // 0..1 (higher = less smoothing)
    private boolean alignActive = false;

    // Shooter control constants
    private static final double SHOOTER_MIN_DIST   = 50.0;  // inches
    private static final double SHOOTER_MAX_DIST   = 120.0;  // inches

    // Servo angle limits
    private static final double SERVO_MIN_ANGLE = 0.25;
    private static final double SERVO_MAX_ANGLE = 0.85;

    // Firing servo timing
    private static final double KICK_EXTEND_POS = 0.0;
    private static final double KICK_RETRACT_POS = 1.0;
    private static final long   KICK_DURATION_MS = 1000;  // how long to stay extended
    private boolean shootingBusy = false;


    private NormalizedColorSensor ballColor;

    private String ballColorValue;
    private double servo2Pos = 0.5;                 // start midpoint (change if you want)
    private static final double SERVO2_MIN = 0.0;
    private static final double SERVO2_MAX = 1.0;
    private static final double SERVO2_RATE = 0.6;

    private double lastServo2Time = 0.0;

    private boolean motifLatched = false;
    private int latchedTagId = -1;
    private String latchedMotif = "NONE";
    private double lastTagDistanceIn = -1;  // -1 means unknown
    // --- Limelight motif listener toggle (gamepad2 triangle) ---
    private boolean motifListenEnabled = false;
    private boolean wasMotifListenTogglePressed = false;
    private static final long SLOT_STABLE_MS = 40;  // try 60-120ms
    private static final long LEAVE_STABLE_MS = 20; // shorter is fine



    // Merry-go-round state machine
    private enum INTAKEState { INIT_TO_SLOT0, WAIT_COLOR_0, MOVE_TO_SLOT1, WAIT_COLOR_1, MOVE_TO_SLOT2, WAIT_COLOR_2, DONE }
    private INTAKEState intakeState = INTAKEState.INIT_TO_SLOT0;

    private int currentSlot = 0;
    private boolean colorLatched = false; // edge-detect so one ball = one store
    private boolean isAlignTag(int id) {
        return (id == 20 || id == 24);
    }

    // Marks which physical slot has already been fired
    private final boolean[] slotFired = new boolean[3];

    // Adjust if servo direction is backwards
    private static final double MGR_FAST_POWER   = 0.25;   // your main spin
    private static final double MGR_CRAWL_POWER  = 0.1;  // slow approach
    private static final long   MGR_BRAKE_MS     = 0;    // short reverse tap
    private static final double MGR_BRAKE_POWER  = -0.30; // brake tap power
    private static final double MGR_BACKUP_POWER   = -0;  // stronger than -0.10
    private static final long   MGR_BACKUP_1DEG_MS = 0;

    private boolean wasKickTrigger = false;
    private static final float KICK_TRIGGER_THRESHOLD = 0.6f; // adjust if you want
    private boolean wasForceSkipTrigger = false;
    private boolean wasShootAdvancePressed = false;
    private static final int INTAKE_TO_SHOOT_OFFSET = 0; // try 1, if reversed use 2

    // which direction is "next" for intake? try +1 first; if wrong use -1
    private static final int INTAKE_SLOT_STEP = +1;
    // Step-shoot-by-color state (one shot per button press)
    private int stepShotIndex = 0;     // 0..2 (which motif char we are on)
    private boolean stepModeActive = false;



    // Safety so you don’t spin forever if a magnet fails
    private static final long MGR_MOVE_TIMEOUT_MS = 10000;

    float RSX;
    float LSY;
    double correctedStrafePower;
    double strafePower;
    float LSX;
    double correctedDrivePower;
    double drivePower;
    double rotatePower;
    private boolean isRedOn = false;
    private boolean isBlueOn = false;
    private boolean wasButtonAPressed = false;
    private boolean wasButtonBPressed = false;
    private Limelight3A limelight;
    String[] slots = new String[3];
    private boolean shoot = false;
    private boolean align = false;
    private static final double CAMERA_X_OFFSET_IN = 5.0;   // camera is 5 inches LEFT
    private static final double IN_TO_M = 0.0254;
    @Override
    public void runOpMode() {
        initLimelight();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");

        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor0b = hardwareMap.get(DcMotor.class, "motor0b");
        motor1b = hardwareMap.get(DcMotor.class, "motor1b");
        motor2b = hardwareMap.get(DcMotor.class, "motor2b");
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        laser = hardwareMap.get(AnalogInput.class, "laser");
        servo2.setPosition(servo2Pos);
        lastServo2Time = getRuntime();
        imu1 = hardwareMap.get(BNO055IMU.class, "imu 1");
        ballColor = hardwareMap.get(NormalizedColorSensor.class, "ballColor"); // match config name

        mag0 = hardwareMap.get(DigitalChannel.class, "mag0");
        mag1 = hardwareMap.get(DigitalChannel.class, "mag1");
        mag2 = hardwareMap.get(DigitalChannel.class, "mag2");
        mag3 = hardwareMap.get(DigitalChannel.class, "mag3");

        redLED  = hardwareMap.get(DigitalChannel.class, "redLED");
        blueLED = hardwareMap.get(DigitalChannel.class, "blueLED");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        blueLED.setMode(DigitalChannel.Mode.OUTPUT);

        // Put initialization blocks here.
        motor0.setDirection(DcMotor.Direction.FORWARD);
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor0b.setDirection(DcMotor.Direction.REVERSE);
        motor0b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1b.setDirection(DcMotor.Direction.FORWARD);
        motor1b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor2b.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo0.setPosition(KICK_RETRACT_POS);
        servo1.setDirection(DcMotorSimple.Direction.REVERSE);

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        Parameters imuParameters = new Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu1.initialize(imuParameters);


        if (ballColor instanceof SwitchableLight) {
            ((SwitchableLight)ballColor).enableLight(true);
        }

        mag0.setMode(DigitalChannel.Mode.INPUT);
        mag1.setMode(DigitalChannel.Mode.INPUT);
        mag2.setMode(DigitalChannel.Mode.INPUT);
        mag3.setMode(DigitalChannel.Mode.INPUT);



        waitForStart();
        if (!opModeIsActive()) return;
        motor0b.setPower(.7);
        motor1b.setPower(.7);
        motifListenEnabled = true;     // auto must enable it
        motifLatched = false;
        driveForwardMeters(0.1, -0.6);
//        telemetryLimeLight();
        motifLatched = false;
        latchedMotif = "NONE";
        latchedTagId = -1;
        telemetry.update();

        while(opModeIsActive() && !motifLatched) {
            updateMotifListener();
            telemetry.addData("Motif Listener", motifListenEnabled ? "ON" : "OFF");
            telemetry.addData("Latched Motif", "%s (Tag %d)", latchedMotif, latchedTagId);
            telemetryUpdateThrottled();
        }
        while (opModeIsActive() && intakeState != INTAKEState.DONE) {
            telemetryUpdateThrottled();
            updateBallColor();
            merryGoRoundIntake();
            telemetryBallColor();
        }
        // ---- Drive backward 0.5 meter ----
        //driveForwardMeters(0.5, -0.4);
        telemetry.update();// ---- Turn right 35 degrees ----
        turnRightDegrees(15.0, 0.25);
        telemetry.update();



        while (!align){
           align = alignToTarget();
        }
        while (opModeIsActive() && !shoot){
            adjustShooterAndFire();
        }
    }

    // ---------------- Init ----------------


    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    private long lastTelemMs = 0;
    private static final long TELEM_PERIOD_MS = 100; // 10Hz

    private void adjustShooterAndFire() {
        if (!motifLatched || latchedMotif == null || latchedMotif.length() != 3 || "UNKNOWN".equals(latchedMotif)) {
            telemetry.addLine("Shooter: No valid motif latched");
            return;
        }
        if (!allSlotsLoaded()) {
            telemetry.addLine("Shooter: Slots not all loaded yet");
            telemetry.addData("Slots", "0=%s 1=%s 2=%s", slots[0], slots[1], slots[2]);
            return;
        }
        if (lastTagDistanceIn <= 0) {
            telemetry.addLine("Shooter: No tag distance");
            return;
        }

        double distanceInches = lastTagDistanceIn;
        double normalized = Math.max(0, Math.min(1,
                (distanceInches - SHOOTER_MIN_DIST) / (SHOOTER_MAX_DIST - SHOOTER_MIN_DIST)));

        double servoAngle = SERVO_MAX_ANGLE - (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) * normalized;
        servo2.setPosition(.65);

        telemetry.addData("Motif", latchedMotif);
        telemetry.addData("Slots", "0=%s 1=%s 2=%s", slots[0], slots[1], slots[2]);
        telemetry.addData("Dist(in)", "%.1f", distanceInches);
        telemetry.addData("Kicker Angle", "%.2f", servoAngle);

        for (int i = 0; i < 3; i++) slotFired[i] = false;

        char[] order = latchedMotif.toCharArray();

        for (int shotIndex = 0; shotIndex < 3 && opModeIsActive(); shotIndex++) {
            char wanted = order[shotIndex];

            int slotToShoot = findSlotForColor(wanted);
            if (slotToShoot < 0) {
                telemetry.addData("Shooter", "Missing color '%c' in slots (or already used)", wanted);
                return;
            }

            telemetry.addData("Shooting", "shot %d wants %c -> slot %d", shotIndex, wanted, slotToShoot);

            int shootFrameSlot = intakeSlotToShootSlot(slotToShoot);
            rotateToSlotBlocking(shootFrameSlot);
            kickOnce();

            // Mark fired + clear stored color so it can be reloaded
            slotFired[slotToShoot] = true;
            slots[slotToShoot] = null;

            sleep(120);
        }
        shoot = true;
        telemetry.addLine("Shoot sequence done!");
        returnTrayToIntakeSlot0();
    }

    private boolean allSlotsLoaded() {
        return slots[0] != null && slots[1] != null && slots[2] != null;
    }

    private void telemetryUpdateThrottled() {
        long now = System.currentTimeMillis();
        if (now - lastTelemMs >= TELEM_PERIOD_MS) {
            telemetry.update();
            lastTelemMs = now;
        }

    }

    private void telemetryLimeLight() {
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            telemetry.addLine("LLResult = null");
            return;
        }

        Pose3D targetCam = getBestTagPoseCameraSpace(result);
        if (targetCam != null) {
            double xIn = targetCam.getPosition().x * 39.37;
            double yIn = targetCam.getPosition().y * 39.37;
            double zIn = targetCam.getPosition().z * 39.37;

            double distTag = Math.sqrt(xIn*xIn + yIn*yIn + zIn*zIn);
            lastTagDistanceIn = distTag;

            telemetry.addData("Distance from tag (in)", "%.1f", distTag);
        } else {
            lastTagDistanceIn = -1;
            telemetry.addLine("TagCam: none");
        }

        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            telemetry.addLine("botpose = null (localization not producing pose)");
            return; // prevent null crash below
        }

        double x = botpose.getPosition().x;
        double y = botpose.getPosition().y;
        double z = botpose.getPosition().z;

        telemetry.addData("Botpose x,y,z (m)", "(%.3f, %.3f, %.3f)", x, y, z);
    }
    private void updateMotifListener() {
        if (!motifListenEnabled) return;

        LLResult result = limelight.getLatestResult();
        if (result == null) return;

        // Try to latch motif (only latches once because updateLatchedMotif() checks motifLatched)
        updateLatchedMotif(result);

    }


    private boolean alignToTarget() {

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        int count = (tags == null) ? 0 : tags.size();
        telemetry.addData("LL valid", result.isValid());
        telemetry.addData("Fiducials", count);

        if (count > 0) {
            for (int i = 0; i < Math.min(6, count); i++) {
                LLResultTypes.FiducialResult t = tags.get(i);
                telemetry.addData("Tag[" + i + "]", "id=%d area=%.3f",
                        t.getFiducialId(), t.getTargetArea());
            }
        }
        if (result == null || !result.isValid()) {
            stopDrive();
            telemetry.addLine("ALIGN: no Limelight result");
            return false;
        }

        // --- Find best ALIGNMENT tag (20 or 24 only) ---
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
            telemetry.addLine("ALIGN: tag 20/24 not visible");
            return false;
        }

        // --- Compute tx from pose (THIS replaces getTx) ---
        Pose3D pose = best.getTargetPoseCameraSpace();
        if (pose == null) {
            stopDrive();
            telemetry.addLine("ALIGN: no pose");
            return false;
        }

        double x = pose.getPosition().x; // meters (left/right)
        double z = pose.getPosition().z; // meters (forward)
        double xCam = pose.getPosition().x; // meters (camera space, +left)
        double zCam = pose.getPosition().z; // meters forward

        // Correct for camera being offset LEFT of robot center
        double xCorrected = xCam + CAMERA_X_OFFSET_IN * IN_TO_M;

        // Compute yaw error using corrected x
        double tx = Math.toDegrees(Math.atan2(xCorrected, zCam));

        telemetry.addData("ALIGN",
                "tag=%d tx=%.2f x=%.2f z=%.2f",
                best.getFiducialId(), tx, x, z);

        // --- Check alignment ---
        if (Math.abs(tx) <= ALIGN_TOLERANCE) {
            stopDrive();
            telemetry.addLine("ALIGN: aligned");
            return true;
        }

        // --- Turn only (robot-centric) ---
        double turn = -tx * ALIGN_KP;   // flip sign if turns wrong way
        turn = Math.max(-ALIGN_MAX_POWER, Math.min(ALIGN_MAX_POWER, turn));

        driveRobotCentric(0, 0, turn);

        return false;
    }

    private boolean atShootSlot(int slot) {
        switch (slot) {
            case 0: return atShootSlot0();
            case 1: return atShootSlot1();
            case 2: return atShootSlot2();
            default: return false;
        }
    }
    private boolean atIntakeSlot(int slot) {
        switch (slot) {
            case 0: return atSlot0();
            case 1: return atSlot1();
            case 2: return atSlot2();
            default: return false;
        }
    }


    private void rotateToIntakeSlotBlocking(int targetSlot) {
        ensureKickerRetracted();

        // If already in target window, leave first
        if (atIntakeSlot(targetSlot)) {
            long leaveStart = System.currentTimeMillis();
            servo1.setPower(MGR_CRAWL_POWER);
            while (opModeIsActive()
                    && atIntakeSlot(targetSlot)
                    && (System.currentTimeMillis() - leaveStart) < 400) {
                idle();
            }
            servo1.setPower(0);
        }

        long start = System.currentTimeMillis();
        servo1.setPower(MGR_FAST_POWER);

        while (opModeIsActive()
                && !atIntakeSlot(targetSlot)
                && (System.currentTimeMillis() - start) < MGR_MOVE_TIMEOUT_MS) {
            idle();
        }

        servo1.setPower(0);
    }

    private int findSlotForColor(char wanted) {
        String w = String.valueOf(wanted);
        for (int i = 0; i < 3; i++) {
            if (!slotFired[i] && w.equals(slots[i])) return i;
        }
        return -1;
    }

    private void driveRobotCentric(double y, double x, double rx) {
        double fl = y + x + rx; // motor2 (FL)
        double fr = y - x - rx; // motor0 (FR)
        double bl = y - x + rx; // motor1 (BL)
        double br = y + x - rx; // motor3 (BR)

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        motor2.setPower(fl / max);
        motor0.setPower(fr / max);
        motor1.setPower(bl / max);
        motor3.setPower(br / max);
    }
    private void rotateToSlotBlocking(int targetSlot) {
        shootingBusy = true;
        try {
            ensureKickerRetracted();

            // If we are already sitting in the target slot window, force a REAL leave first
            if (atShootSlot(targetSlot)) {
                servo1.setPower(MGR_CRAWL_POWER);
                long leaveStart = System.currentTimeMillis();
                while (opModeIsActive()
                        && !notAtShootSlotStable(targetSlot, LEAVE_STABLE_MS)
                        && (System.currentTimeMillis() - leaveStart) < 600) {
                    idle();
                }
                servo1.setPower(0);
            }

            // Rotate until we ENTER the target window, then require stability
            long start = System.currentTimeMillis();
            servo1.setPower(MGR_FAST_POWER);

            while (opModeIsActive() && (System.currentTimeMillis() - start) < MGR_MOVE_TIMEOUT_MS) {

                // First: did we hit the target slot window?
                if (atShootSlot(targetSlot)) {
                    // Second: is it stable (not just a 1-frame flicker)?
                    if (atShootSlotStable(targetSlot, SLOT_STABLE_MS)) {
                        break; // we are really there
                    }
                }
                idle();
            }

            servo1.setPower(0);

            // Safety: if not actually stable at target, give up (magnet failed or missed)
            if (!atShootSlot(targetSlot)) return;

            // Optional brake / reverse nudge (keep yours)
            servo1.setPower(MGR_BRAKE_POWER);
            sleep(MGR_BRAKE_MS);
            servo1.setPower(0);

            rotateTrayBackOneDegree();

        } finally {
            servo1.setPower(0);
            shootingBusy = false;
        }
    }
    private void rotateTrayBackOneDegree() {
        servo1.setPower(MGR_BACKUP_POWER);
        sleep(MGR_BACKUP_1DEG_MS);
        servo1.setPower(0);
    }

    private void stopDrive() {
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }
    private String decodeMotifFromTagId(int tagId) {
        switch (tagId) {
            case 21:  return "gpp";
            case 22:  return "pgp";
            case 23:  return "ppg";
            default: return "UNKNOWN";
        }
    }

    private boolean isMotifTag(int id) {
        return (id == 21 || id == 22 || id == 23);
    }

    private void updateLatchedMotif(LLResult result) {
        if (motifLatched) return;
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;

        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult t : tags) {
            int id = t.getFiducialId();
            if (!isMotifTag(id)) continue; // <-- ONLY 21-23
            if (best == null || t.getTargetArea() > best.getTargetArea()) best = t;
        }
        if (best == null) return;

        int id = best.getFiducialId();
        String motif = decodeMotifFromTagId(id);

        if (!"UNKNOWN".equals(motif)) {
            motifLatched = true;
            latchedTagId = id;
            latchedMotif = motif;
        }
    }

    private int getStableShootSlot(long stableMs, long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {
            int s = getCurrentShootSlot();
            if (s >= 0 && atShootSlotStable(s, stableMs)) return s;
            idle();
        }
        return -1;
    }

    private boolean atShootSlotStable(int slot, long stableMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < stableMs) {
            if (!atShootSlot(slot)) return false;
            idle();
        }
        return true;
    }

    private boolean notAtShootSlotStable(int slot, long stableMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < stableMs) {
            if (atShootSlot(slot)) return false;  // still in slot window
            idle();
        }
        return true;
    }

    private void telemetryBallColor() {
        NormalizedRGBA c = ballColor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.RGBToHSV((int)(c.red*255), (int)(c.green*255), (int)(c.blue*255), hsv);

        //telemetry.addData("BallHue", "%.0f", hsv[0]);
        //telemetry.addData("Ball HSV", "H=%.0f S=%.2f V=%.2f", hsv[0], hsv[1], hsv[2]);
        //telemetry.addData("Ball RGB", "r=%.2f g=%.2f b=%.2f", c.red, c.green, c.blue);
    }

    private Pose3D getBestTagPoseCameraSpace(LLResult result) {
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;

        LLResultTypes.FiducialResult best = tags.get(0);
        for (LLResultTypes.FiducialResult t : tags) {
            if (t.getTargetArea() > best.getTargetArea()) best = t;
        }

        return best.getTargetPoseCameraSpace();
    }

    private void updateBallColor() {
        NormalizedRGBA c = ballColor.getNormalizedColors();

        float[] hsv = new float[3];
        Color.RGBToHSV(
                (int)(c.red   * 255),
                (int)(c.green * 255),
                (int)(c.blue  * 255),
                hsv
        );

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        boolean confident = (sat > 0.05) && (val > 0.009);

        if(confident && (hue >= 120 && hue <= 180)) {
            ballColorValue = "g";
        } else if(confident && (hue >= 210 && hue <= 255)) {
            ballColorValue = "p";
        } else {
            ballColorValue = null;
        }
    }

    private boolean colorSeen() {
        return ballColorValue != null && !ballColorValue.isEmpty();
    }

    // Intake slot detectors
    private boolean atSlot0() { return !mag0.getState() && !mag1.getState(); }
    private boolean atSlot1() { return !mag0.getState() && mag1.getState(); }
    private boolean atSlot2() { return mag0.getState() && !mag1.getState(); }

    // Shooter slot detectors (mag2/mag3)
    private boolean atShootSlot0() { return !mag2.getState() && mag3.getState(); }
    private boolean atShootSlot1() { return mag2.getState() &&  !mag3.getState(); }
    private boolean atShootSlot2() { return !mag2.getState() && !mag3.getState(); }

    private void assignSlot(int slotNumber) {
        slots[slotNumber] = ballColorValue;
    }
    private void returnTrayToIntakeSlot0() {
        shootingBusy = true;
        try {
            ensureKickerRetracted();

            // If we're already at intake slot 0, force a clean leave first
            if (atSlot0()) {
                servo1.setPower(MGR_CRAWL_POWER);
                long leaveStart = System.currentTimeMillis();
                while (opModeIsActive()
                        && atSlot0()
                        && (System.currentTimeMillis() - leaveStart) < 400) {
                    idle();
                }
                servo1.setPower(0);
            }

            // Rotate until intake slot 0 is detected
            long start = System.currentTimeMillis();
            servo1.setPower(MGR_FAST_POWER);

            while (opModeIsActive()
                    && !atSlot0()
                    && (System.currentTimeMillis() - start) < MGR_MOVE_TIMEOUT_MS) {
                idle();
            }

            servo1.setPower(0);

            // Reset intake state machine so it is READY to accept balls
            currentSlot = 0;
            intakeState = INTAKEState.WAIT_COLOR_0;
            colorLatched = false;

        } finally {
            servo1.setPower(0);
            shootingBusy = false;
        }
    }
    private int nextIntakeSlot(int current) {
        return (current + INTAKE_SLOT_STEP + 3) % 3;
    }

    private INTAKEState waitStateForSlot(int slot) {
        switch (slot) {
            case 0: return INTAKEState.WAIT_COLOR_0;
            case 1: return INTAKEState.WAIT_COLOR_1;
            case 2: return INTAKEState.WAIT_COLOR_2;
            default: return INTAKEState.WAIT_COLOR_0;
        }
    }
    private void merryGoRoundIntake() {
        // If shooter/tray rotation is happening, don't fight it
        if (shootingBusy) {
            servo1.setPower(0);
            return;
        }

        // ----------------------------
        // READ COLOR SENSOR
        // ----------------------------
        boolean seen = colorSeen();
        boolean newColorEvent = seen && !colorLatched;
        colorLatched = seen;

        // ============================================================
        // MANUAL STEP: d2 LEFT BUMPER always steps to next INTAKE slot
        // ============================================================
        boolean step = gamepad2.left_bumper;

        if (step && !wasForceSkipTrigger) {
            int from = getCurrentIntakeSlot();
            if (from < 0) from = currentSlot;   // fallback if magnets are "between" slots
            int to = nextIntakeSlot(from);

            telemetry.addData("ManualStep", "intake %d -> %d", from, to);

            // rotate using intake magnets (mag0/mag1)
            rotateToIntakeSlotBlocking(to);

            // after stepping, land in WAIT state for that slot so color can be read
            currentSlot = to;
            intakeState = waitStateForSlot(to);

            // allow a fresh color latch
            colorLatched = false;

            wasForceSkipTrigger = step;
            return; // important: don't also run the auto switch in the same loop
        }
        wasForceSkipTrigger = step;

        // ----------------------------
        // AUTO INTAKE STATE MACHINE
        // ----------------------------
        switch (intakeState) {

            case INIT_TO_SLOT0:
                ensureKickerRetracted();
                servo1.setPower(MGR_FAST_POWER);
                if (atSlot0()) {
                    servo1.setPower(0);
                    currentSlot = 0;
                    intakeState = INTAKEState.WAIT_COLOR_0;
                    colorLatched = false;
                }
                break;

            case WAIT_COLOR_0:
                servo1.setPower(0);
                if (newColorEvent) {
                    assignSlot(0);
                    intakeState = INTAKEState.MOVE_TO_SLOT1;
                }
                break;

            case MOVE_TO_SLOT1:
                ensureKickerRetracted();
                servo1.setPower(MGR_FAST_POWER);
                if (atSlot1()) {
                    servo1.setPower(0);
                    currentSlot = 1;
                    intakeState =INTAKEState.WAIT_COLOR_1;
                    colorLatched = false;
                }
                break;

            case WAIT_COLOR_1:
                servo1.setPower(0);
                if (newColorEvent) {
                    assignSlot(1);
                    intakeState =INTAKEState.MOVE_TO_SLOT2;
                }
                break;

            case MOVE_TO_SLOT2:
                ensureKickerRetracted();
                servo1.setPower(MGR_FAST_POWER);
                if (atSlot2()) {
                    servo1.setPower(0);
                    currentSlot = 2;
                    intakeState =INTAKEState.WAIT_COLOR_2;
                    colorLatched = false;
                }
                break;

            case WAIT_COLOR_2:
                servo1.setPower(0);
                if (newColorEvent) {
                    assignSlot(2);
                    intakeState = INTAKEState.DONE;
                }
                break;

            case DONE:
                servo1.setPower(0);
                break;
        }

        telemetry.addData("Intake State", intakeState);
        //telemetry.addData("BallColor", ballColorValue);
        telemetry.addData("Slots", "0=%s 1=%s 2=%s", slots[0], slots[1], slots[2]);
    }

    private void kickOnce() {
        servo0.setPosition(KICK_EXTEND_POS);
        sleep(KICK_DURATION_MS);
        servo0.setPosition(KICK_RETRACT_POS);
    }

    private void ensureKickerRetracted() {
        servo0.setPosition(KICK_RETRACT_POS);
    }

    private boolean wasStepShootPressed = false;

    private int getCurrentIntakeSlot() {
        if (atSlot0()) return 0;
        if (atSlot1()) return 1;
        if (atSlot2()) return 2;
        return -1;
    }

    private int getCurrentShootSlot() {
        if (atShootSlot0()) return 0;
        if (atShootSlot1()) return 1;
        if (atShootSlot2()) return 2;
        return -1;
    }

    private static final int SHOOT_SLOT_STEP = 1; // try +1 first, if wrong use -1
    private boolean motifReadyForStepShoot() {
        return motifLatched
                && latchedMotif != null
                && latchedMotif.length() == 3
                && !"UNKNOWN".equals(latchedMotif)
                && (slots[0] != null || slots[1] != null || slots[2] != null);
    }
    private void resetStepShootSequence() {
        for (int i = 0; i < 3; i++) slotFired[i] = false;
        stepShotIndex = 0;
        stepModeActive = true;
    }
    private void stepToNextSlotAndShoot() {
        // Don't fight a rotation already in progress
        if (shootingBusy) return;

        ensureKickerRetracted();

        // Must have a valid motif + all 3 stored colors
        if (!motifReadyForStepShoot()) {
            telemetry.addLine("StepShoot: Need motif + all slots loaded");
            telemetry.addData("motifLatched", motifLatched);
            telemetry.addData("latchedMotif", latchedMotif);
            telemetry.addData("Slots", "0=%s 1=%s 2=%s", slots[0], slots[1], slots[2]);
            return;
        }

        // If first time using step-shoot, reset the sequence
        if (!stepModeActive) {
            resetStepShootSequence();
        }

        // If we've already done 3 shots, reset for a new sequence
        if (stepShotIndex >= 3) {
            telemetry.addLine("StepShoot: Sequence complete, resetting");
            resetStepShootSequence();
        }

        // Which color do we want this press?
        char wanted = Character.toLowerCase(latchedMotif.charAt(stepShotIndex));

        // Find a stored slot that matches that color and is not fired yet
        int slotToShoot = findSlotForColor(wanted);
        if (slotToShoot < 0) {
            telemetry.addData("StepShoot", "No available '%c' left in stored slots", wanted);
            telemetry.addData("Slots", "0=%s 1=%s 2=%s", slots[0], slots[1], slots[2]);
            return;
        }

        // Convert intake slot index -> shoot frame slot index
        int shootFrameSlot = intakeSlotToShootSlot(slotToShoot);

        telemetry.addData("StepShoot",
                "press=%d wants=%c -> storedSlot=%d -> shootSlot=%d",
                stepShotIndex, wanted, slotToShoot, shootFrameSlot);

        // Rotate and fire
        rotateToSlotBlocking(shootFrameSlot);
        kickOnce();

        slotFired[slotToShoot] = true;
        slots[slotToShoot] = null;   // <-- clear so new ball can be stored here
        stepShotIndex++;

        sleep(120);
    }

    private int intakeSlotToShootSlot(int intakeSlot) {
        return (intakeSlot + INTAKE_TO_SHOOT_OFFSET + 3) % 3;
    }


    private void sticks2() {
        double gain;
        if (gamepad1.left_bumper) gain = 1;
        else gain = 0.5;

        drivePower  = gain * LSY;      // forward/back
        rotatePower = gain * LSX;      // turn
        strafePower = gain * RSX;      // strafe

        sticks4();
    }

    private void sticks4() {
        // Robot-centric: no IMU, no rotation correction
        correctedDrivePower  = drivePower;   // forward/back
        correctedStrafePower = strafePower;  // strafe left/right

        drive2();
    }

    private void drive2() {
        double y  = correctedDrivePower;   // forward
        double x  = correctedStrafePower;  // strafe
        double rx = rotatePower;           // turn

        double fl = y + x + rx; // motor2
        double fr = y - x - rx; // motor0
        double bl = y - x + rx; // motor1
        double br = y + x - rx; // motor3

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        motor2.setPower(fl / max);
        motor0.setPower(fr / max);
        motor1.setPower(bl / max);
        motor3.setPower(br / max);
    }
    private boolean wasRedToggle = false;
    private boolean wasBlueToggle = false;
    private void lights() {
        // Toggle RED with gamepad2.options
        if (gamepad2.options && !wasRedToggle) {
            isRedOn = !isRedOn;
            // Many REV digital outputs are "active low" depending on wiring:
            redLED.setState(!isRedOn);  // if backwards, change to redLED.setState(isRedOn)
        }
        wasRedToggle = gamepad2.options;

        // Toggle BLUE with gamepad2.share
        if (gamepad2.share && !wasBlueToggle) {
            isBlueOn = !isBlueOn;
            blueLED.setState(!isBlueOn); // if backwards, change to blueLED.setState(isBlueOn)
        }
        wasBlueToggle = gamepad2.share;
    }
    // ---------------- Movement: Forward (encoders) ----------------
    private static final double WHEEL_DIAMETER_M = 0.096;
    private static final double WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M;
    private static final double TICKS_PER_WHEEL_REV = 537.7;

    private void setDriveRunUsingEncoder() {
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    // --- Heading hold fixes ---
    private static final double HEADING_SIGN = +1.0; // +1 works for most bots; if it still turns wrong, set to -1
//    private static final double HEADING_DEADBAND_DEG = 0.25; // was 1.0; start correcting sooner
    private static final double HEADING_DEADBAND_DEG = 1.0; // was 1.0; start correcting sooner

    private void driveForwardMeters(double meters, double power) {
        setDriveRunUsingEncoder();

        // Distance is always magnitude
        double distM = Math.abs(meters);
        if (distM < 1e-6) return;

        // Direction comes ONLY from power sign
        if (Math.abs(power) < 1e-6) {
            stopDrive();
            return;
        }
        int dir = (power >= 0) ? +1 : -1; // +forward, -backward

        // meters -> ticks target
        int ticksTarget = (int) Math.round((distM / WHEEL_CIRCUMFERENCE_M) * TICKS_PER_WHEEL_REV);

        // Start encoders
        int startFR = motor0.getCurrentPosition();
        int startBL = motor1.getCurrentPosition();
        int startFL = motor2.getCurrentPosition();
        int startBR = motor3.getCurrentPosition();

        // Heading hold reference
        double startYaw = getYawDeg();

        // Magnitude of requested power (0..1)
        double p = clip(Math.abs(power), 0.0, 1.0);

        // sticks2 gain is 0.5 unless left_bumper is held
        double gain = 0.5;

        // Convert desired final forward power -> stick input (pre-gain)
        double stickLSY = clip(dir * (p / gain), -1.0, 1.0);

        // ---- IMU heading-hold tuning ----
        final double HEADING_KP = 0.015;
        final double MAX_CORR   = 0.20;
        final long   TIMEOUT_MS = 6000;

        long startMs = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - startMs) < TIMEOUT_MS) {
            int dFR = motor0.getCurrentPosition() - startFR; // FR
            int dBL = motor1.getCurrentPosition() - startBL; // BL (already respects motor direction)
            int dFL = motor2.getCurrentPosition() - startFL; // FL
            int dBR = motor3.getCurrentPosition() - startBR; // BR
            int dBL_fixed = -dBL;
            // ---- Distance estimator (robust): average of magnitudes ----
            // This prevents sign-cancellation (your main "drives too far" bug).
            double travelTicks = (Math.abs(dFL) + Math.abs(dFR) + Math.abs(dBL) + Math.abs(dBR)) / 4.0;

            // Optional: signed forward estimate (telemetry/debug only)
            double forwardTicksSigned = (dFL + dFR + dBL + dBR) / 4.0;

            if (travelTicks >= ticksTarget) break;

            // ---- Heading correction ----
            double yaw = getYawDeg();

            // Error is "how far we've drifted from startYaw"
            // Use yaw-start and a sign multiplier so it corrects the right way.
            double errDeg = angleWrapDeg(yaw - startYaw);

            if (Math.abs(errDeg) < HEADING_DEADBAND_DEG) errDeg = 0.0;

            // If yaw drifted +, we want a - turn (most bots). HEADING_SIGN lets you match your IMU/drive sign.
            double corr = clip((-HEADING_SIGN) * errDeg * HEADING_KP, -MAX_CORR, MAX_CORR);

            // convert final rotation -> stick (pre-gain)
            double stickLSX = clip(corr / gain, -1.0, 1.0);

            // Virtual sticks:
            LSY = (float) stickLSY;   // forward/back
            LSX = (float) stickLSX;   // correction turn
            RSX = 0f;                 // no strafe

            sticks2();

            telemetry.addData("DriveIMU", "dist=%.2fm target=%d travelTicks=%.0f fwdTicks=%.0f",
                    distM, ticksTarget, travelTicks, forwardTicksSigned);
            telemetry.addData("HeadingHold", "start=%.1f yaw=%.1f err=%.1f corr=%.2f",
                    startYaw, yaw, errDeg, corr);
            telemetryUpdateThrottled();
            idle();
        }

        // Stop and clear sticks
        LSY = 0f; LSX = 0f; RSX = 0f;
        sticks2();
        stopDrive();
    }




    private double getYawDeg() {
        // ZYX: firstAngle = Z (yaw)
        return imu1.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;
    }
    private static double angleWrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    // ---------------- Movement: Turn right (IMU) ----------------
    // ---------------- Movement: Turn right (IMU) ----------------
    private void turnRightDegrees(double degrees, double maxPower) {
        double startYaw = getYawDeg();
        double targetYaw = angleWrapDeg(startYaw - degrees); // right turn typically decreases yaw

        final double TOL_DEG = 2.0;
        final double MIN_OUT = 0.10;
        final double KP      = 0.012;
        final long   TIMEOUT = 4000;

        double pMax = clip(Math.abs(maxPower), 0.0, 1.0);

        double gain = (gamepad1.left_bumper) ? 1.0 : 0.5;

        long startMs = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startMs) < TIMEOUT) {
            double yaw = getYawDeg();
            double err = angleWrapDeg(targetYaw - yaw);

            telemetry.addData("Turn", "start=%.1f target=%.1f yaw=%.1f err=%.1f",
                    startYaw, targetYaw, yaw, err);
            telemetryUpdateThrottled();

            if (Math.abs(err) <= TOL_DEG) break;

            double out = err * KP;
            out = clip(out, -pMax, pMax);

            if (Math.abs(out) < MIN_OUT) out = MIN_OUT * Math.signum(out);
            if (Math.abs(err) < 10.0) out = clip(out, -0.18, 0.18);

            // Convert desired final rotate to stick (pre-gain)
            double stickLSX = clip(out / gain, -1.0, 1.0);

            // Pure mecanum rotation (no drive/strafe)
            LSY = 0f;
            RSX = 0f;
            LSX = (float) stickLSX;

            sticks2();
            idle();
        }

        LSY = 0f; LSX = 0f; RSX = 0f;
        sticks2();
        stopDrive();
        sleep(120);
    }


}
