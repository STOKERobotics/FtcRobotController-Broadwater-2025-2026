package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous(name = "Broadwater Robotics Auto (TeleOp-aligned)")
public class broadwater_robotics_25_26_auto extends LinearOpMode {

    // -------------------- Hardware --------------------
    private DcMotor motor0;  // FR
    private DcMotor motor1;  // BL
    private DcMotor motor2;  // FL
    private DcMotor motor3;  // BR
    private DcMotor motor0b; // Shooter 1
    private DcMotor motor1b; // Shooter 2
    private DcMotor motor2b; // Intake

    private Servo servo0;    // Kicker
    private CRServo servo1;  // Merry Go Round Tray
    private Servo servo2;    // Shooter Angle

    private BNO055IMU imu1;
    private AnalogInput laser;

    private DigitalChannel mag0;
    private DigitalChannel mag1;
    private DigitalChannel mag2;
    private DigitalChannel mag3;

    private NormalizedColorSensor ballColor;

    // -------------------- Limelight --------------------
    private Limelight3A limelight;

    // Align
    private static final double ALIGN_KP = 0.03;
    private static final double ALIGN_TOLERANCE = 1.0;   // deg
    private static final double ALIGN_MAX_POWER = 0.3;

    // Shooter angle mapping
    private static final double SHOOTER_MIN_DIST = 50.0;   // inches
    private static final double SHOOTER_MAX_DIST = 120.0;  // inches

    private static final double SERVO_MIN_ANGLE = 0.25;
    private static final double SERVO_MAX_ANGLE = 0.85;

    // Kicker timing (single tap like your revised TeleOp)
    private static final double KICK_EXTEND_POS = 1.0;
    private static final double KICK_RETRACT_POS = 0.0;
    private static final long   KICK_DURATION_MS = 350;

    // Servo2 manual adjust (used in MANUAL fallback)
    private double servo2Pos = 0.5;
    private static final double SERVO2_MIN = 0.0;
    private static final double SERVO2_MAX = 1.0;
    private static final double SERVO2_RATE = 0.6;
    private double lastServo2Time = 0.0;

    // Motif latch
    private boolean motifLatched = false;
    private int latchedTagId = -1;
    private String latchedMotif = "NONE";
    private double lastTagDistanceIn = -1;

    // Merry-go-round intake
    private enum INTAKEState { INIT_TO_SLOT0, WAIT_COLOR_0, MOVE_TO_SLOT1, WAIT_COLOR_1, MOVE_TO_SLOT2, WAIT_COLOR_2, DONE }
    private INTAKEState intakeState = INTAKEState.INIT_TO_SLOT0;

    private boolean colorLatched = false; // edge detect
    private static final double MGR_POWER = 1.0;
    private static final long MGR_MOVE_TIMEOUT_MS = 2000;

    // Slot storage + fired tracking
    private final boolean[] slotFired = new boolean[3];
    private final String[] slots = new String[3]; // "g" or "p"
    private String ballColorValue;

    // -------------------- SEQUENCE ENGINE (kept) --------------------
    private enum ControlMode { MANUAL, SEQUENCE }
    private ControlMode controlMode = ControlMode.SEQUENCE;

    private enum StepType {
        DRIVE_TO,
        INTAKE_UNTIL_FULL,
        ALIGN_TO_TAG,
        SHOOT_MOTIF,
        WAIT,
        DONE
    }

    private static class Command {
        StepType type;

        // DRIVE_TO
        double x, y, headingDeg;
        long timeoutMs;

        // WAIT
        long waitMs;

        Command(StepType t) { type = t; }

        static Command driveTo(double x, double y, double headingDeg, long timeoutMs) {
            Command c = new Command(StepType.DRIVE_TO);
            c.x = x; c.y = y; c.headingDeg = headingDeg;
            c.timeoutMs = timeoutMs;
            return c;
        }

        static Command intakeUntilFull(long timeoutMs) {
            Command c = new Command(StepType.INTAKE_UNTIL_FULL);
            c.timeoutMs = timeoutMs;
            return c;
        }

        static Command alignToTag(long timeoutMs) {
            Command c = new Command(StepType.ALIGN_TO_TAG);
            c.timeoutMs = timeoutMs;
            return c;
        }

        static Command shootMotif(long timeoutMs) {
            Command c = new Command(StepType.SHOOT_MOTIF);
            c.timeoutMs = timeoutMs;
            return c;
        }

        static Command waitMs(long waitMs) {
            Command c = new Command(StepType.WAIT);
            c.waitMs = waitMs;
            return c;
        }

        static Command done() { return new Command(StepType.DONE); }
    }

    private static class PoseEstimate {
        final double x, y, headingDeg;
        PoseEstimate(double x, double y, double headingDeg) {
            this.x = x; this.y = y; this.headingDeg = headingDeg;
        }
    }

    private int seqIndex = 0;
    private long stepStartMs = 0;

    // -------------------- Botpose navigation tuning --------------------
    private static final double KP_XY = 1.2;     // meters -> stick
    private static final double KP_H  = 0.02;    // deg -> stick

    private static final double MAX_DRIVE_POWER  = 0.55;
    private static final double MAX_STRAFE_POWER = 0.55;
    private static final double MAX_TURN_POWER   = 0.35;

    private static final double POS_TOL_M    = 0.05; // 5cm
    private static final double ANG_TOL_DEG  = 3.0;
    private static final int RED_TAG_ID  = 21;  // <-- put your real red tag ID here
    private static final int BLUE_TAG_ID = 22;  // <-- put your real blue tag ID here

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED; // default; you can change this later

    // ===== EDIT YOUR SEQUENCE HERE =====
    private final Command[] sequence = new Command[] {
            //Command.driveTo(0.00, 0.00, 0, 5000),
            Command.alignToTag(3000),
            Command.shootMotif(0),
            Command.done()
    };
    // ===================================

    @Override
    public void runOpMode() {
        initLimelight();

        telemetry.addLine("Broadwater Auto ready (TeleOp-aligned).");
        telemetry.addLine("Runs SEQUENCE. Press BACK to cancel to MANUAL.");
        telemetry.update();

        // Hardware map
        motor0  = hardwareMap.get(DcMotor.class, "motor0");
        motor1  = hardwareMap.get(DcMotor.class, "motor1");
        motor2  = hardwareMap.get(DcMotor.class, "motor2");
        motor3  = hardwareMap.get(DcMotor.class, "motor3");
        motor0b = hardwareMap.get(DcMotor.class, "motor0b");
        motor1b = hardwareMap.get(DcMotor.class, "motor1b");
        motor2b = hardwareMap.get(DcMotor.class, "motor2b");

        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        laser = hardwareMap.get(AnalogInput.class, "laser");
        imu1  = hardwareMap.get(BNO055IMU.class, "imu 1");

        ballColor = hardwareMap.get(NormalizedColorSensor.class, "ballColor");

        mag0 = hardwareMap.get(DigitalChannel.class, "mag0");
        mag1 = hardwareMap.get(DigitalChannel.class, "mag1");
        mag2 = hardwareMap.get(DigitalChannel.class, "mag2");
        mag3 = hardwareMap.get(DigitalChannel.class, "mag3");

        // -------------------- Motor directions (match revised TeleOp) --------------------
        motor0.setDirection(DcMotor.Direction.FORWARD);
        motor1.setDirection(DcMotor.Direction.REVERSE); // IMPORTANT: TeleOp uses REVERSE
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);

        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor0b.setDirection(DcMotor.Direction.REVERSE);
        motor1b.setDirection(DcMotor.Direction.FORWARD);
        motor2b.setDirection(DcMotorSimple.Direction.FORWARD);

        motor0b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor0b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servo init
        servo0.setPosition(KICK_RETRACT_POS);
        servo2.setPosition(servo2Pos);
        lastServo2Time = getRuntime();

        // IMU init (kept for MANUAL field-centric fallback)
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu1.initialize(imuParameters);

        // Enable sensor light if present
        if (ballColor instanceof SwitchableLight) {
            ((SwitchableLight) ballColor).enableLight(true);
        }

        // Sequence setup
        controlMode = ControlMode.SEQUENCE;
        seqIndex = 0;
        stepStartMs = System.currentTimeMillis();


        waitForStart();

        while (opModeIsActive()) {

            // Cancel sequence if needed
            if (gamepad1.back && controlMode == ControlMode.SEQUENCE) cancelSequence();

            // Always update telemetry + latching
            getData();
            telemetryLimeLight();

            // Optional: if you want these always on like TeleOp, uncomment
            // motor0b.setPower(1);
            // motor1b.setPower(1);
            // motor2b.setPower(1);

            if (controlMode == ControlMode.SEQUENCE) {
                runSequence();
            } else {
                // MANUAL fallback uses your revised TeleOp-style IMU field-centric
                sticksManualTeleopStyle();
                buttonsManualServo2();
                updateBallColor();
                merryGoRoundIntake();
                telemetryBallColor();
            }

            // Laser telemetry
            double v = laser.getVoltage();
            double mm = (v / 3.3) * 1000.0;
            double inches = mm / 25.4;
            telemetry.addData("Laser Dist", "%.0f mm  (%.1f in)", mm, inches);

            telemetry.addData("MODE", controlMode);
            telemetry.update();
        }
    }

    // -------------------- Limelight --------------------
    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    private void telemetryLimeLight() {
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            telemetry.addLine("LLResult = null");
            return;
        }

        telemetryTagMotifs(result);
        updateLatchedMotif(result);
        telemetry.addData("Stored Tag Colors", "%s (Tag %d)", latchedMotif, latchedTagId);

        Pose3D targetCam = getBestTagPoseCameraSpace(result);
        if (targetCam != null) {
            double xIn = targetCam.getPosition().x * 39.37;
            double yIn = targetCam.getPosition().y * 39.37;
            double zIn = targetCam.getPosition().z * 39.37;

            double distTag = Math.sqrt(xIn * xIn + yIn * yIn + zIn * zIn);
            lastTagDistanceIn = distTag;
            telemetry.addData("Distance from tag (in)", "%.1f", distTag);
        } else {
            lastTagDistanceIn = -1;
            telemetry.addLine("TagCam: none");
        }

        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            telemetry.addLine("botpose = null");
            return;
        }

        telemetry.addData("Botpose x,y,z (m)", "(%.3f, %.3f, %.3f)",
                botpose.getPosition().x, botpose.getPosition().y, botpose.getPosition().z);
    }

    // -------------------- Sequence Engine --------------------
    private void cancelSequence() {
        controlMode = ControlMode.MANUAL;
        stopDrive();
    }

    private void runSequence() {
        if (seqIndex >= sequence.length) {
            controlMode = ControlMode.MANUAL;
            stopDrive();
            return;
        }

        Command cmd = sequence[seqIndex];
        long elapsed = System.currentTimeMillis() - stepStartMs;

        telemetry.addData("SEQ", "Step %d/%d  %s  t=%dms",
                (seqIndex + 1), sequence.length, cmd.type, elapsed);

        boolean stepDone = false;

        switch (cmd.type) {
            case DRIVE_TO: {
                boolean arrived = driveToBotpose(cmd.x, cmd.y, cmd.headingDeg);
                if (arrived) stepDone = true;

                if (elapsed > cmd.timeoutMs) {
                    telemetry.addLine("DRIVE_TO timeout -> cancel");
                    cancelSequence();
                    return;
                }
                break;
            }

            case INTAKE_UNTIL_FULL: {
                updateBallColor();
                merryGoRoundIntake();
                telemetryBallColor();

                if (allSlotsLoaded()) stepDone = true;

                if (elapsed > cmd.timeoutMs) {
                    telemetry.addLine("INTAKE timeout -> continuing");
                    stepDone = true;
                }
                break;
            }

            case ALIGN_TO_TAG: {
                boolean aligned = alignToTarget();
                if (aligned) stepDone = true;

                if (elapsed > cmd.timeoutMs) {
                    telemetry.addLine("ALIGN timeout -> cancel");
                    cancelSequence();
                    return;
                }
                break;
            }

            case SHOOT_MOTIF: {
                adjustShooterAndFire();
                stepDone = true;
                break;
            }

            case WAIT: {
                stopDrive();
                if (elapsed >= cmd.waitMs) stepDone = true;
                break;
            }

            case DONE: {
                stopDrive();
                controlMode = ControlMode.MANUAL;
                return;
            }
        }

        if (stepDone) {
            stopDrive();
            sleep(120);
            seqIndex++;
            stepStartMs = System.currentTimeMillis();
        }
    }

    // -------------------- BOTPOSE NAV --------------------
    private boolean driveToBotpose(double targetX, double targetY, double targetHeadingDeg) {

        PoseEstimate cur = getPoseEstimate();
        if (cur == null) {
            stopDrive();
            telemetry.addLine("botpose null -> can't navigate");
            return false;
        }

        double dx = targetX - cur.x;
        double dy = targetY - cur.y;
        double dh = angleWrapDeg(targetHeadingDeg - cur.headingDeg);

        double dist = Math.hypot(dx, dy);

        boolean posOk = dist <= POS_TOL_M;
        boolean angOk = Math.abs(dh) <= ANG_TOL_DEG;
        if (posOk && angOk) {
            stopDrive();
            return true;
        }

        double hRad = Math.toRadians(cur.headingDeg);
        double robotForward =  dx * Math.cos(hRad) + dy * Math.sin(hRad);
        double robotLeft    = -dx * Math.sin(hRad) + dy * Math.cos(hRad);

        double posScale = clip(dist / 0.60, 0.18, 1.0);
        double angScale = clip(Math.abs(dh) / 25.0, 0.18, 1.0);

        double LSY = clip(robotForward * KP_XY * posScale, -MAX_DRIVE_POWER,  MAX_DRIVE_POWER);
        double LSX = clip(robotLeft    * KP_XY * posScale, -MAX_STRAFE_POWER, MAX_STRAFE_POWER);
        double RSX = clip(dh          * KP_H  * angScale, -MAX_TURN_POWER,   MAX_TURN_POWER);

        if (dist < 0.15) {
            LSY = 0;
            LSX = 0;
        }

        driveByIntent(LSY, LSX, RSX);

        telemetry.addData("BOTPOSE", "x=%.3f y=%.3f h=%.1f", cur.x, cur.y, cur.headingDeg);
        telemetry.addData("TARGET",  "x=%.3f y=%.3f h=%.1f", targetX, targetY, targetHeadingDeg);
        telemetry.addData("ERR",     "dx=%.3f dy=%.3f dist=%.3f dh=%.1f", dx, dy, dist, dh);
        telemetry.addData("ROBOT_ERR", "fwd=%.3f left=%.3f", robotForward, robotLeft);
        telemetry.addData("FAKE STICKS", "LSY=%.2f LSX=%.2f RSX=%.2f", LSY, LSX, RSX);

        return false;
    }

    private PoseEstimate getPoseEstimate() {
        LLResult result = limelight.getLatestResult();
        if (result == null) return null;

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return null;

        double x = botpose.getPosition().x; // meters
        double y = botpose.getPosition().y; // meters
        double headingDeg = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

        return new PoseEstimate(x, y, headingDeg);
    }

    // Same mixer style as your TeleOp convention (+ normalization)
    private void driveByIntent(double lsy, double lsx, double rsx) {
        double drive  = lsy;
        double strafe = lsx;
        double turn   = rsx;

        // Matches your TeleOp convention
        double correctedDrive  = drive;
        double correctedStrafe = -strafe;

        double fr = (correctedDrive - correctedStrafe) - turn; // motor0
        double fl = (correctedDrive + correctedStrafe) + turn; // motor2
        double br = (correctedDrive - correctedStrafe) + turn; // motor3
        double bl = (correctedDrive + correctedStrafe) - turn; // motor1

        double max = Math.max(1.0,
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(fl),
                                Math.max(Math.abs(br), Math.abs(bl)))));

        fr /= max; fl /= max; br /= max; bl /= max;

        motor0.setPower(fr);
        motor2.setPower(fl);
        motor3.setPower(br);
        motor1.setPower(bl);
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double angleWrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    // -------------------- Align --------------------
    private boolean alignToTarget() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("Alignment", "No valid LL result");
            stopDrive();
            return false;
        }

        int wantedId = getAllianceTagId();

        // Find best matching tag by ID
        LLResultTypes.FiducialResult best = getBestFiducialById(result, wantedId);
        if (best == null) {
            telemetry.addData("Alignment", "No %s tag detected (id=%d)",
                    (alliance == Alliance.RED ? "RED" : "BLUE"), wantedId);
            stopDrive();
            return false;
        }

        // Compute tx (deg) for THIS fiducial from camera-space pose
        Pose3D camPose = best.getTargetPoseCameraSpace();
        if (camPose == null) {
            telemetry.addLine("Alignment: tag pose missing");
            stopDrive();
            return false;
        }

        // In camera space: +x is right, +z is forward (Limelight pose)
        double x = camPose.getPosition().x;
        double z = camPose.getPosition().z;

        // Horizontal angle offset in degrees (tx-like)
        double tx = Math.toDegrees(Math.atan2(x, z));

        if (Math.abs(tx) <= ALIGN_TOLERANCE) {
            stopDrive();
            telemetry.addData("Alignment", "Aligned to %s!",
                    (alliance == Alliance.RED ? "RED" : "BLUE"));

            // Latch motif + distance FROM THIS TAG
            latchMotifAndDistanceFromFiducial(best);
            return true;
        }

        double turnPower = Math.max(-ALIGN_MAX_POWER, Math.min(ALIGN_MAX_POWER, tx * ALIGN_KP));
        driveByIntent(0, 0, turnPower);

        telemetry.addData("Aligning", "%s id=%d tx=%.2f pwr=%.2f",
                (alliance == Alliance.RED ? "RED" : "BLUE"),
                best.getFiducialId(), tx, turnPower);

        return false;
    }

    // -------------------- Shooter + motif firing --------------------
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
        servo2.setPosition(servoAngle);

        telemetry.addData("Motif", latchedMotif);
        telemetry.addData("Slots", "0=%s 1=%s 2=%s", slots[0], slots[1], slots[2]);
        telemetry.addData("Dist(in)", "%.1f", distanceInches);
        telemetry.addData("Shooter Angle", "%.2f", servoAngle);
        telemetry.update();

        for (int i = 0; i < 3; i++) slotFired[i] = false;

        char[] order = latchedMotif.toCharArray();

        for (int shotIndex = 0; shotIndex < 3 && opModeIsActive(); shotIndex++) {
            char wanted = order[shotIndex];

            int slotToShoot = findSlotForColor(wanted);
            if (slotToShoot < 0) {
                telemetry.addData("Shooter", "Missing color '%c' in slots (or already used)", wanted);
                telemetry.update();
                return;
            }

            telemetry.addData("Shooting", "shot %d wants %c -> slot %d", shotIndex, wanted, slotToShoot);
            telemetry.update();

            rotateToSlotBlocking(slotToShoot);

            // Single-tap kick (TeleOp-aligned)
            kickOnce();

            slotFired[slotToShoot] = true;
            sleep(120);
        }

        telemetry.addLine("Shoot sequence done!");
    }

    private void kickOnce() {
        servo0.setPosition(KICK_EXTEND_POS);
        sleep(KICK_DURATION_MS);
        servo0.setPosition(KICK_RETRACT_POS);
    }

    private boolean allSlotsLoaded() {
        return slots[0] != null && slots[1] != null && slots[2] != null;
    }

    private int findSlotForColor(char wanted) {
        String w = String.valueOf(wanted);
        for (int i = 0; i < 3; i++) {
            if (!slotFired[i] && w.equals(slots[i])) return i;
        }
        return -1;
    }

    private boolean atShootSlot(int slot) {
        switch (slot) {
            case 0: return atShootSlot0();
            case 1: return atShootSlot1();
            case 2: return atShootSlot2();
            default: return false;
        }
    }
    private LLResultTypes.FiducialResult getBestFiducialById(LLResult result, int wantedId) {
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;

        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult t : tags) {
            if (t.getFiducialId() != wantedId) continue;
            if (best == null || t.getTargetArea() > best.getTargetArea()) best = t;
        }
        return best;
    }

    private void latchMotifAndDistanceFromFiducial(LLResultTypes.FiducialResult tag) {
        if (tag == null) return;

        int id = tag.getFiducialId();
        String motif = decodeMotifFromTagId(id);

        if (!"UNKNOWN".equals(motif)) {
            motifLatched = true;
            latchedTagId = id;
            latchedMotif = motif;
        }

        Pose3D camPose = tag.getTargetPoseCameraSpace();
        if (camPose != null) {
            double xIn = camPose.getPosition().x * 39.37;
            double yIn = camPose.getPosition().y * 39.37;
            double zIn = camPose.getPosition().z * 39.37;
            lastTagDistanceIn = Math.sqrt(xIn*xIn + yIn*yIn + zIn*zIn);
        } else {
            lastTagDistanceIn = -1;
        }
    }

    private void rotateToSlotBlocking(int targetSlot) {
        long start = System.currentTimeMillis();
        servo1.setPower(MGR_POWER);

        while (opModeIsActive() && !atShootSlot(targetSlot)
                && (System.currentTimeMillis() - start) < MGR_MOVE_TIMEOUT_MS) {

            telemetry.addData("Rotating to shoot slot", targetSlot);
            telemetry.addData("Mag2", mag2.getState());
            telemetry.addData("Mag3", mag3.getState());
            telemetry.update();
            idle();
        }

        servo1.setPower(0);
    }

    // -------------------- Tag motifs --------------------
    private String decodeMotifFromTagId(int tagId) {
        switch (tagId) {
            case 21:  return "gpp";
            case 22:  return "pgp";
            case 23:  return "ppg";
            default: return "UNKNOWN";
        }
    }

    private void updateLatchedMotif(LLResult result) {
        if (motifLatched) return;
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;

        LLResultTypes.FiducialResult best = tags.get(0);
        for (LLResultTypes.FiducialResult t : tags) {
            if (t.getTargetArea() > best.getTargetArea()) best = t;
        }

        int id = best.getFiducialId();
        String motif = decodeMotifFromTagId(id);

        if (!"UNKNOWN".equals(motif)) {
            motifLatched = true;
            latchedTagId = id;
            latchedMotif = motif;
        }
    }

    private void telemetryTagMotifs(LLResult result) {
        if (result == null || !result.isValid()) {
            telemetry.addLine("Tags: none (no valid LL result)");
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            telemetry.addLine("Tags: none detected");
            return;
        }

        telemetry.addData("Tags Detected", tags.size());
        for (int i = 0; i < tags.size(); i++) {
            LLResultTypes.FiducialResult t = tags.get(i);
            int id = t.getFiducialId();
            String motif = decodeMotifFromTagId(id);
            telemetry.addData("April Tag Motif", motif);
        }
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

    // -------------------- Ball color + intake --------------------
    private void telemetryBallColor() {
        NormalizedRGBA c = ballColor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.RGBToHSV((int) (c.red * 255), (int) (c.green * 255), (int) (c.blue * 255), hsv);

        telemetry.addData("BallHue", "%.0f", hsv[0]);
        telemetry.addData("Ball HSV", "H=%.0f S=%.2f V=%.2f", hsv[0], hsv[1], hsv[2]);
        telemetry.addData("Ball RGB", "r=%.2f g=%.2f b=%.2f", c.red, c.green, c.blue);
        telemetry.addData("Ball", ballColorValue);
    }

    // Match your revised TeleOp thresholds
    private void updateBallColor() {
        NormalizedRGBA c = ballColor.getNormalizedColors();

        float[] hsv = new float[3];
        Color.RGBToHSV(
                (int) (c.red * 255),
                (int) (c.green * 255),
                (int) (c.blue * 255),
                hsv
        );

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        boolean confident = (sat > 0.05) && (val > 0.009);

        if (confident && (hue >= 120 && hue <= 180)) {
            ballColorValue = "g";
        } else if (confident && (hue >= 210 && hue <= 255)) {
            ballColorValue = "p";
        } else {
            ballColorValue = null;
        }
    }

    private boolean colorSeen() {
        return ballColorValue != null && !ballColorValue.isEmpty();
    }

    // Slot detectors (mag0/mag1)
    private boolean atSlot0() { return !mag0.getState() && !mag1.getState(); }
    private boolean atSlot1() { return !mag0.getState() &&  mag1.getState(); }
    private boolean atSlot2() { return  mag0.getState() && !mag1.getState(); }

    // Shooter slot detectors (mag2/mag3)
    private boolean atShootSlot0() { return !mag2.getState() && !mag3.getState(); }
    private boolean atShootSlot1() { return !mag2.getState() &&  mag3.getState(); }
    private boolean atShootSlot2() { return  mag2.getState() && !mag3.getState(); }

    private void assignSlot(int slotNumber) {
        slots[slotNumber] = ballColorValue;
    }

    private void merryGoRoundIntake() {
        boolean seen = colorSeen();
        boolean newColorEvent = seen && !colorLatched;
        if (seen) colorLatched = true;
        else colorLatched = false;

        switch (intakeState) {
            case INIT_TO_SLOT0:
                servo1.setPower(1);
                if (atSlot0()) {
                    servo1.setPower(0);
                    intakeState = INTAKEState.WAIT_COLOR_0;
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
                servo1.setPower(1);
                if (atSlot1()) {
                    servo1.setPower(0);
                    intakeState = INTAKEState.WAIT_COLOR_1;
                }
                break;

            case WAIT_COLOR_1:
                servo1.setPower(0);
                if (newColorEvent) {
                    assignSlot(1);
                    intakeState = INTAKEState.MOVE_TO_SLOT2;
                }
                break;

            case MOVE_TO_SLOT2:
                servo1.setPower(1);
                if (atSlot2()) {
                    servo1.setPower(0);
                    intakeState = INTAKEState.WAIT_COLOR_2;
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
        telemetry.addData("BallColor", ballColorValue);
        telemetry.addData("Slots", "0=%s 1=%s 2=%s", slots[0], slots[1], slots[2]);
    }

    // -------------------- MANUAL fallback (TeleOp-aligned IMU field-centric) --------------------
    private void sticksManualTeleopStyle() {
        // same stick convention as your TeleOp
        double rsx = -gamepad1.right_stick_x;  // turn
        double lsy =  gamepad1.left_stick_y;   // forward/back
        double lsx =  gamepad1.left_stick_x;   // strafe

        driveByIntentFieldCentric(lsy, lsx, rsx);
    }

    private void driveByIntentFieldCentric(double lsy, double lsx, double rsx) {
        // Read yaw in degrees (ZYX) and convert once
        double yawDeg = imu1.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;

        double yawRad = Math.toRadians(yawDeg);

        // Rotate inputs by -yaw (field-centric)
        double drivePower  = lsy;
        double strafePower = lsx;

        double robotDrive  =  drivePower * Math.cos(yawRad) + strafePower * Math.sin(yawRad);
        double robotStrafe = -drivePower * Math.sin(yawRad) + strafePower * Math.cos(yawRad);

        // Feed into your normal mixer
        driveByIntent(robotDrive, robotStrafe, rsx);

        telemetry.addData("IMU yaw (deg)", "%.1f", yawDeg);
    }
    private int getAllianceTagId() {
        return (alliance == Alliance.RED) ? RED_TAG_ID : BLUE_TAG_ID;
    }
    private void buttonsManualServo2() {
        double now = getRuntime();
        double dt = now - lastServo2Time;
        lastServo2Time = now;
        dt = Math.max(0, Math.min(0.1, dt));

        if (gamepad2.dpad_up) {
            servo2Pos += SERVO2_RATE * dt;
        } else if (gamepad2.dpad_down) {
            servo2Pos -= SERVO2_RATE * dt;
        }

        servo2Pos = Math.max(SERVO2_MIN, Math.min(SERVO2_MAX, servo2Pos));
        servo2.setPosition(servo2Pos);
        telemetry.addData("Shooter Position", "%.3f", servo2.getPosition());
    }

    // -------------------- Misc --------------------
    private void stopDrive() {
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }

    private void getData() {
        double mps0 = motor0.getCurrentPosition();
        double mps1 = motor1.getCurrentPosition();
        double mps2 = motor2.getCurrentPosition();
        double mps3 = motor3.getCurrentPosition();

        double mpw0 = motor0.getPower();
        double mpw1 = motor1.getPower();
        double mpw2 = motor2.getPower();
        double mpw3 = motor3.getPower();

        double mv0 = ((DcMotorEx) motor0).getVelocity();
        double mv1 = ((DcMotorEx) motor1).getVelocity();
        double mv2 = ((DcMotorEx) motor2).getVelocity();
        double mv3 = ((DcMotorEx) motor3).getVelocity();

        telemetry.addData("M Pos", "(%.1f, %.1f, %.1f, %.1f)", mps0, mps1, mps2, mps3);
        telemetry.addData("M Power", "(%.1f, %.1f, %.1f, %.1f)", mpw0, mpw1, mpw2, mpw3);
        telemetry.addData("M Velocity", "(%.1f, %.1f, %.1f, %.1f)", mv0, mv1, mv2, mv3);
    }
}
