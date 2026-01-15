//Gamepad button layout:
//triangle = a
//circle = b
//cross = y
//square = x


//Controller 1:
//circle = leave auto align in case to april tag detected
//cross = align to april tag
//right stick = strafe
//left bumper = turbo
//left stick = turn + forward and backward

//Controller 2:
//triangle = dry fire release
//cross = dry fire servo 2
//dpad up + down = servo angle + or - respectively

//Magnet orientation (slot 0s are in position of entrance/exit)
//Intake:
//slots =  0  | 1  | 2
//mag 0 =  on | on | off
//mag 1 =  on | off| on
//Shooting:
//slots =  0  | 1  | 2
//mag 0 =  on | on | off
//mag 1 =  on | off| on

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Broadwater Robotics TeleOp")
public class broadwater_robotics_25_26_teleop extends LinearOpMode {

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
    private static final double ALIGN_KP = 0.03;
    private static final double ALIGN_TOLERANCE = 1.0;   // deg
    private static final double ALIGN_MAX_POWER = 0.3;
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

    // --- Dry fire mode ---
    private boolean dryFireMode = false;
    private boolean wasDryToggle = false;
    private boolean wasDryFire = false;

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

    // Merry-go-round state machine
    private enum INTAKEState { INIT_TO_SLOT0, WAIT_COLOR_0, MOVE_TO_SLOT1, WAIT_COLOR_1, MOVE_TO_SLOT2, WAIT_COLOR_2, DONE }
    private INTAKEState intakeState = INTAKEState.INIT_TO_SLOT0;

    private int currentSlot = 0;
    private boolean colorLatched = false; // edge-detect so one ball = one store

    // Marks which physical slot has already been fired
    private final boolean[] slotFired = new boolean[3];

    // Adjust if servo direction is backwards
    private static final double MGR_FAST_POWER   = 0.25;   // your main spin
    private static final double MGR_CRAWL_POWER  = 0.1;  // slow approach
    private static final double MGR_BACK_POWER   = -0.15; // slow reverse
    private static final long   MGR_BRAKE_MS     = 70;    // short reverse tap
    private static final double MGR_BRAKE_POWER  = -0.30; // brake tap power
    private static final double MGR_BACKUP_POWER   = -0.12;  // stronger than -0.10
    private static final long   MGR_BACKUP_1DEG_MS = 45;

    private static final long   MGR_LEAVE_SLOT_TIMEOUT_MS = 500;
    private static final long   MGR_REENTER_TIMEOUT_MS    = 700;

    // Safety so you don’t spin forever if a magnet fails
    private static final long MGR_MOVE_TIMEOUT_MS = 2000;

    float RSX;
    double YawValue;
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

    @Override
    public void runOpMode() {
        initLimelight();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

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

        servo0.setPosition(0);
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
        YawValue = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        if (ballColor instanceof SwitchableLight) {
            ((SwitchableLight)ballColor).enableLight(true);
        }

        mag0.setMode(DigitalChannel.Mode.INPUT);
        mag1.setMode(DigitalChannel.Mode.INPUT);
        mag2.setMode(DigitalChannel.Mode.INPUT);
        mag3.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {


                if (alignActive) {
                    telemetryLimeLight();  // only when aligning
                }
                // Press A to start auto-align & shoot
                if (!dryFireMode && gamepad1.a && !alignActive) {
                    alignActive = true;
                }
                if (alignActive) {
                    boolean aligned = alignToTarget();
                    if (aligned) {
                        adjustShooterAndFire();  // adjust and kick
                        alignActive = false;     // return to manual mode
                    } else if (gamepad1.b && alignActive) {
                        alignActive = false;
                    }
                } else {
                    sticks1();
                    buttons();   // manual controls
                }

                if (intakeState != INTAKEState.DONE) {
                    updateBallColor();
                }
                merryGoRoundIntake();
                // Optional: only show color telemetry while intaking
                if (intakeState != INTAKEState.DONE) {
                    telemetryBallColor();
                }
                motor0b.setPower(1);
                motor1b.setPower(1);
                motor2b.setPower(1);

                double v = laser.getVoltage();        // 0.0 to ~3.3V
                double mm = (v / 3.3) * 1000.0;       // 0–1000mm mapped to 0–3.3V
                double inches = mm / 25.4;

                telemetry.addData("Laser Dist", "%.0f mm  (%.1f in)", mm, inches);
                telemetryUpdateThrottled();
            }
        }
    }

    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    private long lastTelemMs = 0;
    private static final long TELEM_PERIOD_MS = 100; // 10Hz

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
        telemetryTagMotifs(result);
        updateLatchedMotif(result);
        telemetry.addData("Stored Tag Colors", "%s (Tag %d)", latchedMotif, latchedTagId);

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

    private boolean alignToTarget() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();

            if (Math.abs(tx) <= ALIGN_TOLERANCE) {
                stopDrive();
                telemetry.addData("Alignment", "Aligned!");
                return true;
            }

            double turnPower = Math.max(-ALIGN_MAX_POWER,
                    Math.min(ALIGN_MAX_POWER, tx * ALIGN_KP));
            LSX = (float) turnPower;
            sticks2();

            telemetry.addData("Aligning", "tx=%.2f  power=%.2f", tx, turnPower);
            return false;
        } else {
            telemetry.addData("Alignment", "No target detected");
            stopDrive();
            return false;
        }
    }

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
        telemetry.addData("Kicker Angle", "%.2f", servoAngle);
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
            kickOnce();
            slotFired[slotToShoot] = true;
            sleep(120);
        }

        telemetry.addLine("Shoot sequence done!");
    }

    private boolean atShootSlot(int slot) {
        switch (slot) {
            case 0: return atShootSlot0();
            case 1: return atShootSlot1();
            case 2: return atShootSlot2();
            default: return false;
        }
    }

    private int findSlotForColor(char wanted) {
        String w = String.valueOf(wanted);
        for (int i = 0; i < 3; i++) {
            if (!slotFired[i] && w.equals(slots[i])) return i;
        }
        return -1;
    }

    private boolean allSlotsLoaded() {
        return slots[0] != null && slots[1] != null && slots[2] != null;
    }

    private void rotateToSlotBlocking(int targetSlot) {
        shootingBusy = true;
        try {
            ensureKickerRetracted();

            long start = System.currentTimeMillis();

            // 1) Fast spin until we SEE the target slot
            servo1.setPower(MGR_FAST_POWER);
            while (opModeIsActive()
                    && !atShootSlot(targetSlot)
                    && (System.currentTimeMillis() - start) < MGR_MOVE_TIMEOUT_MS) {
                idle();
            }
            servo1.setPower(0);

            if (!atShootSlot(targetSlot)) return; // timeout safety

            // 2) Brake tap (optional but helps overshoot)
            servo1.setPower(MGR_BRAKE_POWER);
            sleep(MGR_BRAKE_MS);
            servo1.setPower(0);

            // 3) Creep forward a bit to settle into the slot pattern
            long creepStart = System.currentTimeMillis();
            servo1.setPower(MGR_CRAWL_POWER);
            while (opModeIsActive()
                    && !atShootSlot(targetSlot)
                    && (System.currentTimeMillis() - creepStart) < 300) {
                idle();
            }
            servo1.setPower(0);

            // 4) Tiny reverse nudge (your “1 degree back”)
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

    private void getData() {
        double mps0 = motor0.getCurrentPosition();
        double mps1 = motor1.getCurrentPosition();
        double mps2 = motor2.getCurrentPosition();
        double mps3 = motor3.getCurrentPosition();
        double mps0b = motor0b.getCurrentPosition();
        double mps1b = motor1b.getCurrentPosition();
        double mps2b = motor2b.getCurrentPosition();

        double mpw0 = motor0.getPower();
        double mpw1 = motor1.getPower();
        double mpw2 = motor2.getPower();
        double mpw3 = motor3.getPower();
        double mpw0b = motor0b.getPower();
        double mpw1b = motor1b.getPower();
        double mpw2b = motor2b.getPower();

        double mv0 = ((DcMotorEx) motor0).getVelocity();
        double mv1 = ((DcMotorEx) motor1).getVelocity();
        double mv2 = ((DcMotorEx) motor2).getVelocity();
        double mv3 = ((DcMotorEx) motor3).getVelocity();
        double mv0b = ((DcMotorEx) motor0b).getVelocity();
        double mv1b = ((DcMotorEx) motor1b).getVelocity();
        double mv2b = ((DcMotorEx) motor2b).getVelocity();

        telemetry.addData("M Pos", "(%.1f, %.1f, %.1f, %.1f)", mps0, mps1, mps2, mps3);
        telemetry.addData("M Power", "(%.1f, %.1f, %.1f, %.1f)", mpw0, mpw1, mpw2, mpw3);
        telemetry.addData("M Velocity", "(%.1f, %.1f, %.1f, %.1f)", mv0, mv1, mv2, mv3);
        telemetry.addData("Shoot Power", "(%.1f, %.1f)", mpw0b, mpw1b);
        telemetry.addData("Shoot Velocity", "(%.1f, %.1f)", mv0b, mv1b);
        telemetry.addData("Intake Power", "(%.1f)", mv2b);
    }

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

            int id;
            try {
                id = t.getFiducialId();
            } catch (Exception e) {
                telemetry.addLine("Tag ID method not found on FiducialResult");
                return;
            }

            String motif = decodeMotifFromTagId(id);
            telemetry.addData("April Tag Motif", motif);
        }
    }

    private void telemetryBallColor() {
        NormalizedRGBA c = ballColor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.RGBToHSV((int)(c.red*255), (int)(c.green*255), (int)(c.blue*255), hsv);

        telemetry.addData("BallHue", "%.0f", hsv[0]);
        telemetry.addData("Ball HSV", "H=%.0f S=%.2f V=%.2f", hsv[0], hsv[1], hsv[2]);
        telemetry.addData("Ball RGB", "r=%.2f g=%.2f b=%.2f", c.red, c.green, c.blue);
        telemetry.addData("Ball", ballColorValue );
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
    private boolean atShootSlot1() { return !mag2.getState() &&  !mag3.getState(); }
    private boolean atShootSlot2() { return  mag2.getState() && !mag3.getState(); }

    private void assignSlot(int slotNumber) {
        slots[slotNumber] = ballColorValue;
    }

    private void merryGoRoundIntake() {
        if (shootingBusy) {
            servo1.setPower(0); // do not fight shooter rotation
            return;
        }
        boolean seen = colorSeen();
        boolean newColorEvent = seen && !colorLatched;
        if (seen) colorLatched = true;
        else      colorLatched = false;

        switch (intakeState) {

            case INIT_TO_SLOT0:
                ensureKickerRetracted();
                servo1.setPower(MGR_FAST_POWER);
                if (atSlot0()) {
                    servo1.setPower(0);
                    currentSlot = 0;
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
                ensureKickerRetracted();
                servo1.setPower(MGR_FAST_POWER);
                if (atSlot1()) {
                    servo1.setPower(0);
                    currentSlot = 1;
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
                ensureKickerRetracted();
                servo1.setPower(MGR_FAST_POWER);
                if (atSlot2()) {
                    servo1.setPower(0);
                    currentSlot = 2;
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
            default:
                throw new IllegalStateException("Unexpected value: " + intakeState);
        }

        telemetry.addData("Intake State", intakeState);
        telemetry.addData("BallColor", ballColorValue);
        telemetry.addData("Slots", "0=%s 1=%s 2=%s", slots[0], slots[1], slots[2]);
    }
    private void kickOnce() {
        servo0.setPosition(KICK_EXTEND_POS);
        sleep(KICK_DURATION_MS);
        servo0.setPosition(KICK_RETRACT_POS);
    }

    private void ensureKickerRetracted() {
        servo0.setPosition(KICK_RETRACT_POS);
        sleep(60);
    }

    private boolean wasStepShootPressed = false;

    private int getCurrentShootSlot() {
        if (atShootSlot0()) return 0;
        if (atShootSlot1()) return 1;
        if (atShootSlot2()) return 2;
        return -1;
    }

    private void stepToNextSlotAndShoot() {
        ensureKickerRetracted();

        int current = getCurrentShootSlot();
        int next = (current >= 0) ? ((current + 1) % 3) : 0;

        telemetry.addData("StepShoot", "current=%d next=%d", current, next);
        telemetry.update();

        rotateToSlotBlocking(next);
        kickOnce();
    }

    private void buttons() {
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

        if (gamepad1.right_bumper) {
            getData();
        }
        if (gamepad2.y && !wasDryToggle) {
            dryFireMode = !dryFireMode;
        }
        wasDryToggle = gamepad2.y;

        telemetry.addData("DryFireMode", dryFireMode ? "ON" : "OFF");

        if (dryFireMode) {
            if (gamepad2.a && !wasDryFire) {
                kickOnce();
            }
            wasDryFire = gamepad2.a;
        }

        if (gamepad2.b && !wasStepShootPressed) {
            stepToNextSlotAndShoot();
        }
        wasStepShootPressed = gamepad2.b;
    }

    private void sticks2() {
        double gain;
        if (gamepad1.left_bumper) gain = 1;
        else gain = 0.5;

        strafePower = gain * LSX;
        drivePower = gain * LSY;
        rotatePower = gain * RSX;
        sticks4();
    }

    private void sticks1() {
        RSX = -gamepad1.right_stick_x;
        LSY = gamepad1.left_stick_x;
        LSX = gamepad1.left_stick_y;
        sticks2();
    }

    private void sticks4() {
        double yawDeg = imu1.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;

        double yawRad = Math.toRadians(yawDeg);

        double correctedDrive =  drivePower * Math.cos(yawRad) + strafePower * Math.sin(yawRad);
        double correctedStrafe = -drivePower * Math.sin(yawRad) + strafePower * Math.cos(yawRad);

        correctedDrivePower = correctedDrive;
        correctedStrafePower = correctedStrafe;

        drive2();
        telemetry.addData("IMU yaw (deg)", "%.1f", yawDeg);
    }

    private void drive2() {
        motor0.setPower((correctedDrivePower - correctedStrafePower) - rotatePower);
        motor2.setPower((correctedDrivePower + correctedStrafePower) + rotatePower);
        motor3.setPower((correctedDrivePower - correctedStrafePower) + rotatePower);
        motor1.setPower((correctedDrivePower + correctedStrafePower) - rotatePower);
    }

    private void lights(){
        if (gamepad2.a && !wasButtonAPressed) {
            isRedOn = !isRedOn;
            redLED.setState(isRedOn);
        }
        wasButtonAPressed = gamepad2.x;

        if (gamepad2.b && !wasButtonBPressed) {
            isBlueOn = !isBlueOn;
            blueLED.setState(isBlueOn);
        }
        wasButtonBPressed = gamepad2.y;
    }
}
