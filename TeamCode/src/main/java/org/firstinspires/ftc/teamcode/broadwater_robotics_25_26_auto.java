package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "Broadwater Auto: 1m + Turn35 + SpinUp")
public class broadwater_robotics_25_26_auto extends LinearOpMode {

    // ---------------- Hardware ----------------
    private DcMotor motor0; // Drive FR
    private DcMotor motor1; // Drive BL
    private DcMotor motor2; // Drive FL
    private DcMotor motor3; // Drive BR

    private DcMotor motor0b; // Shooter 1
    private DcMotor motor1b; // Shooter 2
    private DcMotor motor2b; // Intake

    private Servo servo0;    // Kicker
    private CRServo servo1;  // Merry Go Round Tray
    private Servo servo2;    // Shooter Angle

    private BNO055IMU imu1;

    private DigitalChannel redLED;
    private DigitalChannel blueLED;

    private AnalogInput laser;

    private DigitalChannel mag0; // Intake Magnet 0
    private DigitalChannel mag1; // Intake Magnet 1
    private DigitalChannel mag2; // Shooter Magnet 2
    private DigitalChannel mag3; // Shooter Magnet 3

    private NormalizedColorSensor ballColor;

    // (Optional) Limelight (not used in this simple routine, but kept so your config still matches)
    private Limelight3A limelight;

    // ---------------- Tunables ----------------
    // Wheel + encoder conversion (UPDATE these for your robot!)
    // goBILDA 96mm mecanum: diameter ≈ 0.096m, circumference ≈ 0.3016m
    private static final double WHEEL_DIAMETER_M = 0.096;
    private static final double WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M;

    // Put YOUR real ticks per wheel revolution here:
    // Examples many teams use: 537.7 (19.2:1), 383.6 (13.7:1), 312 (11:1)
    private static final double TICKS_PER_WHEEL_REV = 537.7;

    private static final double DRIVE_MIN_POWER = 0.12;

    // Shooter / intake powers
    private static final double SHOOTER_POWER = 0.6;
    private static final double INTAKE_POWER  = 1.0;

    // Kicker (not used in this routine, but left for consistency)
    private static final double KICK_RETRACT_POS = 1.0;

    @Override
    public void runOpMode() {
        initHardware();
        initLimelight(); // optional — uncomment if you want Limelight running in auto

        telemetry.addLine("Auto ready.");
        telemetry.addLine("Will: spin shooters+intake, drive 1m, turn right 35deg");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // ---- Spin up shooter + intake ----
        motor0b.setPower(SHOOTER_POWER);
        motor1b.setPower(SHOOTER_POWER);
        motor2b.setPower(INTAKE_POWER);

        // ---- Drive forward 1 meter ----
        driveForwardMeters(-1.0, 1);

        // ---- Turn right 35 degrees ----
        turnRightDegrees(-135.0, 0.25);

        // (Optional) keep running shooter/intake, or stop them:
        // motor0b.setPower(0);
        // motor1b.setPower(0);
        // motor2b.setPower(0);


        stopDrive();
    }

    // ---------------- Init ----------------
    private void initHardware() {
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

        mag0 = hardwareMap.get(DigitalChannel.class, "mag0");
        mag1 = hardwareMap.get(DigitalChannel.class, "mag1");
        mag2 = hardwareMap.get(DigitalChannel.class, "mag2");
        mag3 = hardwareMap.get(DigitalChannel.class, "mag3");

        redLED  = hardwareMap.get(DigitalChannel.class, "redLED");
        blueLED = hardwareMap.get(DigitalChannel.class, "blueLED");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        blueLED.setMode(DigitalChannel.Mode.OUTPUT);

        imu1 = hardwareMap.get(BNO055IMU.class, "imu 1");

        ballColor = hardwareMap.get(NormalizedColorSensor.class, "ballColor");

        // Drive directions (kept from your TeleOp)
        motor0.setDirection(DcMotor.Direction.FORWARD);
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);

        // Shooter/intake directions (kept from your TeleOp)
        motor0b.setDirection(DcMotor.Direction.REVERSE);
        motor1b.setDirection(DcMotor.Direction.FORWARD);
        motor2b.setDirection(DcMotorSimple.Direction.FORWARD);

        // Tray direction
        servo1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Kicker safe
        servo0.setPosition(KICK_RETRACT_POS);

        // IMU init
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

        // Color sensor light
        if (ballColor instanceof SwitchableLight) {
            ((SwitchableLight) ballColor).enableLight(true);
        }

        // Magnet inputs
        mag0.setMode(DigitalChannel.Mode.INPUT);
        mag1.setMode(DigitalChannel.Mode.INPUT);
        mag2.setMode(DigitalChannel.Mode.INPUT);
        mag3.setMode(DigitalChannel.Mode.INPUT);

        // Make sure drive motors are in a known mode
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // For better stopping
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Optional
    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(50);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // ---------------- Movement: Forward (encoders) ----------------
    private void driveForwardMeters(double meters, double power) {
        int ticks = (int) Math.round((meters / WHEEL_CIRCUMFERENCE_M) * TICKS_PER_WHEEL_REV);

        int startFR = motor0.getCurrentPosition();
        int startBL = motor1.getCurrentPosition();
        int startFL = motor2.getCurrentPosition();
        int startBR = motor3.getCurrentPosition();

        int targetFR = startFR + ticks;
        int targetBL = startBL + ticks;
        int targetFL = startFL + ticks;
        int targetBR = startBR + ticks;

        setDriveTarget(targetFR, targetBL, targetFL, targetBR);

        double p = Math.max(DRIVE_MIN_POWER, Math.abs(power));
        setDriveRunToPositionPower(p, p, p, p);

        long start = System.currentTimeMillis();
        long timeoutMs = 6000; // increase if needed

        while (opModeIsActive()
                && (System.currentTimeMillis() - start) < timeoutMs
                && driveBusy()) {

            telemetry.addData("Forward", "%.2fm ticks=%d", meters, ticks);
            telemetry.addData("FR/FL", "%d / %d", motor0.getCurrentPosition(), motor2.getCurrentPosition());
            telemetry.addData("IMU yaw", "%.1f", getYawDeg());
            telemetry.update();
            idle();
        }

        stopDrive();
        setDriveRunUsingEncoder();
    }

    // ---------------- Movement: Turn right (IMU) ----------------
    private void turnRightDegrees(double degrees, double maxPower) {
        // Read start yaw
        double startYaw = getYawDeg();

        // Most robots: right turn = yaw decreases. If yours is opposite, flip sign below.
        double targetYaw = angleWrapDeg(startYaw - degrees);

        // Tunables
        final double TOL_DEG = 2.0;     // stop window
        final double MIN_PWR = 0.10;    // overcome friction
        final double KP      = 0.012;   // turn strength (start here)
        final long   TIMEOUT = 4000;

        double pMax = Math.max(MIN_PWR, Math.abs(maxPower));

        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < TIMEOUT) {
            double yaw = getYawDeg();
            double err = angleWrapDeg(targetYaw - yaw);  // wrap-safe error in [-180,180]

            telemetry.addData("Turn", "start=%.1f target=%.1f yaw=%.1f err=%.1f",
                    startYaw, targetYaw, yaw, err);
            telemetry.update();

            if (Math.abs(err) <= TOL_DEG) break;

            // Proportional control with min power + clamp
            double cmd = err * KP;

            // clamp to max
            cmd = clip(cmd, -pMax, pMax);

            // enforce min power if we're not close yet
            if (Math.abs(cmd) < MIN_PWR) cmd = MIN_PWR * Math.signum(cmd);

            // slow down when close (prevents overshoot)
            if (Math.abs(err) < 10.0) {
                cmd = clip(cmd, -0.18, 0.18);
            }

            driveRobotCentric(0, 0, cmd);
            idle();
        }

        stopDrive();

        // Small settle
        sleep(120);
    }


    // ---------------- Drive helpers ----------------
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

    private void stopDrive() {
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }

    private void setDriveTarget(int fr, int bl, int fl, int br) {
        motor0.setTargetPosition(fr);
        motor1.setTargetPosition(bl);
        motor2.setTargetPosition(fl);
        motor3.setTargetPosition(br);

        motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setDriveRunToPositionPower(double fr, double bl, double fl, double br) {
        motor0.setPower(fr);
        motor1.setPower(bl);
        motor2.setPower(fl);
        motor3.setPower(br);
    }

    private boolean driveBusy() {
        return motor0.isBusy() || motor1.isBusy() || motor2.isBusy() || motor3.isBusy();
    }

    private void setDriveRunUsingEncoder() {
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ---------------- IMU helpers ----------------
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

}
