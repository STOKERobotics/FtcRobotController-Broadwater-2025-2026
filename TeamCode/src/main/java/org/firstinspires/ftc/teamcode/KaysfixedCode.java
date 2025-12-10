package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "KaysCode")
public class KaysfixedCode extends LinearOpMode {

    // Motors
    private DcMotor motor0, motor1, motor2, motor3;
    private DcMotor motor0b, motor1b, motor2b;

    // Servos
    private Servo servo0, servo2;
    private CRServo servo1;

    // IMU
    private BNO055IMU imu1;

    // Limelight
    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        // ---------------------------
        // HARDWARE INIT
        // ---------------------------
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

        imu1 = hardwareMap.get(BNO055IMU.class, "imu 1");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // IMU SETTINGS
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu1.initialize(params);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        // ---------------------------
        // AUTONOMOUS BEGINS
        // ---------------------------

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            // If NO tag, rotate left slowly
            if (result == null || !result.isValid()) {
                motor0.setPower(0.15);
                motor1.setPower(0.15);
                motor2.setPower(-0.15);
                motor3.setPower(-0.15);

                telemetry.addLine("NO TAG — Turning left...");
            }

            // If tag found → stop turning
            else {
                motor0.setPower(0);
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);

                telemetry.addLine("TAG FOUND — Stopping turn.");

                // 🔥 DRIVE FORWARD 68 INCHES GYRO STRAIGHT
                driveForwardGyro(68, 0.5);

                break; // stop autonomous
            }

            telemetry.update();
        }

    }

    // -----------------------------------------------------
    // IMU Heading Function
    // -----------------------------------------------------
    private double getHeading() {
        return imu1.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;
    }

    // -----------------------------------------------------
    // GYRO STRAIGHT DRIVE
    // -----------------------------------------------------
    private void driveForwardGyro(double inches, double power) {

        double COUNTS_PER_REV   = 537.7;    // goBILDA 312 RPM
        double WHEEL_DIAMETER   = 4.0;
        double COUNTS_PER_INCH  = COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER);

        int moveCounts = (int)(inches * COUNTS_PER_INCH);

        // RESET ENCODERS
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor0.setTargetPosition(moveCounts);
        motor1.setTargetPosition(moveCounts);
        motor2.setTargetPosition(moveCounts);
        motor3.setTargetPosition(moveCounts);

        motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double startHeading = getHeading();

        // Start driving
        motor0.setPower(power);
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);

        while (opModeIsActive() &&
                (motor0.isBusy() || motor1.isBusy() || motor2.isBusy() || motor3.isBusy())) {

            double currentHeading = getHeading();
            double error = currentHeading - startHeading;
            double kP = 0.03;

            double correction = error * kP;

            // Left motors = +correction
            // Right motors = -correction
            motor0.setPower(power - correction);
            motor1.setPower(power - correction);
            motor2.setPower(power + correction);
            motor3.setPower(power + correction);

            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        // Stop
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);

        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
