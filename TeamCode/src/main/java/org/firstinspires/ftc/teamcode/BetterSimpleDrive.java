package org.firstinspires.ftc.teamcode;


import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "BetterSimpleDrivea")
    public class BetterSimpleDrive extends LinearOpMode {

        // === Your existing hardware ===
        private DcMotor motor0;
        private DcMotor motor1;
        private DcMotor motor2;
        private DcMotor motor3;
        private DcMotor motor0b;
        private DcMotor motor1b;
        private DcMotor motor2b;
        private Servo servo0;       // gate / artifact stopper
        private CRServo servo1;     // shooter wheel
        private Servo servo2;
        private BNO055IMU imu1;
        private DigitalChannel blueLED;
        private DigitalChannel redLED;

        // === Your existing drive vars ===
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

        // === NEW: vision / AprilTag vars ===
        private VisionPortal visionPortal;
        private AprilTagProcessor aprilTagProcessor;
        private List<AprilTagDetection> currentDetections;

        // === NEW: control over auto-aim mode ===
        private boolean autoAimEnabled = false;

        @Override
        public void runOpMode() {

            // ---------------------
            // HARDWARE MAP
            // ---------------------
            motor0 = hardwareMap.get(DcMotor.class, "motor0");
            motor1 = hardwareMap.get(DcMotor.class, "motor1");
            motor2 = hardwareMap.get(DcMotor.class, "motor2");
            motor3 = hardwareMap.get(DcMotor.class, "motor3");
            motor0 = hardwareMap.get(DcMotor.class, "motor0b");
            motor1 = hardwareMap.get(DcMotor.class, "motor1b");
            motor2 = hardwareMap.get(DcMotor.class, "motor2b");

            servo0 = hardwareMap.get(Servo.class, "servo0");
            servo1 = hardwareMap.get(CRServo.class, "servo1");
            servo0 = hardwareMap.get(Servo.class, "servo2");
            imu1   = hardwareMap.get(BNO055IMU.class, "imu 1");

            // If you actually have LEDs, uncomment these:
            // blueLED = hardwareMap.get(DigitalChannel.class, "blueLED");
            // redLED  = hardwareMap.get(DigitalChannel.class, "redLED");

            // ---------------------
            // MOTOR DIRECTIONS
            // ---------------------
            // This matches what you had: mecanum-ish layout
            motor0.setDirection(DcMotor.Direction.FORWARD);   // front right
            motor1.setDirection(DcMotor.Direction.REVERSE);   // back left
            motor2.setDirection(DcMotor.Direction.REVERSE);   // front left
            motor3.setDirection(DcMotor.Direction.FORWARD);   // back right

            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Shooter defaults
            servo0.setPosition(0);                      // gate closed
            servo1.setDirection(DcMotorSimple.Direction.FORWARD);

            // ---------------------
            // IMU SETUP
            // ---------------------
            BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
            imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imuParameters.loggingEnabled = false;
            imu1.initialize(imuParameters);

            // ---------------------
            // NEW: APRILTAG SETUP
            // ---------------------
            initAprilTag();

            telemetry.addLine("Initialized. Press PLAY.");
            telemetry.update();

            waitForStart();

            // ---------------------
            // MAIN LOOP
            // ---------------------
            while (opModeIsActive()) {
                getData();       // your telemetry on motors
                sticks1();       // read joysticks -> drive
                buttons();       // your existing button logic

                // Toggle auto-aim with gamepad2 left bumper (hold = on)
                autoAimEnabled = gamepad2.left_bumper;

                if (autoAimEnabled) {
                    detectAndShootWithAprilTags();
                }

                telemetry.addData("AutoAim", autoAimEnabled ? "ON" : "OFF");
                telemetry.update();
            }
        }

        // =============================================================
        // NEW: initialize the VisionPortal and AprilTag processor
        // =============================================================
        private void initAprilTag() {
            aprilTagProcessor = new AprilTagProcessor.Builder().build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // must match your config name
                    .addProcessor(aprilTagProcessor)
                    .setCameraResolution(new Size(1280, 800))
                    .build();

            telemetry.addLine("AprilTag vision initialized");
        }

        // =============================================================
        // NEW: detect tag, align, and shoot
        // =============================================================
        private void detectAndShootWithAprilTags() {
            currentDetections = aprilTagProcessor.getDetections();

            if (currentDetections.size() > 0) {
                // Just take the first detection for now
                AprilTagDetection tag = currentDetections.get(0);

                double yaw   = tag.ftcPose.yaw;     // degrees left/right
                double range = tag.ftcPose.range;   // inches

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Tag Yaw", yaw);
                telemetry.addData("Tag Range", range);

                // 1) turn toward the tag
                if (Math.abs(yaw) > 3) {  // need to turn
                    double turnPower = yaw / 40.0;  // scale it down
                    motor0.setPower(-turnPower);    // FR
                    motor3.setPower(-turnPower);    // BR
                    motor1.setPower(turnPower);     // BL
                    motor2.setPower(turnPower);     // FL
                }
                // 2) if we're facing it, check distance
                else if (range > 20 && range < 35) {
                    // Good distance -> shoot
                    stopAllDriveMotors();
                    fireShooterQuick();
                } else if (range >= 35) {
                    // too far, creep forward
                    driveForward(0.25);
                } else if (range <= 20) {
                    // too close, back up a bit
                    driveForward(-0.25);
                }

            } else {
                telemetry.addLine("No AprilTag detected");
            }
        }

        // =============================================================
        // NEW: shooter action (using your servo0 + servo1)
        // =============================================================
        private void fireShooterQuick() {
            // open gate
            servo0.setPosition(1.0);
            // spin shooter
            servo1.setPower(1.0);
            sleep(300);          // let one ring/artifact through
            // close gate
            servo0.setPosition(0.0);
            servo1.setPower(0.0);
        }

        // =============================================================
        // NEW: drive helpers
        // =============================================================
        private void stopAllDriveMotors() {
            motor0.setPower(0);
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
        }

        private void driveForward(double power) {
            motor0.setPower(power);  // FR
            motor3.setPower(power);  // BR
            motor2.setPower(power);  // FL
            motor1.setPower(power);  // BL
        }

        // =============================================================
        // YOUR EXISTING METHODS (kept almost as-is)
        // =============================================================
        private void getData() {
            telemetry.addData("Motor 0 Pos", motor0.getCurrentPosition());
            telemetry.addData("Motor 1 Pos", motor1.getCurrentPosition());
            telemetry.addData("motor 2 Pos", motor2.getCurrentPosition());
            telemetry.addData("motor 3 Pos", motor3.getCurrentPosition());
            telemetry.addData("powerMotor0", motor0.getPower());
            telemetry.addData("powerMotor1", motor1.getPower());
            telemetry.addData("powerMotor2", motor2.getPower());
            telemetry.addData("powerMotor3", motor3.getPower());
            telemetry.addData("VelMotor0", ((DcMotorEx) motor0).getVelocity());
            telemetry.addData("VelMotor1", ((DcMotorEx) motor1).getVelocity());
            telemetry.addData("VelMotor2", ((DcMotorEx) motor2).getVelocity());
            telemetry.addData("VelMotor3", ((DcMotorEx) motor3).getVelocity());
        }

        private void buttons() {
            // your dpad stuff commented out in original
            if (gamepad2.right_bumper) {
                servo0.setPosition(1);
            } else {
                servo0.setPosition(0);
            }

            if (gamepad2.x) {
                servo1.setPower(1.0);
            } else if (gamepad2.y) {
                servo1.setPower(-1.0);
            } else {
                servo1.setPower(0);
            }
        }

        private void sticks1() {
            RSX = gamepad1.right_stick_x;
            LSY = gamepad1.left_stick_y;
            LSX = gamepad1.left_stick_x;
            sticks2();
        }

        private void sticks2() {
            double gain;
            if (gamepad1.left_bumper) {
                gain = 1.0;
            } else {
                gain = 0.5;
            }
            strafePower = gain * LSX;
            drivePower = gain * LSY;
            rotatePower = gain * RSX;
            sticks4();
        }

        // IMU correction
        private void sticks4() {
            // get yaw in radians
            double imuYawDeg = imu1.getAngularOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES
            ).thirdAngle;

            // original math was a bit redundant, this is enough:
            double yawRad = Math.toRadians(imuYawDeg);

            correctedStrafePower = strafePower * Math.cos(yawRad) - drivePower * Math.sin(yawRad);
            correctedDrivePower  = drivePower  * Math.cos(yawRad) + strafePower * Math.sin(yawRad);

            drive2();
        }

        // send to motors
        private void drive2() {
            // Front right motor (motor0)
            motor0.setPower((correctedDrivePower - correctedStrafePower) - rotatePower);
            // Front left motor (motor2)
            motor2.setPower(correctedDrivePower + correctedStrafePower + rotatePower);
            // Back right motor (motor3)
            motor3.setPower((correctedDrivePower - correctedStrafePower) + rotatePower);
            // Back left motor (motor1)
            motor1.setPower((correctedDrivePower + correctedStrafePower) - rotatePower);
        }


}
