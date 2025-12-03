package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//@Autonomous(name = "KaysCode")
public class KaysCode extends LinearOpMode{
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor0b;
    private DcMotor motor1b;
    private DcMotor motor2b;
    private Servo servo0;
    private CRServo servo1;
    private Servo servo2;
    private BNO055IMU imu1;
    private DigitalChannel blueLED;
    private DigitalChannel redLED;
    //private DcMotor motor2b;
    // Limelight alignment control
    private static final double ALIGN_KP = 0.03;
    private static final double ALIGN_TOLERANCE = 1.0;   // deg
    private static final double ALIGN_MAX_POWER = 0.3;
    private boolean alignActive = false;

    // Shooter control constants
    private static final double SHOOTER_BASE_POWER = 0.5;
    private static final double SHOOTER_MAX_POWER  = 1.0;
    private static final double SHOOTER_MIN_DIST   = 20.0;  // inches
    private static final double SHOOTER_MAX_DIST   = 70.0;  // inches

    // Servo angle limits
    private static final double SERVO_MIN_ANGLE = 0.25;
    private static final double SERVO_MAX_ANGLE = 0.85;

    // Firing servo timing
    private static final double KICK_EXTEND_POS = 1.0;
    private static final double KICK_RETRACT_POS = 0.0;
    private static final long   KICK_DURATION_MS = 350;  // how long to stay extended


    float RSX;
    double YawValue;
    float LSY;
    double correctedStrafePower;
    double strafePower;
    float LSX;
    double correctedDrivePower;
    double drivePower;
    double rotatePower;
    private boolean wasButtonAPressed = false;
    private boolean wasButtonBPressed = false;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        BNO055IMU.Parameters imuParameters;



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
        imu1 = hardwareMap.get(BNO055IMU.class, "imu 1");

        double getHeading; {
            return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double inches,double power

        private void driveForwardGyro;
            {
            double COUNTS_PER_REV = 537.7;       // GoBilda 312 RPM, change if needed
            double WHEEL_DIAMETER = 4.0;         // your wheel size in inches
            double COUNTS_PER_INCH = COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER);

            int moveCounts = (int)(inches * COUNTS_PER_INCH);

            // Reset encoders
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set target encoder positions
            motor2.setTargetPosition(moveCounts);
            motor1.setTargetPosition(moveCounts);
            motor0.setTargetPosition(moveCounts);
            motor3.setTargetPosition(moveCounts);

            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Save the starting heading
            double startHeading = getHeading();

            // Start driving
            motor2.setPower(power);
            motor1.setPower(power);
            motor0.setPower(power);
            motor3.setPower(power);

            while (opModeIsActive() &&
                    (motor2.isBusy() || motor0.isBusy() ||
                            motor1.isBusy()  || motor3.isBusy())) {

                double currentHeading = getHeading();
                double error = currentHeading - startHeading;
                double kP = 0.03;    // Tune this if needed

                double correction = error * kP;

                // Apply correction (add to left, subtract from right)
                motor2.setPower(power + correction);
                motor1.setPower(power + correction);
                motor0.setPower(power - correction);
                motor3.setPower(power - correction);

                telemetry.addData("Heading", currentHeading);
                telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motors
            motor2.setPower(0);
            motor1.setPower(0);
            motor0.setPower(0);
            motor3.setPower(0);

            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }




        while (opModeIsActive()) {

           LLResult result = limelight.getLatestResult();

         // If we do NOT see any tag -> keep turning left slowly
          if (result == null || !result.isValid()) {
              motor2.setPower(-0.15);
              motor1.setPower(-0.15);
              motor0.setPower(0.15);
              motor3.setPower(0.15);
        }
        // If we DO see a tag -> STOP turning
        else {
            motor2.setPower(0);
            motor1.setPower(0);
            motor0.setPower(0);
            motor3.setPower(0);
        }







    }

    }

    public void getHeading() {
        return heading;
    }
        private void alignToAprilTag() {

            while (opModeIsActive()) {

                LLResult result = limelight.getLatestResult();

                // If no tag → keep turning right slowly looking for center
                if (result == null || !result.isValid()) {
                    leftFront.setPower(0.15);
                    leftRear.setPower(0.15);
                    rightFront.setPower(-0.15);
                    rightRear.setPower(-0.15);
                    telemetry.addLine("Tag not found - sweeping right...");
                    telemetry.update();
                    continue;
                }

                double tx = result.getTx();   // horizontal offset

                telemetry.addData("tx", tx);
                telemetry.update();

                // Alignment tolerance (how perfect you want it)
                double tolerance = 1.0;  // degrees

                // If centered → stop
                if (Math.abs(tx) < tolerance) {
                    stopDrive();
                    return;
                }

                // If tag is left → turn right to center it
                if (tx > 0) {
                    leftFront.setPower(0.12);
                    leftRear.setPower(0.12);
                    rightFront.setPower(-0.12);
                    rightRear.setPower(-0.12);
                }
                // If tag is right → turn left to center it
                else {
                    leftFront.setPower(-0.12);
                    leftRear.setPower(-0.12);
                    rightFront.setPower(0.12);
                    rightRear.setPower(0.12);
                    private void strafeRightGyro(double inches, double power) {
                        double COUNTS_PER_REV = 537.7;          // adjust for your motor
                        double WHEEL_DIAMETER = 4.0;            // inches
                        double COUNTS_PER_INCH = COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER);

                        int moveCounts = (int)(inches * COUNTS_PER_INCH);

                        // RESET ENCODOERS
                        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        // STRAFE RIGHT target positions
                        leftFront.setTargetPosition(+moveCounts);
                        leftRear.setTargetPosition(-moveCounts);
                        rightFront.setTargetPosition(-moveCounts);
                        rightRear.setTargetPosition(+moveCounts);

                        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        double startHeading = getHeading();     // save yaw heading to hold

                        // Start moving
                        leftFront.setPower(power);
                        leftRear.setPower(power);
                        rightFront.setPower(power);
                        rightRear.setPower(power);

                        while (opModeIsActive() &&
                                (leftFront.isBusy() || rightFront.isBusy() ||
                                        leftRear.isBusy()  || rightRear.isBusy())) {

                            // Get current IMU heading
                            double heading = getHeading();
                            double error = heading - startHeading;
                            double kP = 0.03;     // tune if needed

                            double correction = error * kP;

                            // Apply correction to keep straight
                            leftFront.setPower(power + correction);
                            leftRear.setPower(power - correction);
                            rightFront.setPower(power - correction);
                            rightRear.setPower(power + correction);

                            telemetry.addData("Heading", heading);
                            telemetry.addData("Correction", correction);
                            telemetry.update();
                        }

                        // STOP
                        leftFront.setPower(0);
                        leftRear.setPower(0);
                        rightFront.setPower(0);
                        rightRear.setPower(0);

                        // Return motors to normal mode
                        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        private void turnRightDegrees(double degrees, double power) {

                            double target = getHeading() + degrees;

                            // Wrap target angle into -180 to +180 range
                            if (target > 180) target -= 360;

                            while (opModeIsActive()) {

                                double heading = getHeading();
                                double error = target - heading;

                                // Angle wrap (keeps error between -180 and 180)
                                if (error > 180) error -= 360;
                                if (error < -180) error += 360;

                                // Stop when close enough
                                if (Math.abs(error) < 1.0) {
                                    stopDrive();
                                    return;
                                }

                                double turnPower = power;

                                // Turn RIGHT = left side forward, right side backward
                                leftFront.setPower(+turnPower);
                                leftRear.setPower(+turnPower);
                                rightFront.setPower(-turnPower);
                                rightRear.setPower(-turnPower);

                                telemetry.addData("Heading", heading);
                                telemetry.addData("Target", target);
                                telemetry.addData("Error", error);
                                telemetry.update();
                            }
                        }