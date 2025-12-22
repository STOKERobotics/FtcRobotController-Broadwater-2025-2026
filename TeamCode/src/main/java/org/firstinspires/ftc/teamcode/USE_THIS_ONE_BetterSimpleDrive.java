package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;


@TeleOp(name = "USE THIS ONE BetterSimpleDrive")
public class USE_THIS_ONE_BetterSimpleDrive extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */


    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    private DcMotor motor0;
    private DcMotor motor1;`
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
    
    private NormalizedColorSensor ballColor;
    private boolean ballIsGreen = false;
    private boolean ballIsPurple = false;
    
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

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        initLimelight();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

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
        ballColor = hardwareMap.get(NormalizedColorSensor.class, "ballColor"); // match config name
        dist = hardwareMap.get(NormalizedColorSensor.class, "ballColor"); // match config name
        //blueLED = hardwareMap.get(DigitalChannel.class, "blueLED");
        //redLED = hardwareMap.get(DigitalChannel.class, "redLED");

        // Put initialization blocks here.
        motor0.setDirection(DcMotor.Direction.FORWARD);
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setDirection(DcMotor.Direction.FORWARD);
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
        servo1.setDirection(DcMotorSimple.Direction.FORWARD);

        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu1.initialize(imuParameters);
        YawValue = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        //blueLED.setMode(DigitalChannel.Mode.OUTPUT);
        //redLED.setMode(DigitalChannel.Mode.OUTPUT);

        if (ballColor instanceof SwitchableLight) {
            ((SwitchableLight)ballColor).enableLight(true);
        }

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {

                getData();
                telemetryLimeLight();
                // Press A to start auto-align & shoot
                if (gamepad1.a && !alignActive) {
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
                }
                else {
                    sticks1();
                    sticks2();
                    sticks4();// normal drive
                    buttons();   // manual controls
                }
                updateBallColor();
                telemetryBallColor();
                motor0b.setPower(1);
                motor1b.setPower(1);
                motor2b.setPower(1);
                telemetry.update();
            }
        }

    }

    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetryLimeLight();

    }   // end method initLimelight()


    private void telemetryLimeLight() {
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            telemetry.addLine("LLResult = null");
            return;
        }
        Pose3D targetCam = getBestTagPoseCameraSpace(result);
        if (targetCam != null) {
            telemetry.addData("TagCam x z (m)", "%.2f %.2f",
                    targetCam.getPosition().x,
                    targetCam.getPosition().z);
            telemetry.addData("TagDist (in)", "%.1f",
                    getWallTagDistanceInchesFromCameraPose(targetCam));
            telemetry.addData("Tag z only (in)", "%.1f", targetCam.getPosition().z * 39.37);
            telemetry.addData("Tag hypotenuse (in)", "%.1f", Math.hypot(
                    targetCam.getPosition().x, targetCam.getPosition().z) * 39.37);
        } else {
            telemetry.addLine("TagCam: none");
        }
        //telemetry.addData("LL Valid", result.isValid());
        telemetry.addData("tx", "%.2f", result.getTx());
        telemetry.addData("ty", "%.2f", result.getTy());
        telemetry.addData("ta", "%.2f", result.getTa());

        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            telemetry.addLine("botpose = null (localization not producing pose)");
            return;
        }

        double x = botpose.getPosition().x;
        double y = botpose.getPosition().y;
        double z = botpose.getPosition().z;

        telemetry.addData("Botpose x,y,z (m)", "(%.3f, %.3f, %.3f)", x, y, z);
        //telemetry.addData("Dist from origin (in)", "%.1f", Math.sqrt(x*x + y*y) * 39.37);
    }



    private boolean alignToTarget() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();  // horizontal offset

            if (Math.abs(tx) <= ALIGN_TOLERANCE) {
                stopDrive();
                telemetry.addData("Alignment", "Aligned!");
                return true;
            }

            double turnPower = Math.max(-ALIGN_MAX_POWER,
                    Math.min(ALIGN_MAX_POWER, tx * ALIGN_KP));
            LSX = (float) turnPower;
            sticks2();
            // Rotate robot
//            motor0.setPower(-turnPower);
//            motor1.setPower(turnPower);
//            motor2.setPower(turnPower);
//            motor3.setPower(-turnPower);

            telemetry.addData("Aligning", "tx=%.2f  power=%.2f", tx, turnPower);
            return false;
        } else {
            telemetry.addData("Alignment", "No target detected");
            stopDrive();
            return false;
        }
    }
    private void adjustShooterAndFire() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getBotpose() != null) {

            Pose3D botpose = result.getBotpose();
            double distanceMeters = Math.sqrt(
                    botpose.getPosition().x * botpose.getPosition().x +
                    botpose.getPosition().y * botpose.getPosition().y);
            double distanceInches = distanceMeters * 39.37;

            // Normalize to 0–1 range
            double normalized = Math.max(0, Math.min(1,
                    (distanceInches - SHOOTER_MIN_DIST) /
                    (SHOOTER_MAX_DIST - SHOOTER_MIN_DIST)));

            double shooterPower = SHOOTER_BASE_POWER +
                    (SHOOTER_MAX_POWER - SHOOTER_BASE_POWER) * normalized;
            double servoAngle = SERVO_MAX_ANGLE -
                    (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) * normalized;


            servo2.setPosition(servoAngle);

            telemetry.addData("Servo2 Angle", "%.2f", servoAngle);

            // Kick the ball
            servo2.setPosition(KICK_EXTEND_POS);
            sleep(KICK_DURATION_MS);
            servo2.setPosition(KICK_RETRACT_POS);
            telemetry.addLine("Ball Fired!");
        } else {
            telemetry.addLine("Shooter: No tag or botpose");
        }
    }
    private void stopDrive() {
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }
    public double getDistanceFromTag(Pose3D pose){
        if (pose == null) return -1; // or Double.NaN
    
        // Planar distance = sqrt(x^2 + z^2)  (meters)
        double x = pose.getPosition().x;
        double z = pose.getPosition().z;
    
        double distanceMeters = Math.sqrt(x * x + z * z);
        return distanceMeters * 39.37; // return inches (to match your shooter constants)
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

        /*
        telemetry.addData("Motor 0 Pos", motor0.getCurrentPosition());
        telemetry.addData("Motor 1 Pos", motor1.getCurrentPosition());
        telemetry.addData("motor 2 Pos", motor2.getCurrentPosition());
        telemetry.addData("motor 3 Pos", motor3.getCurrentPosition());
        telemetry.addData("powerMotor0", motor0.getPower());
        telemetry.addData("powerMotor1", motor1.getPower());
        telemetry.addData("powerMotor2", motor2.getPower());
        telemetry.addData("powerMotor3", motor3.getPower());
        telemetry.addData("powerMotor0b", motor0b.getPower());
        telemetry.addData("powerMotor1b", motor1b.getPower());
        telemetry.addData("VelMotor0", ((DcMotorEx) motor0).getVelocity());
        telemetry.addData("VelMotor1", ((DcMotorEx) motor1).getVelocity());
        telemetry.addData("VelMotor2", ((DcMotorEx) motor2).getVelocity());
        telemetry.addData("VelMotor3", ((DcMotorEx) motor3).getVelocity());
        */


    }

    private void telemetryBallColor() {
        NormalizedRGBA c = ballColor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.RGBToHSV((int)(c.red*255), (int)(c.green*255), (int)(c.blue*255), hsv);
    
        telemetry.addData("BallHue", "%.0f", hsv[0]);
        telemetry.addData("Ball", ballIsGreen ? "GREEN" : (ballIsPurple ? "PURPLE" : "UNKNOWN"));
    }

    private Pose3D getBestTagPoseCameraSpace(LLResult result) {
        if (result == null || !result.isValid()) return null;
    
        java.util.List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;
    
        // Pick the "best" tag: easiest heuristic = largest target area
        LLResultTypes.FiducialResult best = tags.get(0);
        for (LLResultTypes.FiducialResult t : tags) {
            if (t.getTargetArea() > best.getTargetArea()) best = t;
        }
    
        return best.getTargetPoseCameraSpace(); // <-- this is the key
    }

    private double getWallTagDistanceInchesFromCameraPose(Pose3D targetCamPose) {
        if (targetCamPose == null) return -1;
    
        double x = targetCamPose.getPosition().x;  // left/right (m)
        double z = targetCamPose.getPosition().z;  // forward/out (m)
    
        double horizontalMeters = Math.sqrt(x * x + z * z);
        return horizontalMeters * 39.37;
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
    
        float hue = hsv[0];        // 0..360
        float sat = hsv[1];        // 0..1
        float val = hsv[2];        // 0..1
    
        // Basic “is there actually a colored ball here?” gate
        boolean confident = (sat > 0.35) && (val > 0.10);
    
        // Starting ranges (you WILL tune these with telemetry)
        // Green is usually ~90–150 hue, Purple is usually ~250–310 hue
        ballIsGreen  = confident && (hue >= 90 && hue <= 150);
        ballIsPurple = confident && (hue >= 250 && hue <= 310);
    
        // If lighting makes purple wrap weirdly, you can broaden:
        // ballIsPurple = confident && ((hue >= 240 && hue <= 320));
    }

    private void buttons() {
        // Run motor0b at full power (1) when button A is pressed, stop when released
        if (gamepad2.dpad_up) {
//            motor0b.setPower(1.0); // Move up
        } else if (gamepad2.dpad_down) {
//            motor0b.setPower(-1.0); // Move down
        } else {
//            motor0b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            motor0b.setPower(0); // Stop motor
        }





        if (gamepad2.dpad_left) {
            motor0b.setPower(0);
            motor1b.setPower(0);
        } else {
            motor0b.setPower(1);
            motor1b.setPower(1);

        }
        if (gamepad2.left_bumper) {
 //           servo1.setPower(-1.0);   // backwards while held
        } else {
//            servo1.setPower(1.0);    // forward when released
        }
        if (gamepad2.right_bumper) {
            servo2.setPosition(1.0);   // backwards while held
        } else {
            servo2.setPosition(0.0);    // forward when released
        }



        if (gamepad2.x) {
            //servo1.setPower(1.0); // Move up
        } else if (gamepad2.y) {
            //servo1.setPower(-1.0); // Move down
        } else {
           //servo1.setPower(0); // Stop motor
        }

    }

    /**
     * Describe this function...
     */
    private void sticks2() {
        double gain;

        // turbo mode
        if (gamepad1.left_bumper) {
            gain = 1;
        } else {
            gain = 0.5;
        }
        strafePower = gain * LSX;
        drivePower = gain * LSY;
        rotatePower = gain * RSX;
        sticks4();

    }

    /**
     * Describe this function...
     */
    private void sticks1() {
        RSX = -gamepad1.right_stick_x;
        LSY = gamepad1.left_stick_y;
        LSX = gamepad1.left_stick_x;
        sticks2();
    }

    /**
         * Gyro correct values. Code from ChatGTP created code.
         */
        private void sticks4() {
//            YawValue = Math.round(imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) * (Math.PI / 180);
//            correctedStrafePower = strafePower * Math.cos(YawValue / 180 * Math.PI) - drivePower * Math.sin(YawValue / 180 * Math.PI);
//            correctedDrivePower = drivePower * Math.cos(YawValue / 180 * Math.PI) + strafePower * Math.sin(YawValue / 180 * Math.PI);
//            drive2();

            correctedStrafePower = -strafePower;
            correctedDrivePower = drivePower;

            drive2();
        }

    /**
         * Send corrected values to motors
         */
        private void drive2() {
            // Front right motor
            motor0.setPower((correctedDrivePower - correctedStrafePower) - rotatePower);
            // Front left motor
            motor2.setPower((correctedDrivePower + correctedStrafePower) + rotatePower);
            // Back right motor
            motor3.setPower((correctedDrivePower - correctedStrafePower) + rotatePower);
            // Back left motor
            motor1.setPower((correctedDrivePower + correctedStrafePower) - rotatePower);
        }

    private void lights(){
            if (gamepad2.a && !wasButtonAPressed) { // Detect button press (not hold)
                isRedOn = !isRedOn;                 // Toggle the state
                redLED.setState(isRedOn);           // Update LED state
            }
            wasButtonAPressed = gamepad2.x;         // Remember the button state


            if (gamepad2.b && !wasButtonBPressed) { // Detect button press (not hold)
                isBlueOn = !isBlueOn;               // Toggle the state
                blueLED.setState(isBlueOn);         // Update LED state
            }
            wasButtonBPressed = gamepad2.y;         // Remember the button state
        }
}
/**
     * Describe this function...
     */





