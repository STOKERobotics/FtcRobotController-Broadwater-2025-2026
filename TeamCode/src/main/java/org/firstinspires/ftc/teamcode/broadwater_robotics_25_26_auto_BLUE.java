package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Broadwater Auto Blue (Linked)")
public class broadwater_robotics_25_26_auto_BLUE extends BroadwaterRobotBase {

    // ==================== AUTONOMOUS-SPECIFIC STATE ====================
    private boolean shoot = false;
    private boolean align = false;

    // Virtual stick inputs for reusing drive methods
    private float RSX, LSY, LSX;
    private double strafePower, drivePower, rotatePower;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Start motors
            motor0b.setPower(0.6);
            motor1b.setPower(0.6);
            motor2b.setPower(1.0);

            // Enable motif listener
            motifListenEnabled = true;

            // Main autonomous sequence for BLUE alliance
            executeAutonomousSequence();
        }
    }

    // ==================== AUTONOMOUS SEQUENCE ====================
    private void executeAutonomousSequence() {
        // BLUE alliance autonomous sequence
        // This is mirrored from RED - adjust coordinates as needed

        // 1. Drive forward to scoring position
        driveForwardMeters(1.0, 0.5);
        sleep(500);

        // 2. Turn to face target (BLUE side - opposite angle from RED)
        turnDegrees(-45, 0.3);  // Negative for BLUE side
        sleep(500);

        // 3. Align to AprilTag
        motifListenEnabled = true;
        align = true;

        long alignStart = System.currentTimeMillis();
        while (opModeIsActive() && align && (System.currentTimeMillis() - alignStart) < 5000) {
            updateMagnetStates();

            // Try to latch motif
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                updateLatchedMotif(result);
            }

            // Align to target
            boolean aligned = alignToTarget();
            if (aligned && motifLatched) {
                align = false;
                shoot = true;
            }

            telemetryUpdateThrottled();
            idle();
        }

        // 4. Shoot if aligned
        if (shoot && motifLatched) {
            adjustShooterAndFire();
        }

        // 5. Park (BLUE side parking position)
        driveForwardMeters(-0.5, 0.3);

        telemetry.addData("Status", "Autonomous Complete - BLUE");
        telemetry.update();
    }

    // ==================== MOVEMENT METHODS ====================

    /**
     * Drive forward/backward using encoders with IMU heading hold
     * @param meters Distance to travel (positive = forward, negative = backward)
     * @param power Motor power (0.0 to 1.0)
     */
    private void driveForwardMeters(double meters, double power) {
        setDriveRunUsingEncoder();

        double distM = Math.abs(meters);
        if (distM < 1e-6) return;

        if (Math.abs(power) < 1e-6) {
            stopDrive();
            return;
        }
        int dir = (meters >= 0) ? +1 : -1;

        int ticksTarget = (int) Math.round((distM / WHEEL_CIRCUMFERENCE_M) * TICKS_PER_WHEEL_REV);

        int startFR = motor0.getCurrentPosition();
        int startBL = motor1.getCurrentPosition();
        int startFL = motor2.getCurrentPosition();
        int startBR = motor3.getCurrentPosition();

        double startYaw = getYawDeg();

        double p = clip(Math.abs(power), 0.0, 1.0);
        double gain = 0.5;
        double stickLSY = clip(dir * (p / gain), -1.0, 1.0);

        final double HEADING_KP = 0.015;
        final double MAX_CORR = 0.20;
        final long TIMEOUT_MS = 6000;

        long startMs = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - startMs) < TIMEOUT_MS) {
            updateMagnetStates();

            int dFR = motor0.getCurrentPosition() - startFR;
            int dBL = motor1.getCurrentPosition() - startBL;
            int dFL = motor2.getCurrentPosition() - startFL;
            int dBR = motor3.getCurrentPosition() - startBR;

            double travelTicks = (Math.abs(dFL) + Math.abs(dFR) + Math.abs(dBL) + Math.abs(dBR)) / 4.0;

            if (travelTicks >= ticksTarget) break;

            double yaw = getYawDeg();
            double errDeg = angleWrapDeg(yaw - startYaw);

            if (Math.abs(errDeg) < HEADING_DEADBAND_DEG) errDeg = 0.0;

            double corr = clip((-HEADING_SIGN) * errDeg * HEADING_KP, -MAX_CORR, MAX_CORR);
            double stickLSX = clip(corr / gain, -1.0, 1.0);

            LSY = (float) stickLSY;
            LSX = (float) stickLSX;
            RSX = 0f;

            applyVirtualSticks();

            telemetry.addData("DriveIMU", "target=%d travel=%.0f", ticksTarget, travelTicks);
            telemetry.addData("Heading", "start=%.1f yaw=%.1f err=%.1f", startYaw, yaw, errDeg);
            telemetryUpdateThrottled();
        }

        LSY = 0f;
        LSX = 0f;
        RSX = 0f;
        applyVirtualSticks();
        stopDrive();
    }

    /**
     * Turn robot by degrees using IMU
     * @param degrees Angle to turn (positive = left/CCW, negative = right/CW)
     * @param maxPower Maximum motor power
     */
    private void turnDegrees(double degrees, double maxPower) {
        double startYaw = getYawDeg();
        double targetYaw = angleWrapDeg(startYaw + degrees);

        final double TOL_DEG = 2.0;
        final double MIN_OUT = 0.10;
        final double KP = 0.012;
        final long TIMEOUT = 4000;

        double pMax = clip(Math.abs(maxPower), 0.0, 1.0);
        double gain = 0.5;

        long startMs = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startMs) < TIMEOUT) {
            updateMagnetStates();

            double yaw = getYawDeg();
            double err = angleWrapDeg(targetYaw - yaw);

            telemetry.addData("Turn", "target=%.1f yaw=%.1f err=%.1f", targetYaw, yaw, err);
            telemetryUpdateThrottled();

            if (Math.abs(err) <= TOL_DEG) break;

            double out = err * KP;
            out = clip(out, -pMax, pMax);

            if (Math.abs(out) < MIN_OUT)
                out = MIN_OUT * Math.signum(out);

            if (Math.abs(err) < 10.0)
                out = clip(out, -0.18, 0.18);

            double stickLSX = clip(out / gain, -1.0, 1.0);

            LSY = 0f;
            RSX = 0f;
            LSX = (float) stickLSX;

            applyVirtualSticks();
        }

        LSY = 0f;
        LSX = 0f;
        RSX = 0f;
        applyVirtualSticks();
        stopDrive();
        sleep(120);
    }

    /**
     * Strafe left/right using encoders
     * @param meters Distance to strafe (positive = right, negative = left)
     * @param power Motor power
     */
    private void strafeMeters(double meters, double power) {
        setDriveRunUsingEncoder();

        double distM = Math.abs(meters);
        if (distM < 1e-6) return;

        if (Math.abs(power) < 1e-6) {
            stopDrive();
            return;
        }
        int dir = (meters >= 0) ? +1 : -1;

        // Strafe typically needs more ticks due to wheel slippage
        int ticksTarget = (int) Math.round((distM / WHEEL_CIRCUMFERENCE_M) * TICKS_PER_WHEEL_REV * 1.3);

        int startFR = motor0.getCurrentPosition();
        int startBL = motor1.getCurrentPosition();
        int startFL = motor2.getCurrentPosition();
        int startBR = motor3.getCurrentPosition();

        double startYaw = getYawDeg();

        double p = clip(Math.abs(power), 0.0, 1.0);
        double gain = 0.5;
        double stickRSX = clip(dir * (p / gain), -1.0, 1.0);

        final double HEADING_KP = 0.015;
        final double MAX_CORR = 0.20;
        final long TIMEOUT_MS = 6000;

        long startMs = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - startMs) < TIMEOUT_MS) {
            updateMagnetStates();

            int dFR = motor0.getCurrentPosition() - startFR;
            int dBL = motor1.getCurrentPosition() - startBL;
            int dFL = motor2.getCurrentPosition() - startFL;
            int dBR = motor3.getCurrentPosition() - startBR;

            // For strafe, measure diagonal wheel movement
            double travelTicks = (Math.abs(dFL) + Math.abs(dFR) + Math.abs(dBL) + Math.abs(dBR)) / 4.0;

            if (travelTicks >= ticksTarget) break;

            double yaw = getYawDeg();
            double errDeg = angleWrapDeg(yaw - startYaw);

            if (Math.abs(errDeg) < HEADING_DEADBAND_DEG) errDeg = 0.0;

            double corr = clip((-HEADING_SIGN) * errDeg * HEADING_KP, -MAX_CORR, MAX_CORR);
            double stickLSX = clip(corr / gain, -1.0, 1.0);

            LSY = 0f;
            LSX = (float) stickLSX;
            RSX = (float) stickRSX;

            applyVirtualSticks();

            telemetry.addData("Strafe", "target=%d travel=%.0f", ticksTarget, travelTicks);
            telemetryUpdateThrottled();
        }

        LSY = 0f;
        LSX = 0f;
        RSX = 0f;
        applyVirtualSticks();
        stopDrive();
    }

    // ==================== UTILITY METHODS ====================

    private void setDriveRunUsingEncoder() {
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Apply virtual stick inputs to drive motors
     * Replicates the sticks2() method from TeleOp
     */
    private void applyVirtualSticks() {
        double gain = 0.5; // Always use reduced gain in autonomous

        drivePower = gain * LSY;
        rotatePower = gain * LSX;
        strafePower = gain * RSX;

        driveRobotCentric(drivePower, strafePower, rotatePower);
    }
}
