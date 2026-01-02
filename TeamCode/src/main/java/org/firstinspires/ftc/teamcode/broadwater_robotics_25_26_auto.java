package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous(name = "Broadwater Robotics Auto")
public class broadwater_robotics_25_26_auto extends LinearOpMode {

    // Drive motors (same names as your TeleOp)
    private DcMotor motor0, motor1, motor2, motor3;

    // Limelight
    private Limelight3A limelight;

    // -------------------- TUNING --------------------
    private static final double KP_XY = 1.2;        // meters -> power (start ~0.8 to 1.8)
    private static final double KP_H  = 0.02;       // deg -> power (start ~0.015 to 0.03)

    private static final double MAX_DRIVE_POWER  = 0.55;
    private static final double MAX_STRAFE_POWER = 0.55;
    private static final double MAX_TURN_POWER   = 0.35;

    // "Arrived" tolerances
    private static final double POS_TOL_M  = 0.05;  // 5 cm
    private static final double ANG_TOL_DEG = 3.0;  // degrees

    // Safety timeout per waypoint
    private static final long WAYPOINT_TIMEOUT_MS = 5000;
    // ------------------------------------------------

    // ====== PUT YOUR TARGETS HERE (meters, meters, degrees) ======
    // Example: go to (x=1.20m, y=0.40m, heading=90deg)
    private final Waypoint[] path = new Waypoint[] {
            new Waypoint(1.20, 0.40, 90),
            new Waypoint(1.20, 0.90, 90),
            new Waypoint(0.60, 0.90, 180),
    };
    // ============================================================

    @Override
    public void runOpMode() {

        // --- hardware map ---
        motor0 = hardwareMap.get(DcMotor.class, "motor0"); // front right (per your drive2)
        motor1 = hardwareMap.get(DcMotor.class, "motor1"); // back left
        motor2 = hardwareMap.get(DcMotor.class, "motor2"); // front left
        motor3 = hardwareMap.get(DcMotor.class, "motor3"); // back right

        initLimelight();

        telemetry.addLine("BotposeGoToCoordinates ready.");
        telemetry.addLine("Make sure Limelight localization is enabled and botpose is non-null.");
        telemetry.update();

        waitForStart();

        // Optional: wait briefly until botpose exists
        long startWait = System.currentTimeMillis();
        while (opModeIsActive() && getBotpose() == null && System.currentTimeMillis() - startWait < 1500) {
            telemetry.addLine("Waiting for botpose...");
            telemetry.update();
            idle();
        }

        if (!opModeIsActive()) return;

        // Run your waypoint list
        for (int i = 0; i < path.length && opModeIsActive(); i++) {
            telemetry.addData("Waypoint", "%d / %d", i + 1, path.length);
            telemetry.update();

            boolean ok = driveToWaypoint(path[i], WAYPOINT_TIMEOUT_MS);

            stopDrive();
            sleep(150);

            if (!ok) {
                telemetry.addLine("Failed/Timed out on waypoint. Stopping auto.");
                telemetry.update();
                break;
            }
        }

        stopDrive();
    }

    // -------------------- Core Auto Logic --------------------

    private boolean driveToWaypoint(Waypoint target, long timeoutMs) {
        long start = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {

            PoseEstimate cur = getPoseEstimate();
            if (cur == null) {
                stopDrive();
                telemetry.addLine("botpose is null - can’t navigate.");
                telemetry.update();
                return false;
            }

            // Field error (meters, degrees)
            double dx = target.x - cur.x;
            double dy = target.y - cur.y;
            double dh = angleWrapDeg(target.headingDeg - cur.headingDeg);

            double dist = Math.hypot(dx, dy);

            // Arrived?
            if (dist <= POS_TOL_M && Math.abs(dh) <= ANG_TOL_DEG) {
                telemetry.addLine("Arrived!");
                telemetry.update();
                return true;
            }

            // Convert FIELD error -> ROBOT frame using current heading
            // Assumption: headingDeg is field heading, 0 aligned with +X.
            // If your axes feel swapped/inverted, see notes at bottom.
            double hRad = Math.toRadians(cur.headingDeg);
            double robotForward =  dx * Math.cos(hRad) + dy * Math.sin(hRad);
            double robotLeft    = -dx * Math.sin(hRad) + dy * Math.cos(hRad);

            // P control -> power
            double drive  = clip(robotForward * KP_XY,  -MAX_DRIVE_POWER,  MAX_DRIVE_POWER);
            double strafe = clip(robotLeft    * KP_XY,  -MAX_STRAFE_POWER, MAX_STRAFE_POWER);
            double turn   = clip(dh * KP_H,           -MAX_TURN_POWER,   MAX_TURN_POWER);

            // Send to your mecanum mixer
            setDrivePower(drive, strafe, turn);

            telemetry.addData("CUR (m,deg)", "x=%.3f y=%.3f h=%.1f", cur.x, cur.y, cur.headingDeg);
            telemetry.addData("TGT (m,deg)", "x=%.3f y=%.3f h=%.1f", target.x, target.y, target.headingDeg);
            telemetry.addData("ERR", "dx=%.3f dy=%.3f dist=%.3f dh=%.1f", dx, dy, dist, dh);
            telemetry.addData("CMD", "drive=%.2f strafe=%.2f turn=%.2f", drive, strafe, turn);
            telemetry.update();

            idle();
        }

        stopDrive();
        return false; // timeout
    }

    // -------------------- Limelight Helpers --------------------

    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    private Pose3D getBotpose() {
        LLResult result = limelight.getLatestResult();
        if (result == null) return null;

        Pose3D botpose = result.getBotpose();
        return botpose; // can be null if localization not ready
    }

    private PoseEstimate getPoseEstimate() {
        Pose3D p = getBotpose();
        if (p == null) return null;

        double x = p.getPosition().x; // meters
        double y = p.getPosition().y; // meters

        // Try to read yaw from Pose3D orientation.
        // Different FTC versions name this slightly differently; if this line errors,
        // tell me the exact error and I’ll adjust to your SDK.
        double headingDeg = p.getOrientation().getYaw(AngleUnit.DEGREES);

        return new PoseEstimate(x, y, headingDeg);
    }

    // -------------------- Drive (same mixing idea as your TeleOp) --------------------

    /**
     * drive  = +forward
     * strafe = +left   (if it feels reversed, flip sign here)
     * turn   = +CCW
     */
    private void setDrivePower(double drive, double strafe, double turn) {
        // Your TeleOp had correctedStrafePower = -strafePower
        // so we keep that behavior here:
        double correctedStrafe = -strafe;
        double correctedDrive  = drive;

        // Front right motor0
        motor0.setPower((correctedDrive - correctedStrafe) - turn);
        // Back left motor1
        motor1.setPower((correctedDrive + correctedStrafe) - turn);
        // Front left motor2
        motor2.setPower((correctedDrive + correctedStrafe) + turn);
        // Back right motor3
        motor3.setPower((correctedDrive - correctedStrafe) + turn);
    }

    private void stopDrive() {
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }

    // -------------------- Small utils --------------------

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double angleWrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    // -------------------- Data classes --------------------

    private static class Waypoint {
        final double x, y;
        final double headingDeg;
        Waypoint(double x, double y, double headingDeg) {
            this.x = x; this.y = y; this.headingDeg = headingDeg;
        }
    }

    private static class PoseEstimate {
        final double x, y;
        final double headingDeg;
        PoseEstimate(double x, double y, double headingDeg) {
            this.x = x; this.y = y; this.headingDeg = headingDeg;
        }
    }
}