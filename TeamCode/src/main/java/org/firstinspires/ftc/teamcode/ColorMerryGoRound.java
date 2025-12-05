package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "IntakeColorsWithLimelight")
public class ColorMerryGoRound extends LinearOpMode {

    // Hardware
    private DcMotor motor2b;
    private ColorSensor colorSensor;
    private Limelight3A limelight;

    // Spot storage: Spot 1, Spot 2, Spot 3
    private char[] spots = new char[3];  // 'P' or 'G'
    private int spotIndex = 0;

    @Override
    public void runOpMode() {

        // ---- Hardware Map ----
        motor2b = hardwareMap.get(DcMotor.class, "motor2b");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // ---- Limelight Setup ----
        limelight.pipelineSwitch(0);   // AprilTag pipeline
        limelight.start();

        waitForStart();

        // ===============================
        // INTAKE PHASE
        // ===============================
        while (opModeIsActive() && spotIndex < 3) {

            // Run intake
            motor2b.setPower(1.0);
            sleep(700);
            motor2b.setPower(0);

            // Detect and store color
            spots[spotIndex] = detectColor10Percent();

            telemetry.addData("Stored Spot " + (spotIndex + 1), spots[spotIndex]);
            telemetry.update();

            spotIndex++;
            sleep(300);
        }

        // ===============================
        // LIMELIGHT RECALL PHASE
        // ===============================
        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()
                    && result.getFiducialResults() != null
                    && !result.getFiducialResults().isEmpty()) {

                int tagID = result.getFiducialResults().get(0).getFiducialId();
                String order = getColorOrderFromTag(tagID);

                telemetry.addData("AprilTag ID", tagID);
                telemetry.addData("Requested Order", order);

                if (order != null && order.length() == 3) {

                    for (int step = 0; step < 3; step++) {
                        char wantedColor = order.charAt(step);
                        int spot = findSpotForColor(wantedColor);

                        telemetry.addData(
                                "Step " + (step + 1),
                                "Want " + wantedColor + " → Spot " + spot
                        );
                    }
                }
            }

            telemetry.update();
            idle();
        }
    }

    // ===============================
    // COLOR DETECTION (10% RULE)
    // ===============================
    private char detectColor10Percent() {

        sleep(250); // sensor stabilize

        double r = colorSensor.red();
        double g = colorSensor.green();
        double b = colorSensor.blue();
        double total = r + g + b;

        if (total <= 0) return 'U';

        double greenPercent = g / total;
        double purplePercent = (r + b) / total;

        telemetry.addData("Green %", greenPercent);
        telemetry.addData("Purple %", purplePercent);
        telemetry.update();

        if (greenPercent >= 0.10) return 'G';
        if (purplePercent >= 0.10) return 'P';

        return 'U';
    }

    // ===============================
    // APRILTAG → COLOR ORDER
    // ===============================
    private String getColorOrderFromTag(int tagID) {

        /*
         * Map AprilTag ID to requested color order
         * P = Purple
         * G = Green
         */

        switch (tagID) {
            case 1:
                return "PGP";
            case 2:
                return "GPP";
            case 3:
                return "PPG";
            default:
                return null;
        }
    }

    // ===============================
    // FIND SPOT FOR REQUESTED COLOR
    // ===============================
    private int findSpotForColor(char color) {

        for (int i = 0; i < spots.length; i++) {
            if (spots[i] == color) {
                return i + 1; // Spots are 1-based
            }
        }
        return -1; // Not found
    }
}
