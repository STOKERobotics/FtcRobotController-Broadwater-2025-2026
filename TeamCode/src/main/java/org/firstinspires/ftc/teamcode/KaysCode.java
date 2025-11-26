package org.firstinspires.ftc.teamcode;

public class KaysCode {
while (opModeIsActive()) {

        LLResult result = limelight.getLatestResult();

        // If we do NOT see any tag -> keep turning left slowly
        if (result == null || !result.isValid()) {
            leftFront.setPower(-0.15);
            leftRear.setPower(-0.15);
            rightFront.setPower(0.15);
            rightRear.setPower(0.15);
        }
        // If we DO see a tag -> STOP turning
        else {
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);

        }
        private void driveForwardGyro(double inches, double power) {
            double COUNTS_PER_REV = 537.7;       // GoBilda 312 RPM, change if needed
            double WHEEL_DIAMETER = 4.0;         // your wheel size in inches
            double COUNTS_PER_INCH = COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER);

            int moveCounts = (int)(inches * COUNTS_PER_INCH);

            // Reset encoders
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set target encoder positions
            leftFront.setTargetPosition(moveCounts);
            leftRear.setTargetPosition(moveCounts);
            rightFront.setTargetPosition(moveCounts);
            rightRear.setTargetPosition(moveCounts);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Save the starting heading
            double startHeading = getHeading();

            // Start driving
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightFront.setPower(power);
            rightRear.setPower(power);

            while (opModeIsActive() &&
                    (leftFront.isBusy() || rightFront.isBusy() ||
                            leftRear.isBusy()  || rightRear.isBusy())) {

                double currentHeading = getHeading();
                double error = currentHeading - startHeading;
                double kP = 0.03;    // Tune this if needed

                double correction = error * kP;

                // Apply correction (add to left, subtract from right)
                leftFront.setPower(power + correction);
                leftRear.setPower(power + correction);
                rightFront.setPower(power - correction);
                rightRear.setPower(power - correction);

                telemetry.addData("Heading", currentHeading);
                telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motors
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        private void driveForwardGyro(double inches, double power) {
            double COUNTS_PER_REV = 537.7;       // GoBilda 312 RPM, change if needed
            double WHEEL_DIAMETER = 4.0;         // your wheel size in inches
            double COUNTS_PER_INCH = COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER);

            int moveCounts = (int)(inches * COUNTS_PER_INCH);

            // Reset encoders
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set target encoder positions
            leftFront.setTargetPosition(moveCounts);
            leftRear.setTargetPosition(moveCounts);
            rightFront.setTargetPosition(moveCounts);
            rightRear.setTargetPosition(moveCounts);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Save the starting heading
            double startHeading = getHeading();

            // Start driving
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightFront.setPower(power);
            rightRear.setPower(power);

            while (opModeIsActive() &&
                    (leftFront.isBusy() || rightFront.isBusy() ||
                            leftRear.isBusy()  || rightRear.isBusy())) {

                double currentHeading = getHeading();
                double error = currentHeading - startHeading;
                double kP = 0.03;    // Tune this if needed

                double correction = error * kP;

                // Apply correction (add to left, subtract from right)
                leftFront.setPower(power + correction);
                leftRear.setPower(power + correction);
                rightFront.setPower(power - correction);
                rightRear.setPower(power - correction);

                telemetry.addData("Heading", currentHeading);
                telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motors
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        private void driveForwardGyro(double inches, double power) {
            double COUNTS_PER_REV = 537.7;       // GoBilda 312 RPM, change if needed
            double WHEEL_DIAMETER = 4.0;         // your wheel size in inches
            double COUNTS_PER_INCH = COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER);

            int moveCounts = (int)(inches * COUNTS_PER_INCH);

            // Reset encoders
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set target encoder positions
            leftFront.setTargetPosition(moveCounts);
            leftRear.setTargetPosition(moveCounts);
            rightFront.setTargetPosition(moveCounts);
            rightRear.setTargetPosition(moveCounts);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Save the starting heading
            double startHeading = getHeading();

            // Start driving
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightFront.setPower(power);
            rightRear.setPower(power);

            while (opModeIsActive() &&
                    (leftFront.isBusy() || rightFront.isBusy() ||
                            leftRear.isBusy()  || rightRear.isBusy())) {

                double currentHeading = getHeading();
                double error = currentHeading - startHeading;
                double kP = 0.03;    // Tune this if needed

                double correction = error * kP;

                // Apply correction (add to left, subtract from right)
                leftFront.setPower(power + correction);
                leftRear.setPower(power + correction);
                rightFront.setPower(power - correction);
                rightRear.setPower(power - correction);

                telemetry.addData("Heading", currentHeading);
                telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motors
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        private double getHeading() {
            return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }