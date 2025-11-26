package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "thirds_spin")
public class spin3 extends LinearOpMode {

    private DcMotor motor2b;

    private static final int ONE_THIRD_TURN = 96;   // Core Hex = 288 ticks/rev
    private int targetPos = 0;                      // next encoder target

    @Override
    public void runOpMode() throws InterruptedException {

        motor2b = hardwareMap.get(DcMotor.class, "motor2b");

        // Reset encoder
        motor2b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Current Pos", motor2b.getCurrentPosition());
            telemetry.addData("Next Target", targetPos);

            // Every press of circle moves another 1/3 rotation
            if (gamepad1.circle) {

                // Add 1/3 rotation to target
                targetPos += ONE_THIRD_TURN;

                motor2b.setTargetPosition(targetPos);
                motor2b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor2b.setPower(0.5);

                // Block until it reaches the position
                while (opModeIsActive() && motor2b.isBusy()) {
                    telemetry.addData("Moving to", targetPos);
                    telemetry.addData("Current", motor2b.getCurrentPosition());
                    telemetry.update();
                }

                // Stop motor
                motor2b.setPower(0);

                // Return to RUN_USING_ENCODER so we can command again
                motor2b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // Simple debounce so 1 press = 1 step
                while (opModeIsActive() && gamepad1.circle) {
                    idle();
                }
            }

            telemetry.update();
        }
    }
}
