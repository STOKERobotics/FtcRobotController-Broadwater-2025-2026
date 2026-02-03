package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Broadwater Robotics TeleOp (Linked)")
public class broadwater_robotics_25_26_teleop    extends BroadwaterRoboticsBase {

    // ==================== TELEOP-SPECIFIC STATE ====================
    private boolean alignActive = false;

    // Button press tracking
    private boolean wasMotifListenTogglePressed = false;
    private boolean wasKickTrigger = false;
    private boolean wasForceSkipTrigger = false;
    private boolean wasShootAdvancePressed = false;
    private boolean wasStepShootPressed = false;
    private boolean wasRedToggle = false;
    private boolean wasBlueToggle = false;

    // Drive control
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
//            motor0b.setPower(0.7);
            motor1b.setPower(0.7);
            motor2b.setPower(1.0);

            // Main loop
            while (opModeIsActive()) {
                // OPTIMIZATION: Single magnet read per loop cycle
                updateMagnetStates();

                // Process game logic
                processAlignment();
                processMotifListener();
                processIntake();
                processControls();
                processLights();

                // Telemetry
                updateTelemetry();
                telemetryUpdateThrottled();
            }
        }
    }

    // ==================== TELEOP PROCESSING ====================
    private void processAlignment() {
        // Start align
        if (gamepad2.a && !alignActive) {
            alignActive = true;
        }

        // Cancel align
        if (gamepad2.b && alignActive) {
            alignActive = false;
            stopDrive();
        }

        // Execute alignment
        if (alignActive) {
            boolean aligned = alignToTarget();
            if (aligned) {
                adjustShooterAndFire();
                alignActive = false;
            }
        }
    }

    private void processMotifListener() {
        if (motifListenEnabled) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                updateLatchedMotif(result);
            }
        }
    }

    private void processIntake() {
        // Manual step control
        if (gamepad2.left_bumper && !wasForceSkipTrigger) {
            handleManualStep();
            wasForceSkipTrigger = true;
            return;
        }
        wasForceSkipTrigger = gamepad2.left_bumper;

        // Auto intake state machine
        runIntakeStateMachine();
    }

    private void handleManualStep() {
        int from = getCurrentIntakeSlot();
        if (from < 0) from = currentSlot;
        int to = (from + INTAKE_SLOT_STEP + 3) % 3;

        rotateToSlotBlocking(to, false);

        currentSlot = to;
        intakeState = waitStateForSlot(to);
        colorLatched = false;
    }

    private void processControls() {
        if (alignActive) {
            return; // Don't process manual controls during alignment
        }

        // Drive controls
        LSX = -gamepad1.right_stick_x;
        LSY = gamepad1.left_stick_y;
        RSX = -gamepad1.left_stick_x;

        double gain = gamepad1.left_bumper ? 0.5 : 1.0;
        drivePower = gain * LSY;
        rotatePower = gain * LSX;
        strafePower = gain * RSX;

        driveRobotCentric(drivePower, strafePower, rotatePower);

        // Button controls
        processButtons();
    }

    private void processButtons() {
        // Servo2 control (shooter angle)
        double now = getRuntime();
        double dt = clamp(now - lastServo2Time, 0, 0.1);
        lastServo2Time = now;

        // ONLY update servo2 if dpad is pressed (manual control)
        if (gamepad2.dpadUpWasPressed()) {
            servo2Pos += SERVO2_RATE * dt;
            servo2Pos = clamp(servo2Pos, SERVO2_MIN, SERVO2_MAX);
            servo2.setPosition(servo2Pos); // Update immediately
        } else if (gamepad2.dpadDownWasPressed()) {
            servo2Pos -= SERVO2_RATE * dt;
            servo2Pos = clamp(servo2Pos, SERVO2_MIN, SERVO2_MAX);
            servo2.setPosition(servo2Pos); // Update immediately
        }
        // NOTE: Removed "always update" behavior - only updates when dpad pressed

        // Shooter power adjustment
        if (gamepad2.dpadRightWasPressed()) {
//            motor0b.setPower(motor0b.getPower() + 0.1);
            motor1b.setPower(motor1b.getPower() + 0.05);
        }
        if (gamepad2.dpadLeftWasPressed()) {
//            motor0b.setPower(motor0b.getPower() - 0.1);
            motor1b.setPower(motor1b.getPower() - 0.05);
        }
        if (motor1b.getPower() < 0.1)
            motor1b.setPower(.15);

        // Motif listener toggle
        if (gamepad2.x && !wasMotifListenTogglePressed) {
            motifListenEnabled = !motifListenEnabled;
            if (motifListenEnabled) {
                motifLatched = false;
                latchedMotif = "NONE";
                latchedTagId = -1;
            }
        }
        wasMotifListenTogglePressed = gamepad2.x;

        // Manual kick
        boolean kickTrigger = gamepad2.right_trigger > 0.6f;
        if (kickTrigger && !wasKickTrigger) {
            ensureKickerRetracted();
            kickOnce();
        }
        wasKickTrigger = kickTrigger;

        // Shoot tray advance
        if (gamepad2.right_bumper && !wasShootAdvancePressed) {
            stopDrive();
            stepShootTrayOneSlotNoKick();
        }
        wasShootAdvancePressed = gamepad2.right_bumper;

        // Step shoot
        if (gamepad2.y && !wasStepShootPressed) {
            stepToNextSlotAndShoot();
        }
        wasStepShootPressed = gamepad2.y;
    }

    private void stepShootTrayOneSlotNoKick() {
        if (shootingBusy) {
            telemetry.addData("BLOCKED", "Shooting busy - wait for current action to finish");
            telemetry.update();
            return;
        }
        ensureKickerRetracted();

        int current = getCurrentShootSlot();
        if (current < 0) current = 0;
        int next = (current + SHOOT_SLOT_STEP + 3) % 3;

        rotateToSlotBlocking(next, true);
    }

    private void stepToNextSlotAndShoot() {
        if (shootingBusy) {
            telemetry.addData("BLOCKED", "Shooting busy - wait for current action to finish");
            telemetry.update();
            return;
        }
        ensureKickerRetracted();

        if (!motifReadyForStepShoot()) {
            telemetry.addData("NOT READY", "Need motif and balls loaded");
            telemetry.update();
            return;
        }

        if (!stepModeActive || stepShotIndex >= 3) {
            resetStepShootSequence();
        }

        char wanted = Character.toLowerCase(latchedMotif.charAt(stepShotIndex));
        int slotToShoot = findSlotForColor(wanted);

        if (slotToShoot < 0) {
            telemetry.addData("ERROR", "No ball found for color '%c'", wanted);
            telemetry.update();
            return;
        }

        int shootFrameSlot = intakeSlotToShootSlot(slotToShoot);
        rotateToSlotBlocking(shootFrameSlot, true);
        kickOnce();

        slotFired[slotToShoot] = true;
        slots[slotToShoot] = null;
        stepShotIndex++;

        sleep(120);
    }

    private boolean motifReadyForStepShoot() {
        return motifLatched
                && latchedMotif != null
                && latchedMotif.length() == 3
                && !"UNKNOWN".equals(latchedMotif)
                && (slots[0] != null || slots[1] != null || slots[2] != null);
    }

    private void resetStepShootSequence() {
        for (int i = 0; i < 3; i++) slotFired[i] = false;
        stepShotIndex = 0;
        stepModeActive = true;
    }

    private void processLights() {
        // Toggle RED
        if (gamepad2.options && !wasRedToggle) {
            isRedOn = !isRedOn;
            redLED.setState(!isRedOn);
        }
        wasRedToggle = gamepad2.options;

        // Toggle BLUE
        if (gamepad2.share && !wasBlueToggle) {
            isBlueOn = !isBlueOn;
            blueLED.setState(!isBlueOn);
        }
        wasBlueToggle = gamepad2.share;
    }

    private void updateTelemetry() {
        // Laser distance
        double v = laser.getVoltage();
        double mm = (v / 3.3) * 1000.0;
        double inches = mm / 25.4;

        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Slots", "0=%s 1=%s 2=%s", slots[0], slots[1], slots[2]);
        telemetry.addData("Shoot Slot", getCurrentShootSlot());
        telemetry.addData("Laser Dist", "%.0f mm (%.1f in)", mm, inches);
        telemetry.addData("Motif", "%s (Tag %d) %s",
                latchedMotif, latchedTagId, motifListenEnabled ? "LISTEN" : "");
        telemetry.addData("Shooter Pos", "%.3f", servo2.getPosition());
        telemetry.addData("Shooter Power", "%.2f / %.2f", motor0b.getPower(), motor1b.getPower());
        // telemetry.addData("imu", imu2.getAngularOrientation());

        // Last kick timing (only show if a kick has occurred)
        if (lastKickTotalMs > 0) {
            telemetry.addData("Last Kick", "ext=%dms ret=%dms TOTAL=%dms",
                    lastKickExtendMs, lastKickRetractMs, lastKickTotalMs);
        }
    }
}