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

    private boolean fieldCentric = false;
    private boolean wasToggleFC = false;
    private double fcZeroYawDeg = 0.0;
    private long lastShootAdvanceMs = 0;
    private static final long SHOOT_ADV_COOLDOWN_MS = 2000;


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
                updateArmAngle();

                // Process game logic
                processAlignment();
                processMotifListener();
                processIntake();
                processControls();
                processLights();
                updateArmAngle();

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
        stopDrive();

        int found = spinToNextIntakeSlotAndStopFastCapture(MGR_FAST_POWER, MGR_CRAWL_POWER);


        if (found != -1) {
            currentSlot = found;
            intakeState = waitStateForSlot(found);
            colorLatched = false;

            // settle + prevent “recapture”
            servo1.setPower(0);
            sleep(180);
            resetMagnetTiming();   // this clears BOTH intake+shoot timers (fine)
            updateMagnetStates();
        }
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

        if (fieldCentric) {
            // Use yaw relative to your chosen "field forward"
            double headingDeg = angleWrapDeg(getYawDeg() - fcZeroYawDeg);
            double headingRad = Math.toRadians(headingDeg);

            double cosA = Math.cos(-headingRad);
            double sinA = Math.sin(-headingRad);

            double yRobot = drivePower * cosA - strafePower * sinA;
            double xRobot = drivePower * sinA + strafePower * cosA;

            driveRobotCentric(yRobot, xRobot, rotatePower);
        } else {
            driveRobotCentric(drivePower, strafePower, rotatePower);
        }


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
            servo2Pos += 0.05;
            servo2.setPosition(servo2Pos); // Update immediately
        } else if (gamepad2.dpadDownWasPressed()) {
            servo2Pos -= 0.05;
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
        // Toggle field-centric on/off
        if (gamepad1.y && !wasToggleFC) {
            fieldCentric = !fieldCentric;
        }
        wasToggleFC = gamepad1.y;

        // Re-zero heading for field-centric (face forward then press)
        if (gamepad1.x) {
            fcZeroYawDeg = getYawDeg();
        }

        wasMotifListenTogglePressed = gamepad2.x;

        // Manual kick
        boolean kickTrigger = gamepad2.right_trigger > 0.6f;
        if (kickTrigger && !wasKickTrigger) {
            ensureKickerRetracted();
            kickOnce();
        }
        wasKickTrigger = kickTrigger;

        if (gamepad2.right_bumper && !wasShootAdvancePressed) {
            long nowMs = System.currentTimeMillis();
            if (nowMs - lastShootAdvanceMs >= SHOOT_ADV_COOLDOWN_MS) {
                lastShootAdvanceMs = nowMs;
                stepShootTrayOneSlotWithKick(); // or your no-kick version
            }
        }
        wasShootAdvancePressed = gamepad2.right_bumper;

        // Step shoot
        if (gamepad2.y && !wasStepShootPressed) {
            stepToNextSlotAndShoot();
        }
        wasStepShootPressed = gamepad2.y;
    }



    private void stepShootTrayOneSlotWithKick() {
        if (shootingBusy) return;

        stopDrive();
        ensureKickerRetracted();

        int found = spinToNextShootSlotAndStopFastCapture(MGR_FAST_POWER, MGR_CRAWL_POWER);
        //int found = spinToNextShootSlotAndStopFastOnly(MGR_FAST_POWER);

        if (found != -1) {
            // Optional: tiny settle so you don't bounce off the magnet edge
            sleep(80);

            // Shoot!
            kickOnce();

            telemetry.addData("Manual Step+Shoot", "shot at slot=%d patt=%s",
                    found, (mag2State ? "1":"0") + (mag3State ? "1":"0"));
        } else {
            telemetry.addData("Manual Step+Shoot", "FAILED patt=%s",
                    (mag2State ? "1":"0") + (mag3State ? "1":"0"));
        }
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
    protected double armAngleFiltered = 0.0;

    protected double getArmAngleDegFiltered() {
        double a = getArmAngleDeg();
        armAngleFiltered = 0.85 * armAngleFiltered + 0.15 * a;
        return armAngleFiltered;
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
    private double getFieldHeadingDeg() {
        return angleWrapDeg(getYawDeg() - fcZeroYawDeg);
    }

    private void updateTelemetry() {
        long now = System.currentTimeMillis();
        if (now - lastTelemMs < TELEM_PERIOD_MS) {
            return;  // ← nothing happens
        }

        telemetry.clearAll();

        double v = laser.getVoltage();
        double mm = (v / 3.3) * 1000.0;
        double inches = mm / 25.4;

        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Slots", "0=%s 1=%s 2=%s", slots[0], slots[1], slots[2]);
        //telemetry.addData("Shoot mags raw", "mag2=%s mag3=%s", mag2State ? "1" : "0", mag3State ? "1" : "0");
        telemetry.addData("Shoot Slot", getCurrentShootSlot());
        //telemetry.addData("Laser Dist", "%.0f mm (%.1f in)", mm, inches);
        telemetry.addData("imu1 yaw", "%.1f", getYawDeg());
        telemetry.addData("imu2 pitch", "%.1f", getImu2PitchDeg());
        telemetry.addData("Arm Angle", "%.1f", getArmAngleDeg());
        telemetry.addData("ShootRotate", "%s", lastShootRotateStatus);
        telemetry.addData("Drive Mode", fieldCentric ? "FIELD" : "ROBOT");


        telemetry.addData("Motif", "%s (Tag %d) %s",
                latchedMotif, latchedTagId, motifListenEnabled ? "LISTEN" : "");

        telemetry.addData("Shooter Pos", "%.3f", servo2.getPosition());
        telemetry.addData("Shooter Power", "%.2f", motor1b.getPower());
        telemetry.addData("Intake mags", "mag0/mag1=%s", intakePattern());
        telemetry.addData("Intake Slot", "%d", classifyIntakeSlotImmediate());
        telemetry.addData("Intake Slot(last)", "%d", getCurrentIntakeSlot()); // your latched value


        if (lastKickTotalMs > 0) {
            telemetry.addData("Last Kick", "ext=%dms ret=%dms TOTAL=%dms",
                    lastKickExtendMs, lastKickRetractMs, lastKickTotalMs);
        }

        // DO NOT telemetry.update() here; you already throttle it in the main loop
    }
    protected int spinToNextShootSlotAndStopFastCapture(double fastPower, double crawlPower) {
        shootingBusy = true;
        try {
            ensureKickerRetracted();
            updateMagnetStates();

            final long TIMEOUT_MS   = 0;
            final long STABLE_MS    = 70;
            final long CRAWL_MAX_MS = 0;

            long deadline = System.currentTimeMillis() + TIMEOUT_MS;

            int startSlot = classifyShootSlotImmediate();

            // KEY IDEA:
            // Only allow a "new slot" capture AFTER we've seen "between" (11).
            boolean seenBetween = !isShootSlotPattern(); // if we start in 11, we're already good.

            // 1) LEAVE current slot until we see between (11)
            if (!seenBetween) {
                servo1.setPower(fastPower);
                long leaveDeadline = Math.min(deadline, System.currentTimeMillis() + 600);
                while (opModeIsActive() && System.currentTimeMillis() < leaveDeadline) {
                    updateMagnetStates();
                    if (!isShootSlotPattern()) { // 11
                        seenBetween = true;
                        break;
                    }
                }
            }

            // 2) FAST SEARCH until we hit any slot pattern (10/01/00) AFTER we've seenBetween
            servo1.setPower(fastPower);
            while (opModeIsActive() && System.currentTimeMillis() < deadline) {
                updateMagnetStates();
                if (seenBetween && isShootSlotPattern()) break;
                telemetryUpdateThrottled();
            }

            // 3) CRAWL + STABLE HOLD to land precisely on the *next* slot (not the startSlot)
            servo1.setPower(crawlPower);
            long crawlDeadline = Math.min(deadline, System.currentTimeMillis() + CRAWL_MAX_MS);

            int found = -1;
            while (opModeIsActive() && System.currentTimeMillis() < crawlDeadline) {
                updateMagnetStates();

                int s = classifyShootSlotImmediate();
                boolean candidate = (s != -1) && seenBetween && (startSlot == -1 || s != startSlot);

                if (candidate) {
                    long holdStart = System.currentTimeMillis();
                    boolean ok = true;

                    while (opModeIsActive() && System.currentTimeMillis() < crawlDeadline) {
                        updateMagnetStates();
                        if (classifyShootSlotImmediate() != s) { ok = false; break; }
                        if (System.currentTimeMillis() - holdStart >= STABLE_MS) break;
                    }

                    if (ok && classifyShootSlotImmediate() == s) {
                        found = s;
                        break;
                    }
                }

                telemetryUpdateThrottled();
            }

            servo1.setPower(0);

            if (found == -1) {
                setShootRotateStatus("SHOOT FASTCAP FAIL start=" + startSlot + " patt=" + shootPattern());
            } else {
                setShootRotateStatus("SHOOT FASTCAP -> " + found + " patt=" + shootPattern());
            }

            return found;

        } finally {
            servo1.setPower(0);
            shootingBusy = false;
        }
    }


}