package org.firstinspires.ftc.teamcode.teleOpModes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Variables;

import java.util.List;

@TeleOp(name = "NewTeleOp")
public class NewTeleOp extends LinearOpMode {

    // Drivetrain
    private Motor fL, fR, bL, bR;
    private Motor intakeMotor;
    public double drive_speed = 1;

    // Shooter
    private DcMotorEx shooterLeft, shooterRight;
    private Servo shooterAngleServo;
    private double shooterAnglePos = 0.0;

    private static final int CPR = 28;
    private static final int MAX_RPM = 6000;
    private static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;

    private double targetVelocity = 0;

    // Shooter stability
    private final double SHOOTER_TOLERANCE_TICKS = 150.0;
    private final long SHOOTER_STABLE_TIME_MS = 300;
    private final ElapsedTime shooterStableTimer = new ElapsedTime();
    private boolean shooterSpeedInRange = false;
    private boolean shooterReady = false;

    // Sorter + transfer
    private Servo sorterLeftServo;
    private Servo sorterRightServo;
    private Servo kickerTopServo, kickerBottomServo;

    // Analog feedback
    private AnalogInput kickerAnalog;
    private AnalogInput sorterAnalog;

    // ---- Your measured voltages (midpoints of ranges) ----
    private static final double KICKER_DOWN_V = 3.0025;
    private static final double KICKER_UP_V   = 2.6975;

    // Intake pockets (1st ball goes to intake 1, etc.)
    private static final double SORTER_INTAKE_1_V  = 0.2200;
    private static final double SORTER_INTAKE_2_V  = 1.2850;
    private static final double SORTER_INTAKE_3_V  = 2.3640;

    // Outtake pockets (your shoot positions)
    private static final double SORTER_OUTTAKE_1_V = 1.7425;
    private static final double SORTER_OUTTAKE_2_V = 2.8405;
    private static final double SORTER_OUTTAKE_3_V = 0.6715;

    // Tolerances (tune as needed)
    private static final double KICKER_V_TOL = 0.020; // 20mV
    private static final double SORTER_V_TOL = 0.030; // 30mV

    // Timeouts (ms) so code never hangs forever
    private static final long KICKER_TIMEOUT_MS = 300;
    private static final long SORTER_TIMEOUT_MS = 300;

    // Intake power while outtaking (shoot all)
    private static final double OUTTAKE_INTAKE_POWER = 0.4;
    private boolean intakeWasEnabledBeforeShootAll = false;

    // Sensors
    private ColorSensor sensorColorFront;
    private DistanceSensor sensorDistanceFront;
    private ColorSensor sensorColorBack;
    private DistanceSensor sensorDistanceBack;

    // Ball memory (0 purple, 1 green, -1 unknown)
    private int ball1 = -1, ball2 = -1, ball3 = -1;

    // 0=waiting for ball1, 1=waiting for ball2, 2=waiting for ball3, 3=full
    private int sorterState = 0;

    // actual sorter rotation position
    // 1 = sorter1Position, 2 = sorter2Position, 3 = sorter3Position
    private int sorterPositionIndex = 1;

    private boolean lastBallPresent = false;
    private boolean intakeEnabled = false;

    private boolean lastShootAllPressed = false;

    // Detection constants
    private final double BALL_DETECT_DISTANCE = 3.0;
    private final long BALL_COOLDOWN_MS = 100;
    private final ElapsedTime ballTimer = new ElapsedTime();

    // Non-blocking “delay then move sorter” logic
    private final ElapsedTime sorterDelayTimer = new ElapsedTime();
    private boolean pendingBallSequence = false;
    private int pendingSorterState = -1;
    private int pendingColor = -1;

    // Sorter move tracking (voltage-based)
    private boolean sorterMoveActive = false;
    private double sorterTargetV = 0.0;
    private final ElapsedTime sorterMoveTimer = new ElapsedTime();
    private String sorterMoveError = "";

    // Non-blocking kicker pulse
    private enum KickerPulseState { IDLE, COMMAND_DOWN, WAIT_DOWN, COMMAND_UP, WAIT_UP }
    private KickerPulseState kickerPulseState = KickerPulseState.IDLE;
    private final ElapsedTime kickerPulseTimer = new ElapsedTime();

    // Non-blocking shoot-all sequence
    private boolean shootAllRunning = false;
    private int shootAllStep = 0;

    // ---------- TURRET + LIMELIGHT ----------
    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    private static final double TURRET_MAX_ANGLE_DEG = 90.0;
    private static final double TURRET_TICKS_PER_DEGREE = 232.0 / 90.0;

    private static final int TURRET_POSITION_TOLERANCE = 5;
    public static double TURRET_POSITION_P = 5.0;
    private static final double TURRET_MOVE_POWER = 0.5;

    private int turretPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // ----------------------------- DRIVETRAIN -----------------------------
        fL = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        fR = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        bL = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        bR = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        MecanumDrive drive = new MecanumDrive(fL, fR, bL, bR);

        // ----------------------------- INTAKE + SHOOTER -----------------------------
        intakeMotor = new Motor(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_435);

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setVelocityPIDFCoefficients(50, 0, 0.001, 11.7);

        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterAngleServo = hardwareMap.get(Servo.class, "shooterAngleServo");
        shooterAngleServo.setPosition(shooterAnglePos);

        // ----------------------------- SERVOS + SENSORS -----------------------------
        kickerTopServo = hardwareMap.get(Servo.class, "kickerTopServo");
        kickerBottomServo = hardwareMap.get(Servo.class, "kickerBottomServo");

        sorterLeftServo = hardwareMap.get(Servo.class, "sorterLeftServo");
        sorterRightServo = hardwareMap.get(Servo.class, "sorterRightServo");

        // Analog inputs (Robot Config names)
        kickerAnalog = hardwareMap.get(AnalogInput.class, "kickerAnalog");
        sorterAnalog = hardwareMap.get(AnalogInput.class, "sorterAnalog");

        sensorColorFront = hardwareMap.get(ColorSensor.class, "colourSensorFront");
        sensorDistanceFront = hardwareMap.get(DistanceSensor.class, "colourSensorFront");

        sensorColorBack = hardwareMap.get(ColorSensor.class, "colourSensorBack");
        sensorDistanceBack = hardwareMap.get(DistanceSensor.class, "colourSensorBack");

        // Start in intake pocket 1
        requestSorterMoveToIndexIntake(1);

        // Command kicker up initially
        kickerTopServo.setPosition(0.99);
        kickerBottomServo.setPosition(0.99);

        ballTimer.reset();
        shooterStableTimer.reset();

        // ----------------------------- TURRET SETUP -----------------------------
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretPositionMotor");
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPositionTolerance(TURRET_POSITION_TOLERANCE);
        turretMotor.setPositionPIDFCoefficients(TURRET_POSITION_P);

        // ----------------------------- LIMELIGHT SETUP -----------------------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ----------------------------- DRIVETRAIN -----------------------------
            drive.driveRobotCentric(
                    -gamepad1.left_stick_x * drive_speed,
                    gamepad1.left_stick_y * drive_speed,
                    -gamepad1.right_stick_x * drive_speed,
                    false
            );

            drive_speed = (gamepad1.right_trigger > 0.5) ? 0.45 : 1.0;

            // ----------------------------- INTAKE (disabled while shootAllRunning) -----------------------------
            if (!shootAllRunning) {
                if (gamepad1.triangle) {
                    intakeMotor.set(1);
                    intakeEnabled = true;
                }
                if (gamepad1.cross) {
                    intakeMotor.set(0);
                    intakeEnabled = false;
                }
                if (gamepad1.left_bumper) {
                    intakeMotor.set(-1);
                    intakeEnabled = false;
                }
            }

            // ----------------------------- MANUAL KICKER RESET -----------------------------
            if (gamepad1.dpad_left && gamepad1.left_trigger < 0.5) {
                kickerTopServo.setPosition(0.99);
                kickerBottomServo.setPosition(0.99);
                kickerPulseState = KickerPulseState.IDLE;
            }

            // ----------------------------- TURRET MANUAL NUDGE (kept as-is) -----------------------------
            if (gamepad1.dpad_left && gamepad1.left_trigger > 0.5) {
                turretPos = turretMotor.getCurrentPosition() + 10;
                turretMotor.setTargetPosition(turretPos);
                sleep(50);
            }
            if (gamepad1.dpad_right && gamepad1.left_trigger > 0.5) {
                turretPos = turretMotor.getCurrentPosition() - 10;
                turretMotor.setTargetPosition(turretPos);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(50);
            }

            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // ----------------------------- SHOOT ALL (NON-BLOCKING) -----------------------------
            boolean shootAllPressed = gamepad1.dpad_right && gamepad1.left_trigger < 0.5;

            if (shootAllPressed && !lastShootAllPressed && shooterReady && !shootAllRunning) {
                aimTurretAtAprilTag();

                // Remember intake state, then run intake slowly during outtake
                intakeWasEnabledBeforeShootAll = intakeEnabled;
                intakeMotor.set(OUTTAKE_INTAKE_POWER);

                shootAllRunning = true;
                shootAllStep = 0;
            }
            lastShootAllPressed = shootAllPressed;

            // While shoot-all is running, keep intake at 0.2
            if (shootAllRunning) {
                intakeMotor.set(OUTTAKE_INTAKE_POWER);
            }

            // ----------------------------- SHOOTER -----------------------------
            if (gamepad1.square) {
                targetVelocity = MAX_TICKS_PER_SEC * Variables.shooterSpeedMid;
                shooterAnglePos = Variables.shooterAngleMid;
                shooterAngleServo.setPosition(shooterAnglePos);
            }
            if (gamepad1.circle) {
                targetVelocity = MAX_TICKS_PER_SEC * Variables.shooterSpeedFar;
                shooterAnglePos = Variables.shooterAngleFar;
                shooterAngleServo.setPosition(shooterAnglePos);
            }

            if (gamepad1.dpad_up) targetVelocity += 2;
            if (gamepad1.dpad_down) targetVelocity -= 2;

            if (gamepad1.guide) {
                targetVelocity = 0;
                shooterAnglePos = 0;
                shooterAngleServo.setPosition(shooterAnglePos);
                intakeMotor.set(0.6);
            }

            if (gamepad1.share && shooterAnglePos > 0.05) {
                shooterAnglePos -= 0.02;
                shooterAngleServo.setPosition(shooterAnglePos);
                sleep(300);
            }

            if (gamepad1.options && shooterAnglePos < 0.45) {
                shooterAnglePos += 0.02;
                shooterAngleServo.setPosition(shooterAnglePos);
                sleep(300);
            }

            targetVelocity = Math.max(0, Math.min(MAX_TICKS_PER_SEC, targetVelocity));
            shooterLeft.setVelocity(targetVelocity);

            double power = targetVelocity / MAX_TICKS_PER_SEC;
            power = Math.max(0, Math.min(1, power));
            shooterRight.setPower(power);

            // Shooter stability check
            double measuredVel = shooterLeft.getVelocity();

            if (targetVelocity > 0 && Math.abs(measuredVel - targetVelocity) < SHOOTER_TOLERANCE_TICKS) {
                if (!shooterSpeedInRange) {
                    shooterSpeedInRange = true;
                    shooterStableTimer.reset();
                }
                shooterReady = shooterStableTimer.milliseconds() >= SHOOTER_STABLE_TIME_MS;
            } else {
                shooterSpeedInRange = false;
                shooterReady = false;
            }

            // ----------------------------- SORTER/KICKER UPDATES -----------------------------
            updateSorterMove();
            updateKickerPulse();

            if (shootAllRunning) {
                updateShootAllSequence();
            }

            // ----------------------------- BALL DETECTION + CLOSED LOOP SORTER ROTATION -----------------------------
            if (!shootAllRunning && intakeEnabled && sorterState < 3) {

                double distF = sensorDistanceFront.getDistance(DistanceUnit.CM);
                double distB = sensorDistanceBack.getDistance(DistanceUnit.CM);

                boolean ballDetected = (distF < BALL_DETECT_DISTANCE) || (distB < BALL_DETECT_DISTANCE);
                boolean allowNewBall = ballTimer.milliseconds() > BALL_COOLDOWN_MS;

                boolean newBall = ballDetected && !lastBallPresent && allowNewBall;

                // Start pending sequence (non-blocking)
                if (newBall && !pendingBallSequence) {
                    pendingColor = detectColourFromTwoSensors();
                    pendingSorterState = sorterState;
                    pendingBallSequence = true;
                    sorterDelayTimer.reset();
                }

                // After 150ms, command the next sorter move (and then we WAIT for analog to reach)
                if (pendingBallSequence && sorterDelayTimer.milliseconds() > 10) {

                    if (!sorterMoveActive) {
                        switch (pendingSorterState) {

                            case 0: // first ball goes into pocket 1, then rotate to pocket 2
                                ball1 = pendingColor;
                                requestSorterMoveToIndexIntake(2);
                                sorterState = 1;
                                break;

                            case 1: // second ball goes into pocket 2, then rotate to pocket 3
                                ball2 = pendingColor;
                                requestSorterMoveToIndexIntake(3);
                                sorterState = 2;
                                break;

                            case 2: // third ball stored; stop intake
                                ball3 = pendingColor;
                                sorterState = 3;
                                intakeMotor.set(0);
                                intakeEnabled = false;
                                break;
                        }

                        ballTimer.reset();
                        pendingBallSequence = false;
                    }
                }

                lastBallPresent = ballDetected;
            } else {
                lastBallPresent = false;
                pendingBallSequence = false;
            }

            // Cut turret power when it has reached its target
            if (!turretMotor.isBusy()) {
                turretMotor.setPower(0.2);
            } else {
                turretMotor.setPower(0.5);
            }

            // ----------------------------- TELEMETRY -----------------------------
            telemetry.addData("SorterState (balls stored)", sorterState);
            telemetry.addData("SorterPosIndex (1/2/3)", sorterPositionIndex);
            telemetry.addData("Ball1", colourToString(ball1));
            telemetry.addData("Ball2", colourToString(ball2));
            telemetry.addData("Ball3", colourToString(ball3));

            telemetry.addData("Target Vel", targetVelocity);
            telemetry.addData("Measured Vel", measuredVel);
            telemetry.addData("ShooterReady", shooterReady);

            telemetry.addData("KickerV", "%.3f", kickerAnalog.getVoltage());
            telemetry.addData("SorterV", "%.3f", sorterAnalog.getVoltage());
            telemetry.addData("SorterMoveActive", sorterMoveActive);
            telemetry.addData("KickerPulseState", kickerPulseState);
            telemetry.addData("ShootAllRunning", shootAllRunning);
            telemetry.addData("ShootAllStep", shootAllStep);

            if (!sorterMoveError.isEmpty()) telemetry.addLine("ERR: " + sorterMoveError);

            telemetry.addData("turret pos", turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    // ----------------------------- NON-BLOCKING: KICKER PULSE -----------------------------

    private boolean withinTol(double v, double target, double tol) {
        return Math.abs(v - target) <= tol;
    }

    private void startKickerPulse() {
        kickerPulseState = KickerPulseState.COMMAND_DOWN;
        kickerPulseTimer.reset();
    }

    private void updateKickerPulse() {
        if (kickerPulseState == KickerPulseState.IDLE) return;

        double v = kickerAnalog.getVoltage();

        switch (kickerPulseState) {

            case COMMAND_DOWN:
                kickerTopServo.setPosition(0.85);
                kickerBottomServo.setPosition(0.85);
                kickerPulseState = KickerPulseState.WAIT_DOWN;
                kickerPulseTimer.reset();
                break;

            case WAIT_DOWN:
                if (withinTol(v, KICKER_DOWN_V, KICKER_V_TOL) || kickerPulseTimer.milliseconds() > KICKER_TIMEOUT_MS) {
                    kickerPulseState = KickerPulseState.COMMAND_UP;
                }
                break;

            case COMMAND_UP:
                kickerTopServo.setPosition(0.99);
                kickerBottomServo.setPosition(0.99);
                kickerPulseState = KickerPulseState.WAIT_UP;
                kickerPulseTimer.reset();
                break;

            case WAIT_UP:
                if (withinTol(v, KICKER_UP_V, KICKER_V_TOL) || kickerPulseTimer.milliseconds() > KICKER_TIMEOUT_MS) {
                    kickerPulseState = KickerPulseState.IDLE;
                }
                break;

            case IDLE:
                break;
        }
    }

    private boolean kickerPulseFinished() {
        return kickerPulseState == KickerPulseState.IDLE;
    }

    // ----------------------------- SORTER VOLTAGE TARGETING -----------------------------

    private double intakeVoltageForIndex(int index) {
        switch (index) {
            case 1: return SORTER_INTAKE_1_V;
            case 2: return SORTER_INTAKE_2_V;
            case 3: return SORTER_INTAKE_3_V;
            default: return SORTER_INTAKE_1_V;
        }
    }

    private double outtakeVoltageForIndex(int index) {
        switch (index) {
            case 1: return SORTER_OUTTAKE_1_V;
            case 2: return SORTER_OUTTAKE_2_V;
            case 3: return SORTER_OUTTAKE_3_V;
            default: return SORTER_OUTTAKE_1_V;
        }
    }

    private void requestSorterMoveToIndexIntake(int index) {
        // Uses your Variables intake positions
        moveSorterToIndex(index);

        sorterTargetV = intakeVoltageForIndex(index);
        sorterMoveActive = true;
        sorterMoveTimer.reset();
        sorterMoveError = "";
    }

    private void requestSorterMoveToIndexOuttakeHardcoded(int index) {
        // Your shoot positions: 1->0.54, 2->0.93, 3->0.16
        if (index == 1) {
            sorterLeftServo.setPosition(0.54);
            sorterRightServo.setPosition(0.54 + Variables.sorterOffset);
        } else if (index == 2) {
            sorterLeftServo.setPosition(0.93);
            sorterRightServo.setPosition(0.93 + Variables.sorterOffset);
        } else {
            sorterLeftServo.setPosition(0.16);
            sorterRightServo.setPosition(0.16 + Variables.sorterOffset);
        }

        sorterTargetV = outtakeVoltageForIndex(index);
        sorterMoveActive = true;
        sorterMoveTimer.reset();
        sorterMoveError = "";
    }

    private void updateSorterMove() {
        if (!sorterMoveActive) return;

        double v = sorterAnalog.getVoltage();

        if (withinTol(v, sorterTargetV, SORTER_V_TOL)) {
            sorterMoveActive = false;
            return;
        }

        if (sorterMoveTimer.milliseconds() > SORTER_TIMEOUT_MS) {
            sorterMoveError = "SORTER TIMEOUT (check analog wire / tol / targetV)";
            sorterMoveActive = false;
        }
    }

    // ----------------------------- SHOOT ALL SEQUENCE (NON-BLOCKING) -----------------------------

    private void updateShootAllSequence() {
        // Steps:
        // 0 move to outtake1, 1 pulse, 2 move to outtake2, 3 pulse, 4 move to outtake3, 5 pulse, 6 reset & go intake1

        switch (shootAllStep) {

            case 0:
                requestSorterMoveToIndexOuttakeHardcoded(1);
                shootAllStep = 1;
                break;

            case 1:
                if (!sorterMoveActive) {
                    startKickerPulse();
                    shootAllStep = 2;
                }
                break;

            case 2:
                if (kickerPulseFinished()) {
                    requestSorterMoveToIndexOuttakeHardcoded(2);
                    shootAllStep = 3;
                }
                break;

            case 3:
                if (!sorterMoveActive) {
                    startKickerPulse();
                    shootAllStep = 4;
                }
                break;

            case 4:
                if (kickerPulseFinished()) {
                    requestSorterMoveToIndexOuttakeHardcoded(3);
                    shootAllStep = 5;
                }
                break;

            case 5:
                if (!sorterMoveActive) {
                    startKickerPulse();
                    shootAllStep = 6;
                }
                break;

            case 6:
                if (kickerPulseFinished()) {
                    // reset memory + return sorter to intake pocket 1
                    ball1 = ball2 = ball3 = -1;
                    sorterState = 0;

                    requestSorterMoveToIndexIntake(1);

                    shootAllRunning = false;

                    // Restore intake to previous state
                    if (intakeWasEnabledBeforeShootAll) {
                        intakeMotor.set(1.0);
                        intakeEnabled = true;
                    } else {
                        intakeMotor.set(0.0);
                        intakeEnabled = false;
                    }
                }
                break;
        }
    }

    // ----------------------------- EXISTING HELPERS (YOURS) -----------------------------

    // Move sorter to physical pocket and update index (INTAKE positions)
    private void moveSorterToIndex(int index) {
        switch (index) {
            case 1:
                sorterLeftServo.setPosition(Variables.sorter1Position);
                sorterRightServo.setPosition(Variables.sorter1Position + Variables.sorterOffset);
                break;
            case 2:
                sorterLeftServo.setPosition(Variables.sorter2Position);
                sorterRightServo.setPosition(Variables.sorter2Position + Variables.sorterOffset);
                break;
            case 3:
                sorterLeftServo.setPosition(Variables.sorter3Position);
                sorterRightServo.setPosition(Variables.sorter3Position + Variables.sorterOffset);
                break;
        }
        sorterPositionIndex = index;
    }

    private void aimTurretAtAprilTag() {
        LLResult result = limelight.getLatestResult();
        boolean hasTarget = result != null && result.isValid();
        if (!hasTarget) return;

        double tx = result.getTx();
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        int tag = tags.get(0).getFiducialId();

        double desiredAngleDeg = -tx;
        if (tag == 20) desiredAngleDeg += 7;
        if (tag == 24) desiredAngleDeg -= 7;

        if (desiredAngleDeg > TURRET_MAX_ANGLE_DEG) desiredAngleDeg = TURRET_MAX_ANGLE_DEG;
        if (desiredAngleDeg < -TURRET_MAX_ANGLE_DEG) desiredAngleDeg = -TURRET_MAX_ANGLE_DEG;

        int targetTicks = (int) Math.round(desiredAngleDeg * TURRET_TICKS_PER_DEGREE);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_MOVE_POWER);
    }

    private int detectColourFromTwoSensors() {

        double gOverRFront = (double) sensorColorFront.green() / Math.max(sensorColorFront.red(), 1);
        double gOverBFront = (double) sensorColorFront.green() / Math.max(sensorColorFront.blue(), 1);

        double gOverRBack = (double) sensorColorBack.green() / Math.max(sensorColorBack.red(), 1);
        double gOverBBack = (double) sensorColorBack.green() / Math.max(sensorColorBack.blue(), 1);

        if ((sensorColorFront.green() > sensorColorFront.red() &&
                sensorColorFront.green() > sensorColorFront.blue() &&
                gOverRFront > 1.7 && gOverBFront > 1.15)

                || (sensorColorBack.green() > sensorColorBack.red() &&
                sensorColorBack.green() > sensorColorBack.blue() &&
                gOverRBack > 1.9 && gOverBBack > 1.2)) {
            return 1;
        }

        if ((sensorColorFront.green() < sensorColorFront.blue() &&
                gOverRFront < 1.4 && gOverBFront < 0.8)

                || (sensorColorBack.green() < sensorColorBack.blue() &&
                gOverBBack < 0.8 && gOverRBack < 1.4)) {
            return 0;
        }

        return -1;
    }

    private String colourToString(int c) {
        switch (c) {
            case 1: return "Green";
            case 0: return "Purple";
            default: return "Unknown";
        }
    }
}
