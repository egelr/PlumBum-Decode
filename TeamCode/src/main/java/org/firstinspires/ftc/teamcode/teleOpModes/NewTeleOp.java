package org.firstinspires.ftc.teamcode.teleOpModes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

    // ----------------------------- DRIVETRAIN -----------------------------
    private Motor fL, fR, bL, bR;
    private Motor intakeMotor;
    public double drive_speed = 1;

    // ----------------------------- SHOOTER -----------------------------
    private DcMotorEx shooterLeft, shooterRight;
    private Servo shooterAngleServo;
    private double shooterAnglePos = 0.0;

    private static final int CPR = 28;
    private static final int MAX_RPM = 6000;
    private static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;
    private double targetVelocity = 0;

    private final double SHOOTER_TOLERANCE_TICKS = 150.0;
    private final long SHOOTER_STABLE_TIME_MS = 300;
    private final ElapsedTime shooterStableTimer = new ElapsedTime();
    private boolean shooterSpeedInRange = false;
    private boolean shooterReady = false;

    // ----------------------------- SORTER + KICKER -----------------------------
    private Servo sorterLeftServo, sorterRightServo;
    private Servo kickerTopServo, kickerBottomServo;

    private AnalogInput kickerAnalog, sorterAnalog;

    private static final double KICKER_DOWN_V = 3.0025;
    private static final double KICKER_UP_V   = 2.7495;

    private static final double SORTER_INTAKE_1_V  = 0.2200;
    private static final double SORTER_INTAKE_2_V  = 1.2850;
    private static final double SORTER_INTAKE_3_V  = 2.3640;

    private static final double SORTER_OUTTAKE_1_V = 1.7425;
    private static final double SORTER_OUTTAKE_2_V = 2.8405;
    private static final double SORTER_OUTTAKE_3_V = 0.6715;

    private static final double KICKER_V_TOL = 0.030;
    private static final double SORTER_V_TOL = 0.030;

    private static final long KICKER_TIMEOUT_MS = 350;
    private static final long SORTER_TIMEOUT_MS = 450;

    private static final long KICKER_STABLE_MS = 25;
    private static final long KICKER_MIN_DOWN_MS = 70;

    private static final double OUTTAKE_INTAKE_POWER = 0.4;
    private boolean intakeWasEnabledBeforeShootAll = false;

    // ----------------------------- SENSORS -----------------------------
    private ColorSensor sensorColorFront, sensorColorBack;
    private DistanceSensor sensorDistanceFront, sensorDistanceBack;

    private int ball1 = -1, ball2 = -1, ball3 = -1;
    private int sorterState = 0;
    private int sorterPositionIndex = 1;
    private boolean far = false;

    private boolean lastBallPresent = false;
    private boolean intakeEnabled = false;
    private boolean lastShootAllPressed = false;

    private final double BALL_DETECT_DISTANCE = 3.0;
    private final long BALL_COOLDOWN_MS = 100;
    private final ElapsedTime ballTimer = new ElapsedTime();

    private final ElapsedTime sorterDelayTimer = new ElapsedTime();
    private boolean pendingBallSequence = false;
    private int pendingSorterState = -1;
    private int pendingColor = -1;

    private boolean sorterMoveActive = false;
    private double sorterTargetV = 0.0;
    private final ElapsedTime sorterMoveTimer = new ElapsedTime();
    private String sorterMoveError = "";

    // ----------------------------- KICKER STATE MACHINE -----------------------------
    private enum KickerPulseState { IDLE, COMMAND_DOWN, WAIT_DOWN, COMMAND_UP, WAIT_UP }
    private KickerPulseState kickerPulseState = KickerPulseState.IDLE;

    private final ElapsedTime kickerPulseTimer = new ElapsedTime();
    private final ElapsedTime kickerStableTimer = new ElapsedTime();
    private boolean kickerStableRunning = false;

    private boolean shootAllRunning = false;
    private int shootAllStep = 0;

    // ----------------------------- TURRET + LIMELIGHT (EXACT LIKE YOUR EXAMPLE) -----------------------------
    private Servo turretServo1, turretServo2;
    private Limelight3A limelight;

    // turretDeg = servoDeg * (35/84)  => servoDeg = turretDeg / (35/84)
    private static final double GEAR_RATIO =  (35.0 / 84.0) * 2;

    private static final double DIR = -1.0; //reversed

    private static final int TARGET_TAG_ID = -1; // -1 = first seen

    // Vision telemetry
    private boolean tagVisible = false;
    private int currentTagId = -1;
    private double lastTx = 0.0;
    private double lastArea = 0.0;

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

        // ----------------------------- SORTER + KICKER -----------------------------
        kickerTopServo = hardwareMap.get(Servo.class, "kickerTopServo");
        kickerBottomServo = hardwareMap.get(Servo.class, "kickerBottomServo");

        sorterLeftServo = hardwareMap.get(Servo.class, "sorterLeftServo");
        sorterRightServo = hardwareMap.get(Servo.class, "sorterRightServo");

        kickerAnalog = hardwareMap.get(AnalogInput.class, "kickerAnalog");
        sorterAnalog = hardwareMap.get(AnalogInput.class, "sorterAnalog");

        sensorColorFront = hardwareMap.get(ColorSensor.class, "colourSensorFront");
        sensorDistanceFront = hardwareMap.get(DistanceSensor.class, "colourSensorFront");

        sensorColorBack = hardwareMap.get(ColorSensor.class, "colourSensorBack");
        sensorDistanceBack = hardwareMap.get(DistanceSensor.class, "colourSensorBack");

        /*requestSorterMoveToIndexIntake(1);

        kickerTopServo.setPosition(0.99);
        kickerBottomServo.setPosition(0.99);*/

        ballTimer.reset();
        shooterStableTimer.reset();

        // ----------------------------- TURRET -----------------------------
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");


        // ----------------------------- LIMELIGHT -----------------------------
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

            // ----------------------------- INTAKE -----------------------------
            if (!shootAllRunning) {
                if (gamepad1.triangle) { intakeMotor.set(1); sorterLeftServo.setPosition(0); sorterRightServo.setPosition(0 + Variables.sorterOffset); intakeEnabled = true; }
                if (gamepad1.cross)    { intakeMotor.set(0); intakeEnabled = false; }
                if (gamepad1.left_bumper) { intakeMotor.set(-1); intakeEnabled = false; }
            }

            // ----------------------------- MANUAL KICKER RESET -----------------------------
            if (gamepad1.dpad_left && gamepad1.left_trigger < 0.5) {
                kickerTopServo.setPosition(0.99);
                kickerBottomServo.setPosition(0.99);
                kickerPulseState = KickerPulseState.IDLE;
                kickerStableRunning = false;
            }

            // ----------------------------- SHOOT ALL -----------------------------
            boolean shootAllPressed = gamepad1.dpad_right && gamepad1.left_trigger < 0.5;

            if (shootAllPressed && !lastShootAllPressed && shooterReady && !shootAllRunning) {

                // EXACT one-time detection + aim like your example
                aimAtTagSnap_ExactlyLikeExample();

                intakeWasEnabledBeforeShootAll = intakeEnabled;
                intakeMotor.set(OUTTAKE_INTAKE_POWER);

                shootAllRunning = true;
                shootAllStep = 0;
            }
            lastShootAllPressed = shootAllPressed;

            if (shootAllRunning) {
                intakeMotor.set(OUTTAKE_INTAKE_POWER);
            }

            // ----------------------------- SHOOTER -----------------------------
            if (gamepad1.square) {
                targetVelocity = MAX_TICKS_PER_SEC * Variables.shooterSpeedMid;
                shooterAnglePos = Variables.shooterAngleMid;
                shooterAngleServo.setPosition(shooterAnglePos);
                far = false;
            }
            if (gamepad1.circle) {
                targetVelocity = MAX_TICKS_PER_SEC * Variables.shooterSpeedFar;
                shooterAnglePos = Variables.shooterAngleFar;
                shooterAngleServo.setPosition(shooterAnglePos);
                far = true;
            }

            if (gamepad1.dpad_up) targetVelocity += 2;
            if (gamepad1.dpad_down) targetVelocity -= 2;

            if (gamepad1.guide) {
                targetVelocity = 0;
                shooterAnglePos = 0;
                shooterAngleServo.setPosition(shooterAnglePos);
                turretServo1.setPosition(0.5);
                turretServo2.setPosition(0.5);
                kickerTopServo.setPosition(0.99);
                kickerBottomServo.setPosition(0.99);
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

            // ----------------------------- BALL DETECTION -----------------------------
            if (!shootAllRunning && intakeEnabled && sorterState < 3) {

                double distF = sensorDistanceFront.getDistance(DistanceUnit.CM);
                double distB = sensorDistanceBack.getDistance(DistanceUnit.CM);

                boolean ballDetected = (distF < BALL_DETECT_DISTANCE) || (distB < BALL_DETECT_DISTANCE);
                boolean allowNewBall = ballTimer.milliseconds() > BALL_COOLDOWN_MS;
                boolean newBall = ballDetected && !lastBallPresent && allowNewBall;

                if (newBall && !pendingBallSequence) {
                    pendingColor = detectColourFromTwoSensors();
                    pendingSorterState = sorterState;
                    pendingBallSequence = true;
                    sorterDelayTimer.reset();
                }

                if (pendingBallSequence && sorterDelayTimer.milliseconds() > 10) {
                    if (!sorterMoveActive) {
                        switch (pendingSorterState) {
                            case 0:
                                ball1 = pendingColor;
                                requestSorterMoveToIndexIntake(2);
                                sorterState = 1;
                                break;
                            case 1:
                                ball2 = pendingColor;
                                requestSorterMoveToIndexIntake(3);
                                sorterState = 2;
                                break;
                            case 2:
                                ball3 = pendingColor;
                                sorterState = 3;
                                sorterLeftServo.setPosition(0.54);
                                sorterRightServo.setPosition(0.54 + Variables.sorterOffset);
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

            // ----------------------------- TELEMETRY -----------------------------
            telemetry.addData("SorterState", sorterState);
            telemetry.addData("SorterIndex", sorterPositionIndex);
            telemetry.addData("Ball1", colourToString(ball1));
            telemetry.addData("Ball2", colourToString(ball2));
            telemetry.addData("Ball3", colourToString(ball3));

            telemetry.addData("TargetVel", targetVelocity);
            telemetry.addData("MeasuredVel", measuredVel);
            telemetry.addData("ShooterReady", shooterReady);

            telemetry.addData("KickerV", "%.3f", kickerAnalog.getVoltage());
            telemetry.addData("SorterV", "%.3f", sorterAnalog.getVoltage());
            telemetry.addData("SorterMoveActive", sorterMoveActive);
            telemetry.addData("KickerPulseState", kickerPulseState);

            telemetry.addData("ShootAllRunning", shootAllRunning);
            telemetry.addData("ShootAllStep", shootAllStep);

            telemetry.addData("TagVisible", tagVisible);
            telemetry.addData("TagID", currentTagId);
            telemetry.addData("tx", "%.2f", lastTx);
            telemetry.addData("area", "%.4f", lastArea);

            if (!sorterMoveError.isEmpty()) telemetry.addLine("ERR: " + sorterMoveError);

            telemetry.update();
        }
    }

    // ----------------------------- TURRET AIM (EXACT LIKE YOUR EXAMPLE) -----------------------------

    private void aimAtTagSnap_ExactlyLikeExample() {

        tagVisible = false;
        currentTagId = -1;
        lastTx = 0.0;
        lastArea = 0.0;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            // same behavior: just return
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return;
        }

        // Pick tag EXACTLY like your example
        LLResultTypes.FiducialResult tag = null;
        if (TARGET_TAG_ID < 0) {
            tag = fiducials.get(0);
        } else {
            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f != null && f.getFiducialId() == TARGET_TAG_ID) {
                    tag = f;
                    break;
                }
            }
        }

        if (tag == null) {
            return;
        }

        tagVisible = true;
        currentTagId = tag.getFiducialId();

        // Horizontal angle error to tag (degrees). Want tx -> 0 when centered.
        double txDeg = tag.getTargetXDegrees();
        lastTx = txDeg;
        lastArea = tag.getTargetArea();

        // Convert txDeg into a turret correction (EXACT)
        double turretDeg = (-txDeg) * DIR;
        int tagid = tag.getFiducialId();
        if (tagid == 20 && far == false) turretDeg -= 4;
        if (tagid == 24 && far == false) turretDeg += 4;
        if (tagid == 20 && far == true) turretDeg -= 2;
        if (tagid == 24 && far == true) turretDeg += 2;

        // turretDeg -> servoDeg using gear ratio (EXACT)
        double servoDeg = turretDeg / GEAR_RATIO / 360.0;

        // 360° servo: degrees -> position 0..1 (EXACT)
        double servoPos = turretServo1.getPosition() + servoDeg;

        turretServo1.setPosition(servoPos);
        turretServo2.setPosition(servoPos);
    }

    // ----------------------------- KICKER: STABLE-IN-TOLERANCE CHECK -----------------------------

    private boolean withinTol(double v, double target, double tol) {
        return Math.abs(v - target) <= tol;
    }

    private boolean stableAtTarget(double v, double target, double tol) {
        boolean inTol = withinTol(v, target, tol);

        if (inTol) {
            if (!kickerStableRunning) {
                kickerStableRunning = true;
                kickerStableTimer.reset();
            }
            return kickerStableTimer.milliseconds() >= KICKER_STABLE_MS;
        } else {
            kickerStableRunning = false;
            return false;
        }
    }

    private void startKickerPulse() {
        kickerPulseState = KickerPulseState.COMMAND_DOWN;
        kickerPulseTimer.reset();
        kickerStableRunning = false;
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
                kickerStableRunning = false;
                break;

            case WAIT_DOWN: {
                boolean reachedDownStable = stableAtTarget(v, KICKER_DOWN_V, KICKER_V_TOL);
                boolean minDownTimeOk = kickerPulseTimer.milliseconds() >= KICKER_MIN_DOWN_MS;
                boolean timedOut = kickerPulseTimer.milliseconds() > KICKER_TIMEOUT_MS;

                if ((reachedDownStable && minDownTimeOk) || timedOut) {
                    kickerPulseState = KickerPulseState.COMMAND_UP;
                    kickerStableRunning = false;
                }
                break;
            }

            case COMMAND_UP:
                kickerTopServo.setPosition(0.99);
                kickerBottomServo.setPosition(0.99);
                kickerPulseState = KickerPulseState.WAIT_UP;
                kickerPulseTimer.reset();
                kickerStableRunning = false;
                break;

            case WAIT_UP: {
                boolean reachedUpStable = stableAtTarget(v, KICKER_UP_V, KICKER_V_TOL);
                boolean timedOut = kickerPulseTimer.milliseconds() > KICKER_TIMEOUT_MS;

                if (reachedUpStable || timedOut) {
                    kickerPulseState = KickerPulseState.IDLE;
                    kickerStableRunning = false;
                }
                break;
            }

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
        moveSorterToIndex(index);

        sorterTargetV = intakeVoltageForIndex(index);
        sorterMoveActive = true;
        sorterMoveTimer.reset();
        sorterMoveError = "";
    }

    private void requestSorterMoveToIndexOuttakeHardcoded(int index) {
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

    // ----------------------------- SHOOT ALL SEQUENCE -----------------------------

    private void updateShootAllSequence() {
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
                    ball1 = ball2 = ball3 = -1;
                    sorterState = 0;

                    requestSorterMoveToIndexIntake(1);

                    shootAllRunning = false;
                    intakeMotor.set(1.0);
                    intakeEnabled = true;
                    turretServo1.setPosition(0.5);
                    turretServo2.setPosition(0.5);
                    targetVelocity = 0;

                }
                break;
        }
    }

    // ----------------------------- SORTER INTAKE POSITIONS -----------------------------

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

    // ----------------------------- COLOR DETECTION -----------------------------

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
            return 1; // Green
        }

        if ((sensorColorFront.green() < sensorColorFront.blue() &&
                gOverRFront < 1.4 && gOverBFront < 0.8)
                || (sensorColorBack.green() < sensorColorBack.blue() &&
                gOverBBack < 0.8 && gOverRBack < 1.4)) {
            return 0; // Purple
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
