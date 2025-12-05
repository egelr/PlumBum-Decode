package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "teleopnew/testing")
public class NewTeleOpOld extends LinearOpMode {

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
    private Servo transferOutputServo;

    // Sensors
    private ColorSensor sensorColorFront;
    private DistanceSensor sensorDistanceFront;
    private ColorSensor sensorColorBack;
    private DistanceSensor sensorDistanceBack;

    // Ball memory (0 purple, 1 green, -1 unknown)
    private int ball1 = -1, ball2 = -1, ball3 = -1;

    private int sorterState = 0;  // 0=ball1, 1=ball2, 2=ball3, 3=full
    private boolean lastBallPresent = false;
    private boolean intakeEnabled = false;

    private boolean lastShootAllPressed = false;

    // Detection constants
    private final double BALL_DETECT_DISTANCE = 2.0;
    private final long BALL_COOLDOWN_MS = 200;
    private final ElapsedTime ballTimer = new ElapsedTime();

    // NON-BLOCKING SORTER DELAY
    private final ElapsedTime sorterDelayTimer = new ElapsedTime();
    private boolean waitingForSorterMove = false;
    private int pendingSorterState = -1;
    private int pendingColor = -1;

    // ---------- TURRET + LIMELIGHT ----------
    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    // Turret positions (about ±90°)
    // - negative = 90° right (clockwise)
    // + positive = 90° left  (counterclockwise)
    private static final int TURRET_POS_CENTER = 0;
    private static final int TURRET_POS_RIGHT  = -232;
    private static final int TURRET_POS_LEFT   = 232;

    private static final double TURRET_MAX_ANGLE_DEG = 90.0;
    private static final double TURRET_TICKS_PER_DEGREE = 232.0 / 90.0;  // ≈2.58

    private static final int TURRET_POSITION_TOLERANCE = 5;
    public static double TURRET_POSITION_P = 5.0;  // SDK PID P gain
    private static final double TURRET_MOVE_POWER = 0.5;

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
        transferOutputServo = hardwareMap.get(Servo.class, "transferOutputServo");
        sorterLeftServo = hardwareMap.get(Servo.class, "sorterLeftServo");
        sorterRightServo = hardwareMap.get(Servo.class, "sorterRightServo");

        sensorColorFront = hardwareMap.get(ColorSensor.class, "colourSensorFront");
        sensorDistanceFront = hardwareMap.get(DistanceSensor.class, "colourSensorFront");

        sensorColorBack = hardwareMap.get(ColorSensor.class, "colourSensorBack");
        sensorDistanceBack = hardwareMap.get(DistanceSensor.class, "colourSensorBack");

        // Set neutral sorter position
        sorterLeftServo.setPosition(Variables.sorter1Position);
        sorterRightServo.setPosition(Variables.sorter1Position + Variables.sorterOffset);

        transferOutputServo.setPosition(Variables.transferDownPosition);

        ballTimer.reset();
        shooterStableTimer.reset();

        // ----------------------------- TURRET SETUP -----------------------------
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretPositionMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(TURRET_POS_CENTER);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPositionTolerance(TURRET_POSITION_TOLERANCE);
        turretMotor.setPositionPIDFCoefficients(TURRET_POSITION_P);

        // ----------------------------- LIMELIGHT SETUP -----------------------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100);    // how often we poll for data
        limelight.pipelineSwitch(0);     // AprilTag pipeline index
        limelight.start();               // start polling

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

            if (gamepad1.right_trigger > 0.5) {
                drive_speed = 0.45;
            } else {
                drive_speed = 1;
            }

            // ----------------------------- INTAKE -----------------------------
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

            // ----------------------------- TRANSFER & SHOOT ALL -----------------------------
            if (gamepad1.dpad_left) {
                transferOutputServo.setPosition(Variables.transferDownPosition);
            }

            boolean shootAllPressed = gamepad1.dpad_right;

            // When shoot-all is pressed and shooter is ready → AIM WITH LIMELIGHT, THEN SHOOT
            if (shootAllPressed && !lastShootAllPressed && shooterReady) {

                // 1) Aim turret at AprilTag (only now we "search" for tag)
                aimTurretAtAprilTag();

                // 2) Shoot balls in order (your original logic)
                if (ball3 != -1) {
                    sorterLeftServo.setPosition(Variables.sorter3Position);
                    sorterRightServo.setPosition(Variables.sorter3Position + Variables.sorterOffset);
                    transferShootPulse();
                }

                if (ball2 != -1) {
                    sorterLeftServo.setPosition(Variables.sorter2Position);
                    sorterRightServo.setPosition(Variables.sorter2Position + Variables.sorterOffset);
                    transferShootPulse();
                }

                if (ball1 != -1) {
                    sorterLeftServo.setPosition(Variables.sorter1Position);
                    sorterRightServo.setPosition(Variables.sorter1Position + Variables.sorterOffset);
                    transferShootPulse();
                }

                // Reset
                ball1 = ball2 = ball3 = -1;
                sorterState = 0;

                sorterLeftServo.setPosition(Variables.sorter1Position);
                sorterRightServo.setPosition(Variables.sorter1Position + Variables.sorterOffset);
            }
            lastShootAllPressed = shootAllPressed;

            // ----------------------------- SHOOTER -----------------------------
            if (gamepad1.square) {
                targetVelocity = MAX_TICKS_PER_SEC * Variables.shooterSpeedMid;
                shooterAnglePos = Variables.shooterAngleMid;
                shooterAngleServo.setPosition(shooterAnglePos);
            }
            if (gamepad1.circle ) {
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

            double power = targetVelocity / MAX_TICKS_PER_SEC;  // simple feedforward
            power = Math.max(0, Math.min(1, power));

            shooterRight.setPower(power);

            // Shooter stability check
            double measuredVel = shooterLeft.getVelocity();  // axle speed

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

            // ----------------------------- BALL DETECTION + NON-BLOCKING SORTER -----------------------------
            if (intakeEnabled && sorterState < 3) {

                double distF = sensorDistanceFront.getDistance(DistanceUnit.CM);
                double distB = sensorDistanceBack.getDistance(DistanceUnit.CM);

                boolean ballDetected = (distF < BALL_DETECT_DISTANCE) || (distB < BALL_DETECT_DISTANCE);
                boolean allowNewBall = ballTimer.milliseconds() > BALL_COOLDOWN_MS;

                boolean newBall = ballDetected && !lastBallPresent && allowNewBall;

                // New ball detected → start delay
                if (newBall && !waitingForSorterMove) {
                    pendingColor = detectColourFromTwoSensors();
                    pendingSorterState = sorterState;
                    waitingForSorterMove = true;
                    sorterDelayTimer.reset();
                }

                // After 150ms → move sorter
                if (waitingForSorterMove && sorterDelayTimer.milliseconds() > 150) {

                    switch (pendingSorterState) {

                        case 0: // first ball
                            ball1 = pendingColor;
                            sorterLeftServo.setPosition(Variables.sorter2Position);
                            sorterRightServo.setPosition(Variables.sorter2Position + Variables.sorterOffset);
                            sorterState = 1;
                            break;

                        case 1: // second ball
                            ball2 = pendingColor;
                            sorterLeftServo.setPosition(Variables.sorter3Position);
                            sorterRightServo.setPosition(Variables.sorter3Position + Variables.sorterOffset);
                            sorterState = 2;
                            break;

                        case 2: // third ball (NO ROTATION)
                            ball3 = pendingColor;
                            sorterState = 3;
                            intakeMotor.set(0);
                            intakeEnabled = false;
                            break;
                    }

                    ballTimer.reset();
                    waitingForSorterMove = false;
                }

                lastBallPresent = ballDetected;
            } else {
                lastBallPresent = false;
                waitingForSorterMove = false;
            }

            // Cut turret power when it has reached its target
            if (!turretMotor.isBusy()) {
                turretMotor.setPower(0);
            }

            // ----------------------------- TELEMETRY -----------------------------
            telemetry.addData("SorterState", sorterState);
            telemetry.addData("Ball1", colourToString(ball1));
            telemetry.addData("Ball2", colourToString(ball2));
            telemetry.addData("Ball3", colourToString(ball3));
            telemetry.addData("Target Vel", targetVelocity);
            telemetry.addData("Measured Vel", measuredVel);
            telemetry.addData("Target RPM", (targetVelocity / CPR) * 60);
            telemetry.addData("ShooterReady", shooterReady);
            telemetry.addData("Waiting Sorter Move", waitingForSorterMove);
            telemetry.update();
        }
    }

    // ----------------------------- HELPER FUNCTIONS -----------------------------

    /**
     * Use Limelight AprilTag result to aim turret.
     * Called ONLY when the shoot-all button is pressed and shooter is ready.
     */
    private void aimTurretAtAprilTag() {
        LLResult result = limelight.getLatestResult();
        boolean hasTarget = result != null && result.isValid();

        if (!hasTarget) return;

        double tx = result.getTx();  // horizontal offset (deg)

        // Limelight: tx > 0 => tag to the RIGHT of crosshair
        // Turret: negative ticks = right, positive = left
        // To turn turret toward tag, we negate tx:
        double desiredAngleDeg = -tx;   // if it goes the wrong way, change to: double desiredAngleDeg = tx;

        // Clamp angle
        if (desiredAngleDeg > TURRET_MAX_ANGLE_DEG)  desiredAngleDeg = TURRET_MAX_ANGLE_DEG;
        if (desiredAngleDeg < -TURRET_MAX_ANGLE_DEG) desiredAngleDeg = -TURRET_MAX_ANGLE_DEG;

        int targetTicks = (int) Math.round(desiredAngleDeg * TURRET_TICKS_PER_DEGREE);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_MOVE_POWER);
    }

    private void transferShootPulse() {
        sleep(500);
        transferOutputServo.setPosition(Variables.transferUpPosition);
        sleep(200);
        transferOutputServo.setPosition(Variables.transferDownPosition);
        sleep(200);
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
                gOverRBack < 1.4 && gOverBBack < 0.8)) {
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
