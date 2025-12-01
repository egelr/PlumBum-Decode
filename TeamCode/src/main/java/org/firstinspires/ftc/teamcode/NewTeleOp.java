package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    // Shooter specs
    private static final int CPR = 28;
    private static final int MAX_RPM = 6000;
    private static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;  // ~2800
    private double targetVelocity = 0;

    // Shooter stability detection
    private final double SHOOTER_TOLERANCE_TICKS = 150.0;
    private final long SHOOTER_STABLE_TIME_MS = 300;
    private final ElapsedTime shooterStableTimer = new ElapsedTime();
    private boolean shooterSpeedInRange = false;
    private boolean shooterReady = false;

    // Sorter + transfer
    private Servo sorterLeftServo;
    private Servo sorterRightServo;
    private Servo transferOutputServo;

    // Color + distance sensors
    private ColorSensor sensorColorFront;
    private DistanceSensor sensorDistanceFront;
    private ColorSensor sensorColorBack;
    private DistanceSensor sensorDistanceBack;

    // Ball memory: 0 = purple, 1 = green, -1 = unknown
    private int ball1 = -1;
    private int ball2 = -1;
    private int ball3 = -1;

    // Filling state
    // 0 = waiting for ball 1
    // 1 = waiting for ball 2
    // 2 = waiting for ball 3
    // 3 = full
    private int sorterState = 0;

    // Ball presence detection
    private boolean lastBallPresent = false;

    // Intake state
    private boolean intakeEnabled = false;

    // D-pad right shoot-all edge detection
    private boolean lastShootAllPressed = false;

    // Sorter constants
    private final double sorterOffset = 0.05;          // right servo offset
    private final double BALL1_POS = 0;
    private final double BALL2_POS = 0.75;
    private final double BALL3_POS = 0.76;             // adjust if needed

    // Detection constants
    private final double BALL_DETECT_DISTANCE = 4.0;   // cm
    private final long BALL_COOLDOWN_MS = 700;         // ms between balls

    // Transfer positions
    private final double TRANSFER_DOWN_POS = 0.08;
    private final double TRANSFER_UP_POS   = 0.18;

    private final ElapsedTime ballTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // -----------------------------
        // 1. DRIVETRAIN SETUP
        // -----------------------------
        fL = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        fR = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        bL = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        bR = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        MecanumDrive drive = new MecanumDrive(fL, fR, bL, bR);

        // -----------------------------
        // 2. INTAKE + SHOOTER SETUP
        // -----------------------------
        intakeMotor = new Motor(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_435);

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterLeft.setVelocityPIDFCoefficients(50, 0.0, 0.001, 11.7);
        shooterRight.setVelocityPIDFCoefficients(50, 0.0, 0.001, 11.7);

        shooterAngleServo = hardwareMap.get(Servo.class, "shooterAngleServo");
        shooterAnglePos = 0.0;
        shooterAngleServo.setPosition(shooterAnglePos);

        // -----------------------------
        // 3. SERVOS + SENSORS SETUP
        // -----------------------------
        transferOutputServo = hardwareMap.get(Servo.class, "transferOutputServo");
        sorterLeftServo = hardwareMap.get(Servo.class, "sorterLeftServo");
        sorterRightServo = hardwareMap.get(Servo.class, "sorterRightServo");

        sensorColorFront = hardwareMap.get(ColorSensor.class, "colourSensorFront");
        sensorDistanceFront = hardwareMap.get(DistanceSensor.class, "colourSensorFront");

        sensorColorBack = hardwareMap.get(ColorSensor.class, "colourSensorBack");
        sensorDistanceBack = hardwareMap.get(DistanceSensor.class, "colourSensorBack");

        sorterLeftServo.setPosition(BALL1_POS);
        sorterRightServo.setPosition(BALL1_POS + sorterOffset);

        transferOutputServo.setPosition(TRANSFER_UP_POS);

        ballTimer.reset();
        shooterStableTimer.reset();

        telemetry.addLine("Ready. Press PLAY to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -----------------------------
            // 1. DRIVETRAIN
            // -----------------------------
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

            // -----------------------------
            // 2. INTAKE CONTROL
            // -----------------------------
            if (gamepad1.triangle) {          // intake forward ON
                intakeMotor.set(1);
                intakeEnabled = true;
            }
            if (gamepad1.circle) {            // intake OFF
                intakeMotor.set(0);
                intakeEnabled = false;
            }
            if (gamepad1.left_bumper) {       // reverse intake
                intakeMotor.set(-1);
                intakeEnabled = false;
            }

            // -----------------------------
            // 3. TRANSFER SERVO + SHOOT ALL
            // -----------------------------
            // D-pad left = manual DOWN
            if (gamepad1.dpad_left) {
                transferOutputServo.setPosition(TRANSFER_DOWN_POS);
            }

            // D-pad right (on press) = shoot balls 3,2,1 IF shooter is ready
            boolean shootAllPressed = gamepad1.dpad_right;
            if (shootAllPressed && !lastShootAllPressed) {

                if (shooterReady) {
                    // ----- BALL 3 -----
                    if (ball3 != -1) {
                        sorterLeftServo.setPosition(BALL3_POS);
                        sorterRightServo.setPosition(BALL3_POS + sorterOffset);
                        //sleep(200);

                        transferOutputServo.setPosition(TRANSFER_UP_POS);
                        sleep(200);
                        transferOutputServo.setPosition(TRANSFER_DOWN_POS);
                        sleep(200);
                    }

                    // ----- BALL 2 -----
                    if (ball2 != -1) {
                        sorterLeftServo.setPosition(BALL2_POS);
                        sorterRightServo.setPosition(BALL2_POS + sorterOffset);

                        sleep(200);

                        transferOutputServo.setPosition(TRANSFER_UP_POS);
                        sleep(200);
                        transferOutputServo.setPosition(TRANSFER_DOWN_POS);
                        sleep(200);
                    }

                    // ----- BALL 1 -----
                    if (ball1 != -1) {
                        sorterLeftServo.setPosition(BALL1_POS);
                        sorterRightServo.setPosition(BALL1_POS + sorterOffset);
                        sleep(200);

                        transferOutputServo.setPosition(TRANSFER_UP_POS);
                        sleep(200);
                        transferOutputServo.setPosition(TRANSFER_DOWN_POS);
                        sleep(200);
                    }

                    // Reset memory + sorter + transfer
                    ball1 = -1;
                    ball2 = -1;
                    ball3 = -1;
                    sorterState = 0;
                    sorterLeftServo.setPosition(BALL1_POS);
                    sorterRightServo.setPosition(BALL1_POS + sorterOffset);
                    transferOutputServo.setPosition(TRANSFER_UP_POS);
                }
            }
            lastShootAllPressed = shootAllPressed;

            // -----------------------------
            // 4. SHOOTER CONTROL
            // -----------------------------
            if (gamepad1.square) {
                targetVelocity = MAX_TICKS_PER_SEC * 0.52;
            }

            if (gamepad1.dpad_up) {
                targetVelocity += 0.5;
            }
            if (gamepad1.dpad_down) {
                targetVelocity -= 0.5;
            }

            // guide: stop shooter + reset angle + sorter neutral
            if (gamepad1.guide) {
                targetVelocity = 0;
                shooterAnglePos = 0;
                shooterAngleServo.setPosition(shooterAnglePos);
                sorterLeftServo.setPosition(BALL1_POS);
                sorterRightServo.setPosition(BALL1_POS + sorterOffset);            }

            // share: angle preset
            if (gamepad1.share) {
                shooterAnglePos = 0.5;
                shooterAngleServo.setPosition(shooterAnglePos);
            }

            // triangle: tilt up
            if (gamepad1.triangle && shooterAnglePos < 0.45) {
                shooterAnglePos += 0.05;
                shooterAngleServo.setPosition(shooterAnglePos);
                sleep(300);
            }

            // cross: tilt down
            if (gamepad1.cross && shooterAnglePos > 0.05) {
                shooterAnglePos -= 0.05;
                shooterAngleServo.setPosition(shooterAnglePos);
                sleep(300);
            }

            // Clamp shooter velocity
            targetVelocity = Math.max(0, Math.min(MAX_TICKS_PER_SEC, targetVelocity));

            shooterLeft.setVelocity(targetVelocity);
            shooterRight.setVelocity(targetVelocity);

            // -----------------------------
            // 4.1 SHOOTER STABILITY CHECK
            // -----------------------------
            double leftVel = shooterLeft.getVelocity();
            double rightVel = shooterRight.getVelocity();
            double avgVel = (leftVel + rightVel) / 2.0;

            if (targetVelocity > 0 && Math.abs(avgVel - targetVelocity) < SHOOTER_TOLERANCE_TICKS) {
                if (!shooterSpeedInRange) {
                    shooterSpeedInRange = true;
                    shooterStableTimer.reset();
                }
                shooterReady = shooterStableTimer.milliseconds() >= SHOOTER_STABLE_TIME_MS;
            } else {
                shooterSpeedInRange = false;
                shooterReady = false;
            }

            // -----------------------------
            // 5. BALL DETECTION + SORTER
            // -----------------------------
            if (intakeEnabled && sorterState < 3) {

                double distF = sensorDistanceFront.getDistance(DistanceUnit.CM);
                double distB = sensorDistanceBack.getDistance(DistanceUnit.CM);

                boolean ballDetected = (distF < BALL_DETECT_DISTANCE) || (distB < BALL_DETECT_DISTANCE);
                boolean allowNewBall = ballTimer.milliseconds() > BALL_COOLDOWN_MS;

                boolean newBall = ballDetected && !lastBallPresent && allowNewBall;

                int detectedColour = detectColourFromTwoSensors();

                switch (sorterState) {

                    case 0: // first ball
                        if (newBall) {
                            ball1 = detectedColour;
                            sorterLeftServo.setPosition(BALL1_POS);
                            sorterRightServo.setPosition(BALL1_POS + sorterOffset);                            sorterState = 1;
                            ballTimer.reset();
                        }
                        break;

                    case 1: // second ball
                        if (newBall) {
                            ball2 = detectedColour;
                            sorterLeftServo.setPosition(BALL2_POS);
                            sorterRightServo.setPosition(BALL2_POS + sorterOffset);
                            sorterState = 2;
                            ballTimer.reset();
                        }
                        break;

                    case 2: // third ball
                        if (newBall) {
                            ball3 = detectedColour;
                            sorterLeftServo.setPosition(BALL3_POS);
                            sorterRightServo.setPosition(BALL3_POS + sorterOffset );
                            sorterState = 3;
                            ballTimer.reset();

                            intakeMotor.set(0);
                            intakeEnabled = false;
                        }
                        break;
                }

                lastBallPresent = ballDetected;

                telemetry.addData("Sorter State", sorterState);
                telemetry.addData("Ball 1", colourToString(ball1));
                telemetry.addData("Ball 2", colourToString(ball2));
                telemetry.addData("Ball 3", colourToString(ball3));

                telemetry.addData("Dist Front (cm)", "%.2f", distF);
                telemetry.addData("Dist Back  (cm)", "%.2f", distB);
                telemetry.addData("Cooldown (ms left)",
                        Math.max(0, BALL_COOLDOWN_MS - (long) ballTimer.milliseconds()));

            } else {
                lastBallPresent = false;
            }

            // -----------------------------
            // 6. GENERAL TELEMETRY
            // -----------------------------
            telemetry.addData("Intake Enabled", intakeEnabled);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Avg Shooter Vel", (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0);
            telemetry.addData("Target RPM", (targetVelocity / CPR) * 60);
            telemetry.addData("Shooter Ready", shooterReady);
            telemetry.addData("Shooter Angle Pos", shooterAnglePos);

            telemetry.addData("Ball1", colourToString(ball1));
            telemetry.addData("Ball2", colourToString(ball2));
            telemetry.addData("Ball3", colourToString(ball3));
            telemetry.addData("SorterState", sorterState);

            telemetry.update();
        }
    }

    // -------------------------------
    // Helper: set sorter positions from base
    // -------------------------------

    // -------------------------------
    // Colour detection using 2 sensors
    // returns: 1 = green, 0 = purple, -1 = unknown
    // -------------------------------
    private int detectColourFromTwoSensors() {

        double gOverRFront = (double) sensorColorFront.green() /
                Math.max(sensorColorFront.red(), 1);
        double gOverBFront = (double) sensorColorFront.green() /
                Math.max(sensorColorFront.blue(), 1);

        double gOverRBack = (double) sensorColorBack.green() /
                Math.max(sensorColorBack.red(), 1);
        double gOverBBack = (double) sensorColorBack.green() /
                Math.max(sensorColorBack.blue(), 1);

        // GREEN (either sensor)
        if ((sensorColorFront.green() > sensorColorFront.red() &&
                sensorColorFront.green() > sensorColorFront.blue() &&
                gOverRFront > 1.7 && gOverBFront > 1.15)

                || (sensorColorBack.green() > sensorColorBack.red() &&
                sensorColorBack.green() > sensorColorBack.blue() &&
                gOverRBack > 1.9 && gOverBBack > 1.2)) {

            return 1;   // green
        }

        // PURPLE (either sensor)
        if ((sensorColorFront.green() < sensorColorFront.blue() &&
                gOverRFront < 1.4 && gOverBFront < 0.8)

                || (sensorColorBack.green() < sensorColorBack.blue() &&
                gOverRBack < 1.4 && gOverBBack < 0.8)) {

            return 0;   // purple
        }

        return -1;  // unknown
    }

    // -------------------------------
    // Helper for telemetry text
    // -------------------------------
    private String colourToString(int c) {
        switch (c) {
            case 1:  return "Green (1)";
            case 0:  return "Purple (0)";
            default: return "Unknown (-1)";
        }
    }
}
