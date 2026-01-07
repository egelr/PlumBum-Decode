package org.firstinspires.ftc.teamcode.teleOpModes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Variables;

@TeleOp(name = "TwoDriverTeleOp")
public class TwoDriverTeleOp extends LinearOpMode {

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

    // Pattern buttons edge detection (gamepad2)
    private boolean lastGPPPressed = false;  // square
    private boolean lastPGPPressed = false;  // triangle
    private boolean lastPPGPressed = false;  // circle

    // Detection constants
    private final double BALL_DETECT_DISTANCE = 4.0;   // cm
    private final long BALL_COOLDOWN_MS = 700;         // ms between balls

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

        shooterLeft.setDirection(DcMotorEx.Direction.REVERSE);
        shooterLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setVelocityPIDFCoefficients(50, 0.0, 0.001, 11.7);

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

        sorterLeftServo.setPosition(Variables.sorter1Position);
        sorterRightServo.setPosition(Variables.sorter1Position + Variables.sorterOffset);

        transferOutputServo.setPosition(Variables.transferDownPosition);

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
            // 3. TRANSFER SERVO + SHOOT ALL (GAMEPAD1)
            // -----------------------------
            // D-pad left = manual DOWN
            if (gamepad1.dpad_left) {
                transferOutputServo.setPosition(Variables.transferDownPosition);
            }

            // D-pad right (on press) = shoot balls 3,2,1 IF shooter is ready
            boolean shootAllPressed = gamepad1.dpad_right;
            if (shootAllPressed && !lastShootAllPressed) {

                if (shooterReady) {
                    // ----- BALL 3 -----
                    if (ball3 != -1) {
                        moveSorterToSlot(3);
                        //sleep(200);

                        transferOutputServo.setPosition(Variables.transferUpPosition);
                        sleep(200);
                        transferOutputServo.setPosition(Variables.transferDownPosition);
                        sleep(200);
                    }

                    // ----- BALL 2 -----
                    if (ball2 != -1) {
                        moveSorterToSlot(2);
                        sleep(200);

                        transferOutputServo.setPosition(Variables.transferUpPosition);
                        sleep(200);
                        transferOutputServo.setPosition(Variables.transferDownPosition);
                        sleep(200);
                    }

                    // ----- BALL 1 -----
                    if (ball1 != -1) {
                        moveSorterToSlot(1);
                        sleep(200);

                        transferOutputServo.setPosition(Variables.transferUpPosition);
                        sleep(200);
                        transferOutputServo.setPosition(Variables.transferDownPosition);
                        sleep(200);
                    }

                    // Reset memory + sorter + transfer
                    ball1 = -1;
                    ball2 = -1;
                    ball3 = -1;
                    sorterState = 0;
                    moveSorterToSlot(1);
                    transferOutputServo.setPosition(Variables.transferUpPosition);
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
                moveSorterToSlot(1);
            }

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
                            moveSorterToSlot(1);
                            sorterState = 1;
                            ballTimer.reset();
                        }
                        break;

                    case 1: // second ball
                        if (newBall) {
                            ball2 = detectedColour;
                            moveSorterToSlot(2);
                            sorterState = 2;
                            ballTimer.reset();
                        }
                        break;

                    case 2: // third ball
                        if (newBall) {
                            ball3 = detectedColour;
                            moveSorterToSlot(3);
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
            // 6. PATTERN SHOOTING (GAMEPAD2)
            // Patterns: GPP, PGP, PPG
            // 0 = purple, 1 = green
            // -----------------------------
            boolean gppPressed = gamepad2.square;    // GPP: [1,0,0]
            boolean pgpPressed = gamepad2.triangle;  // PGP: [0,1,0]
            boolean ppgPressed = gamepad2.circle;    // PPG: [0,0,1]

            if (gppPressed && !lastGPPPressed && shooterReady) {
                shootPattern(new int[]{1, 0, 0});   // GPP
            }
            if (pgpPressed && !lastPGPPressed && shooterReady) {
                shootPattern(new int[]{0, 1, 0});   // PGP
            }
            if (ppgPressed && !lastPPGPressed && shooterReady) {
                shootPattern(new int[]{0, 0, 1});   // PPG
            }

            lastGPPPressed = gppPressed;
            lastPGPPressed = pgpPressed;
            lastPPGPressed = ppgPressed;

            // -----------------------------
            // 7. GENERAL TELEMETRY
            // -----------------------------
            telemetry.addData("Intake Enabled", intakeEnabled);
            telemetry.addData("Target Velocity", targetVelocity);
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
    // Helper: move sorter to a given slot (1,2,3)
    // -------------------------------
    private void moveSorterToSlot(int slot) {
        double basePos;
        if (slot == 1)      basePos = Variables.sorter1Position;
        else if (slot == 2) basePos = Variables.sorter2Position;
        else                basePos = Variables.sorter3Position;

        sorterLeftServo.setPosition(basePos);
        sorterRightServo.setPosition(basePos + Variables.sorterOffset);
    }

    // -------------------------------
    // Helper: shoot a specific slot once (UP -> DOWN pulse)
    // -------------------------------
    private void shootSlot(int slot) {
        moveSorterToSlot(slot);
        sleep(200);

        transferOutputServo.setPosition(Variables.transferUpPosition);
        sleep(200);
        transferOutputServo.setPosition(Variables.transferDownPosition);
        sleep(200);

        if (slot == 1)      ball1 = -1;
        else if (slot == 2) ball2 = -1;
        else                ball3 = -1;
    }

    // -------------------------------
    // Helper: shoot according to pattern (array of 3 colours)
    // patternColours: e.g. [1,0,0] = GPP
    // Uses current ball1/2/3 (2 P, 1 G in random order)
    // -------------------------------
    private void shootPattern(int[] patternColours) {
        // Copy ball colours into array for easier handling
        int[] balls = new int[]{ball1, ball2, ball3};
        boolean[] used = new boolean[]{false, false, false};
        int[] slotsToShoot = new int[3];

        // For each required colour in pattern, find a matching slot
        for (int p = 0; p < 3; p++) {
            int neededColour = patternColours[p];
            int chosenSlot = -1;

            for (int s = 0; s < 3; s++) {
                if (!used[s] && balls[s] == neededColour) {
                    chosenSlot = s + 1;   // slots are 1-based
                    used[s] = true;
                    break;
                }
            }

            if (chosenSlot == -1) {
                // Pattern can't be formed from current balls -> abort
                return;
            }

            slotsToShoot[p] = chosenSlot;
        }

        // If we get here, we have a valid mapping.
        // Shoot in the order determined by slotsToShoot
        for (int i = 0; i < 3; i++) {
            int slot = slotsToShoot[i];
            shootSlot(slot);
        }

        // After pattern is fully shot, reset sorter state
        sorterState = 0;
        moveSorterToSlot(1);
        transferOutputServo.setPosition(Variables.transferUpPosition);
    }

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
