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

    //Creating the variables for Motors
    private Motor fL, fR, bL, bR;
    private Motor intakeMotor;
    public double drive_speed = 1;

    private DcMotorEx shooterLeft, shooterRight;

    // Motor specs
    private static final int CPR = 28;             // encoder counts per rev (GoBILDA 6000RPM)
    private static final int MAX_RPM = 6000;
    private static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;  // ~2800

    private double targetVelocity = 0;

    // Sorter + sensors
    private Servo sorterLeftServo;
    private Servo sorterRightServo;
    private Servo transferOutputServo;

    private ColorSensor sensorColorFront;
    private DistanceSensor sensorDistanceFront;

    private ColorSensor sensorColorBack;
    private DistanceSensor sensorDistanceBack;

    // Ball memory: 0 = purple, 1 = green, -1 = unknown
    private int ball1 = -1;
    private int ball2 = -1;
    private int ball3 = -1;

    // Sorter state:
    // 0 = waiting for ball 1 (move to 0.375)
    // 1 = waiting for ball 2 (move to 0.76)
    // 2 = waiting for ball 3 (just record colour, stop + stop intake)
    // 3 = done
    private int sorterState = 0;

    // Edge detection for ball presence
    private boolean lastBallPresent = false;

    // Intake state
    private boolean intakeEnabled = false;  // true when intake running forward

    // Constants
    private final double x = 0.05;                // servo offset
    private final double BALL_DETECT_DISTANCE = 4.0;   // cm, tune this
    private final long BALL_COOLDOWN_MS = 700;         // ms between valid balls

    private final ElapsedTime ballTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Drivetrain Motors
        fL = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        fR = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        bL = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        bR = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //Creating the Mecanum Drivetrain
        MecanumDrive drive = new MecanumDrive(fL, fR, bL, bR);

        // Subsystem Motors
        intakeMotor = new Motor(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_435);
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // PIDF coefficients tuned for 6000 RPM
        shooterLeft.setVelocityPIDFCoefficients(50, 0.0, 0.001, 11.7);
        shooterRight.setVelocityPIDFCoefficients(50, 0.0, 0.001, 11.7);

        // Servos
        transferOutputServo = hardwareMap.get(Servo.class, "transferOutputServo");
        sorterLeftServo = hardwareMap.get(Servo.class, "sorterLeftServo");
        sorterRightServo = hardwareMap.get(Servo.class, "sorterRightServo");

        // Sensors
        sensorColorFront = hardwareMap.get(ColorSensor.class, "colourSensorFront");
        sensorDistanceFront = hardwareMap.get(DistanceSensor.class, "colourSensorFront");

        sensorColorBack = hardwareMap.get(ColorSensor.class, "colourSensorBack");
        sensorDistanceBack = hardwareMap.get(DistanceSensor.class, "colourSensorBack");

        // Initial sorter position
        sorterRightServo.setPosition(0 + x);
        sorterLeftServo.setPosition(0);

        ballTimer.reset();

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

            // Speed mode
            if (gamepad1.right_trigger > 0.5) {
                drive_speed = 0.45;
            } else {
                drive_speed = 1;
            }

            // -----------------------------
            // 2. INTAKE CONTROL
            // -----------------------------
            // Triangle = intake forward ON
            if (gamepad1.triangle) {
                intakeMotor.set(1);
                intakeEnabled = true;
            }
            // Circle = intake OFF
            if (gamepad1.circle) {
                intakeMotor.set(0);
                intakeEnabled = false;
            }
            // Left bumper = reverse intake (no detection while reversing)
            if (gamepad1.left_bumper) {
                intakeMotor.set(-1);
                intakeEnabled = false;
            }

            // -----------------------------
            // 3. TRANSFER + SHOOTER
            // -----------------------------
            if (gamepad1.square) {
                transferOutputServo.setPosition(0.08);
            }
            if (gamepad1.cross) {
                transferOutputServo.setPosition(0.18);
            }

            if (gamepad1.cross) {
                targetVelocity = MAX_TICKS_PER_SEC * 0.52;
            }

            if (gamepad1.guide) {
                targetVelocity = 0;
                sorterRightServo.setPosition(0 + x);
                sorterLeftServo.setPosition(0);
            }

            // Clamp target
            targetVelocity = Math.max(0, Math.min(MAX_TICKS_PER_SEC, targetVelocity));

            // Apply shooter velocity
            shooterLeft.setVelocity(targetVelocity);
            shooterRight.setVelocity(targetVelocity);

            // -----------------------------
            // 4. BALL DETECTION + SORTER
            // Only active while intakeEnabled == true
            // -----------------------------
            if (intakeEnabled && sorterState < 3) {

                // 4.1 Ball presence
                double distF = sensorDistanceFront.getDistance(DistanceUnit.CM);
                double distB = sensorDistanceBack.getDistance(DistanceUnit.CM);

                boolean ballDetected = (distF < BALL_DETECT_DISTANCE) || (distB < BALL_DETECT_DISTANCE);
                boolean allowNewBall = ballTimer.milliseconds() > BALL_COOLDOWN_MS;

                boolean newBall = ballDetected && !lastBallPresent && allowNewBall;

                // 4.2 Colour detection
                int detectedColour = detectColourFromTwoSensors();

                // 4.3 Sorter state machine
                switch (sorterState) {

                    case 0: // first ball
                        if (newBall) {
                            ball1 = detectedColour;

                            sorterRightServo.setPosition(0.375 + x);
                            sorterLeftServo.setPosition(0.375);

                            sorterState = 1;
                            ballTimer.reset();
                        }
                        break;

                    case 1: // second ball
                        if (newBall) {
                            ball2 = detectedColour;

                            sorterRightServo.setPosition(0.76 + x);
                            sorterLeftServo.setPosition(0.76);

                            sorterState = 2;
                            ballTimer.reset();
                        }
                        break;

                    case 2: // third ball
                        if (newBall) {
                            ball3 = detectedColour;

                            sorterState = 3;   // done
                            ballTimer.reset();

                            // Turn off intake automatically when 3rd ball in
                            intakeMotor.set(0);
                            intakeEnabled = false;
                        }
                        break;
                }

                lastBallPresent = ballDetected;

                // Telemetry related to balls
                telemetry.addData("Sorter State", sorterState);
                telemetry.addData("Ball 1", colourToString(ball1));
                telemetry.addData("Ball 2", colourToString(ball2));
                telemetry.addData("Ball 3", colourToString(ball3));

                telemetry.addData("Dist Front (cm)", "%.2f", distF);
                telemetry.addData("Dist Back  (cm)", "%.2f", distB);
                telemetry.addData("Cooldown (ms left)", Math.max(0,
                        BALL_COOLDOWN_MS - (long) ballTimer.milliseconds()));

            } else {
                // If intake not enabled, reset detection edge so next enable starts clean
                lastBallPresent = false;
            }

            // -----------------------------
            // 5. GENERAL TELEMETRY
            // -----------------------------
            telemetry.addData("Intake Enabled", intakeEnabled);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Target RPM", (targetVelocity / CPR) * 60);
            telemetry.addData("Left Vel", shooterLeft.getVelocity());
            telemetry.addData("Right Vel", shooterRight.getVelocity());

            telemetry.update();
        }
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
