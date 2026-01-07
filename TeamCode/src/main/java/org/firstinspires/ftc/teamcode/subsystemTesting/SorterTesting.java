package org.firstinspires.ftc.teamcode.subsystemTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "SorterTesting")
public class SorterTesting extends LinearOpMode {

    // Sensors
    private ColorSensor sensorColorFront;
    private DistanceSensor sensorDistanceFront;

    private ColorSensor sensorColorBack;
    private DistanceSensor sensorDistanceBack;

    // Servos
    private Servo sorterRightServo;
    private Servo sorterLeftServo;

    // Ball memory: 0 = purple, 1 = green, -1 = unknown
    private int ball1 = -1;
    private int ball2 = -1;
    private int ball3 = -1;

    // Sorter state:
    // 0 = waiting for ball 1 (move to 0.375)
    // 1 = waiting for ball 2 (move to 0.76)
    // 2 = waiting for ball 3 (just record colour, stop)
    // 3 = done
    private int sorterState = 0;

    // Edge detection for ball presence
    private boolean lastBallPresent = false;

    // Constants
    private final double x = 0.05;              // servo offset
    private final double BALL_DETECT_DISTANCE = 2.0;   // cm, tune this
    private final long BALL_COOLDOWN_MS = 700;         // ms between valid balls

    private final ElapsedTime ballTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Map sensors (make sure names match your configuration)
        sensorColorFront = hardwareMap.get(ColorSensor.class, "colourSensorFront");
        sensorDistanceFront = hardwareMap.get(DistanceSensor.class, "colourSensorFront");

        sensorColorBack = hardwareMap.get(ColorSensor.class, "colourSensorBack");
        sensorDistanceBack = hardwareMap.get(DistanceSensor.class, "colourSensorBack");

        // Map servos (change names if needed)
        sorterRightServo = hardwareMap.get(Servo.class, "sorterRightServo");
        sorterLeftServo  = hardwareMap.get(Servo.class, "sorterLeftServo");

        // Start with sorter at neutral (0)
        sorterRightServo.setPosition(0 + x);
        sorterLeftServo.setPosition(0);

        ballTimer.reset();

        telemetry.addLine("Ready. Press PLAY to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -------------------------------
            // 1. Ball presence detection
            // -------------------------------
            double distF = sensorDistanceFront.getDistance(DistanceUnit.CM);
            double distB = sensorDistanceBack.getDistance(DistanceUnit.CM);

            boolean ballDetected = (distF < BALL_DETECT_DISTANCE) || (distB < BALL_DETECT_DISTANCE);
            boolean allowNewBall = ballTimer.milliseconds() > BALL_COOLDOWN_MS;

            // Rising edge + cooldown = new ball
            boolean newBall = ballDetected && !lastBallPresent && allowNewBall;

            // -------------------------------
            // 2. Colour detection (0/1/-1)
            // -------------------------------
            int detectedColour = detectColourFromTwoSensors();

            // -------------------------------
            // 3. Sorter state machine
            // -------------------------------
            switch (sorterState) {

                case 0: // first ball
                    if (newBall) {
                        ball1 = detectedColour;   // remember colour (can be -1 if unknown)

                        // move sorter to 0.375
                        sorterRightServo.setPosition(0.375 + x);
                        sorterLeftServo.setPosition(0.375);

                        sorterState = 1;
                        ballTimer.reset();
                    }
                    break;

                case 1: // second ball
                    if (newBall) {
                        ball2 = detectedColour;

                        // move sorter to 0.76
                        sorterRightServo.setPosition(0.76 + x);
                        sorterLeftServo.setPosition(0.76);

                        sorterState = 2;
                        ballTimer.reset();
                    }
                    break;

                case 2: // third ball
                    if (newBall) {
                        ball3 = detectedColour;

                        // stop – keep sorter at 0.76
                        sorterState = 3;
                        ballTimer.reset();
                    }
                    break;

                case 3:
                    // finished, do nothing
                    break;
            }

            // Update for edge detection
            lastBallPresent = ballDetected;

            // -------------------------------
            // 4. Telemetry
            // -------------------------------
            telemetry.addData("Sorter State", sorterState);
            telemetry.addData("Ball 1", colourToString(ball1));
            telemetry.addData("Ball 2", colourToString(ball2));
            telemetry.addData("Ball 3", colourToString(ball3));

            telemetry.addData("Detected now", colourToString(detectedColour));
            telemetry.addData("Ball present", ballDetected);
            telemetry.addData("Cooldown (ms)", BALL_COOLDOWN_MS - ballTimer.milliseconds());

            telemetry.addData("Dist Front (cm)", "%.2f", distF);
            telemetry.addData("Dist Back  (cm)", "%.2f", distB);

            telemetry.addData("Front RGB", "%d, %d, %d",
                    sensorColorFront.red(),
                    sensorColorFront.green(),
                    sensorColorFront.blue());

            telemetry.addData("Back  RGB", "%d, %d, %d",
                    sensorColorBack.red(),
                    sensorColorBack.green(),
                    sensorColorBack.blue());

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
