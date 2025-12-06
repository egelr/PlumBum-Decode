package org.firstinspires.ftc.teamcode.hardware;


import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Variables;


/**
 * Sorter subsystem:
 *  - Controls sorterLeftServo + sorterRightServo (3 slots)
 *  - Reads front/back colour + distance sensors
 *  - Remembers colours of up to 3 balls:
 *      0 = purple, 1 = green, -1 = unknown
 *  - Has an Action to intake & load up to 3 balls with a 2s timeout.
 */
public class Sorter {


    // Servos
    private final Servo sorterLeftServo;
    private final Servo sorterRightServo;


    // Intake motor (controlled directly by this class in auto)
    private final DcMotorEx intakeMotor;


    // Sensors
    private final ColorSensor sensorColorFront;
    private final DistanceSensor sensorDistanceFront;
    private final ColorSensor sensorColorBack;
    private final DistanceSensor sensorDistanceBack;


    // Ball memory (0 purple, 1 green, -1 unknown)
    private int ball1 = -1, ball2 = -1, ball3 = -1;


    // 0 = first slot, 1 = second, 2 = third, 3 = full
    private int sorterState = 0;


    private boolean lastBallPresent = false;


    // Detection constants (same as TeleOp)
    private static final double BALL_DETECT_DISTANCE_CM = 2.0;
    private static final long BALL_COOLDOWN_MS = 200;


    private final ElapsedTime ballTimer = new ElapsedTime();


    // Non-blocking delay before rotating sorter after detection
    private final ElapsedTime sorterDelayTimer = new ElapsedTime();
    private boolean waitingForSorterMove = false;
    private int pendingSorterState = -1;
    private int pendingColor = -1;


    public Sorter(HardwareMap hardwareMap) {
        sorterLeftServo = hardwareMap.get(Servo.class, "sorterLeftServo");
        sorterRightServo = hardwareMap.get(Servo.class, "sorterRightServo");


        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");


        sensorColorFront = hardwareMap.get(ColorSensor.class, "colourSensorFront");
        sensorDistanceFront = hardwareMap.get(DistanceSensor.class, "colourSensorFront");


        sensorColorBack = hardwareMap.get(ColorSensor.class, "colourSensorBack");
        sensorDistanceBack = hardwareMap.get(DistanceSensor.class, "colourSensorBack");


        // Start at slot 1
        moveSorterToSlot(0);

        /*ball1 =0;
        ball2 =1;
        ball3 =0;*/

        ballTimer.reset();
        sorterDelayTimer.reset();
    }


    // --------------------------------------------------
    // PUBLIC API
    // --------------------------------------------------


    /** Reset ball memory and sorter position to slot 1. */
    public void reset() {
        ball1 = ball2 = ball3 = -1;
        sorterState = 0;
        waitingForSorterMove = false;
        pendingSorterState = -1;
        pendingColor = -1;
        lastBallPresent = false;
        ballTimer.reset();
        sorterDelayTimer.reset();
        moveSorterToSlot(0);
    }


    /**
     * Move sorter to a given slot:
     * 0 -> Variables.sorter1Position
     * 1 -> Variables.sorter2Position
     * 2 -> Variables.sorter3Position
     */
    public void moveSorterToSlot(int slotIndex) {
        double basePos;
        switch (slotIndex) {
            default:
            case 0:
                basePos = Variables.sorter1Position;
                break;
            case 1:
                basePos = Variables.sorter2Position;
                break;
            case 2:
                basePos = Variables.sorter3Position;
                break;
        }
        sorterLeftServo.setPosition(basePos);
        sorterRightServo.setPosition(basePos + Variables.sorterOffset);
    }


    // --- getters for balls and state ---


    public int getBall1() { return ball1; }
    public int getBall2() { return ball2; }
    public int getBall3() { return ball3; }


    public String getBall1String() { return colourToString(ball1); }
    public String getBall2String() { return colourToString(ball2); }
    public String getBall3String() { return colourToString(ball3); }


    public int getSorterState() { return sorterState; }
    public boolean isWaitingForSorterMove() { return waitingForSorterMove; }


    /**
     * Pattern as 3 chars:
     *   P = purple, G = green, U = unknown
     * Example: "PPG", "PGP", "GPP", "PUU", etc.
     */
    public String getPatternString() {
        return "" + colourToChar(ball1) + colourToChar(ball2) + colourToChar(ball3);
    }


    // --------------------------------------------------
    // MAIN AUTONOMOUS ACTION:
    //  - Turn intake ON
    //  - Try to load 3 balls
    //  - Stop early if timeout hits 2 seconds
    // --------------------------------------------------

    public class loadedBalls implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ball1 = 0;
            ball2 = 1;
            ball3 = 0;
        return false;
        }

    }
    public Action loadedBalls() {
        return new Sorter.loadedBalls();
    }


    public Action intakeAndLoadThree() {
        return new IntakeAndLoadThreeAction();
    }

    public class preset implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            reset();
            return false;
        }

    }

    public Action preset() {
        return new Sorter.preset();
    }


    private class IntakeAndLoadThreeAction implements Action {
        private boolean initialized = false;
        private final ElapsedTime timeoutTimer = new ElapsedTime();  // 2s max time


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {


            // ---------- INITIALIZATION ----------
            if (!initialized) {
                reset();
                intakeMotor.setPower(1.0);   // start intake
                timeoutTimer.reset();        // start 2s timer
                initialized = true;
            }


            // ---------- TIMEOUT CHECK ----------
            if (timeoutTimer.seconds() > 3.0) {
                // Stop intake and finish even if we don't have 3 balls
                intakeMotor.setPower(0.0);


                packet.put("TIMEOUT", true);
                packet.put("pattern", getPatternString());
                return false;
            }


            // ---------- NORMAL COMPLETION CHECK ----------
            if (sorterState >= 3) {
                intakeMotor.setPower(0.0);
                packet.put("pattern", getPatternString());
                return false;
            }


            // ---------- BALL DETECTION + SORTER LOGIC ----------


            double distF = sensorDistanceFront.getDistance(DistanceUnit.CM);
            double distB = sensorDistanceBack.getDistance(DistanceUnit.CM);


            boolean ballDetected =
                    (distF < BALL_DETECT_DISTANCE_CM) || (distB < BALL_DETECT_DISTANCE_CM);
            boolean allowNewBall = ballTimer.milliseconds() > BALL_COOLDOWN_MS;


            boolean newBall = ballDetected && !lastBallPresent && allowNewBall;


            // New ball detected → detect colour & start 150ms delay before moving sorter
            if (newBall && !waitingForSorterMove && sorterState < 3) {
                pendingColor = detectColourFromTwoSensors();
                pendingSorterState = sorterState;
                waitingForSorterMove = true;
                sorterDelayTimer.reset();
            }


            // After delay → move sorter and store colour
            if (waitingForSorterMove && sorterDelayTimer.milliseconds() > 50) {


                switch (pendingSorterState) {
                    case 0:
                        ball1 = pendingColor;
                        moveSorterToSlot(1);  // move to second slot
                        sorterState = 1;
                        break;


                    case 1:
                        ball2 = pendingColor;
                        moveSorterToSlot(2);  // move to third slot
                        sorterState = 2;
                        break;


                    case 2:
                        ball3 = pendingColor;
                        sorterState = 3;
                        // full → stop intake
                        intakeMotor.setPower(0.0);
                        break;
                }
                ballTimer.reset();
                waitingForSorterMove = false;
            }


            lastBallPresent = ballDetected;


            // ---------- TELEMETRY ----------
            packet.put("timeout_s", timeoutTimer.seconds());
            packet.put("dist_front_cm", distF);
            packet.put("dist_back_cm", distB);
            packet.put("ballDetected", ballDetected);
            packet.put("sorter_state", sorterState);
            packet.put("ball1", getBall1String());
            packet.put("ball2", getBall2String());
            packet.put("ball3", getBall3String());
            packet.put("pattern", getPatternString());
            packet.put("waiting", waitingForSorterMove);


            // keep running until full or timeout
            return true;
        }
    }


    // --------------------------------------------------
    // COLOUR HELPERS (same thresholds as TeleOp)
    // --------------------------------------------------


    private int detectColourFromTwoSensors() {


        double gOverRFront = (double) sensorColorFront.green()
                / Math.max(sensorColorFront.red(), 1);
        double gOverBFront = (double) sensorColorFront.green()
                / Math.max(sensorColorFront.blue(), 1);


        double gOverRBack = (double) sensorColorBack.green()
                / Math.max(sensorColorBack.red(), 1);
        double gOverBBack = (double) sensorColorBack.green()
                / Math.max(sensorColorBack.blue(), 1);


        // GREEN
        if ((sensorColorFront.green() > sensorColorFront.red() &&
                sensorColorFront.green() > sensorColorFront.blue() &&
                gOverRFront > 1.7 && gOverBFront > 1.15)


                || (sensorColorBack.green() > sensorColorBack.red() &&
                sensorColorBack.green() > sensorColorBack.blue() &&
                gOverRBack > 1.9 && gOverBBack > 1.2)) {
            return 1;
        }


        // PURPLE
        if ((sensorColorFront.green() < sensorColorFront.blue() &&
                gOverRFront < 1.4 && gOverBFront < 0.8)


                || (sensorColorBack.green() < sensorColorBack.blue() &&
                gOverRBack < 1.4 && gOverBBack < 0.8)) {
            return 0;
        }


        // UNKNOWN
        return -1;
    }


    private String colourToString(int c) {
        switch (c) {
            case 1: return "Green";
            case 0: return "Purple";
            default: return "Unknown";
        }
    }


    private char colourToChar(int c) {
        switch (c) {
            case 1: return 'G';
            case 0: return 'P';
            default: return 'U';
        }
    }
}

