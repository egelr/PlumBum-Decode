package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
 *  - Has an Action to intake & load up to 3 balls with a timeout.
 */
public class Sorter {

    // Servos
    private final Servo sorterLeftServo;
    private final Servo sorterRightServo;

    // Intake motor
    private final DcMotorEx intakeMotor;

    // Sensors
    private final ColorSensor sensorColorFront;
    private final DistanceSensor sensorDistanceFront;
    private final ColorSensor sensorColorBack;
    private final DistanceSensor sensorDistanceBack;

    // Analog feedback for sorter position
    private final AnalogInput sorterAnalog;

    // Ball memory (0 purple, 1 green, -1 unknown)
    private int ball1 = -1, ball2 = -1, ball3 = -1;

    // 0 = first slot, 1 = second, 2 = third, 3 = full
    private int sorterState = 0;

    private boolean lastBallPresent = false;

    // Detection constants
    private static final double BALL_DETECT_DISTANCE_CM = 2.0;
    private static final long BALL_COOLDOWN_MS = 200;

    private final ElapsedTime ballTimer = new ElapsedTime();

    // Non-blocking delay before rotating sorter after detection
    private final ElapsedTime sorterDelayTimer = new ElapsedTime();
    private boolean waitingForSorterMove = false;
    private int pendingSorterState = -1;
    private int pendingColor = -1;

    // -----------------------------
    // VOLTAGES YOU MEASURED
    // -----------------------------
    private static final double INTAKE_1_V  = 0.2225;
    private static final double INTAKE_2_V  = 0.756;
    private static final double INTAKE_3_V  = 1.2995;

    private static final double OUTTAKE_1_V = 0.985;
    private static final double OUTTAKE_2_V = 1.53;
    private static final double OUTTAKE_3_V = 0.445;

    // Simple tolerance + timeout
    private static final double SORTER_TOL_V = 0.05;
    private static final double SORTER_TIMEOUT_S = 0.45; //1

    public Sorter(HardwareMap hardwareMap) {
        sorterLeftServo = hardwareMap.get(Servo.class, "sorterLeftServo");
        sorterRightServo = hardwareMap.get(Servo.class, "sorterRightServo");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        sensorColorFront = hardwareMap.get(ColorSensor.class, "colourSensorFront");
        sensorDistanceFront = hardwareMap.get(DistanceSensor.class, "colourSensorFront");

        sensorColorBack = hardwareMap.get(ColorSensor.class, "colourSensorBack");
        sensorDistanceBack = hardwareMap.get(DistanceSensor.class, "colourSensorBack");

        sorterAnalog = hardwareMap.get(AnalogInput.class, "sorterAnalog");

        // Start at INTAKE slot 1 (index 0)
        //moveSorterToIntakeSlot(0);

        ballTimer.reset();
        sorterDelayTimer.reset();
    }

    // --------------------------------------------------
    // PUBLIC API
    // --------------------------------------------------

    public double getSorterVoltage() {
        return sorterAnalog.getVoltage();
    }

    /** Reset ball memory and sorter position to intake slot 1. */
    public void reset() {
        ball1 = ball2 = ball3 = -1;
        sorterState = 0;
        waitingForSorterMove = false;
        pendingSorterState = -1;
        pendingColor = -1;
        lastBallPresent = false;
        ballTimer.reset();
        sorterDelayTimer.reset();
        moveSorterToIntakeSlot(0);
    }

    // -----------------------------
    // SERVO POSITIONS (from TeleOp)
    // -----------------------------

    public void moveSorterToIntakeSlot(int slotIndex) {
        double basePos;
        switch (slotIndex) {
            default:
            case 0: basePos = Variables.sorter1Position;   break;   // dpad_down
            case 1: basePos = Variables.sorter2Position; break;   // dpad_left
            case 2: basePos = Variables.sorter3Position;  break;   // dpad_right
        }
        sorterLeftServo.setPosition(basePos);
        sorterRightServo.setPosition(basePos + Variables.sorterOffset);
    }

    public void moveSorterToOuttakeSlot(int slotIndex) {
        double basePos;
        switch (slotIndex) {
            default:
            case 0: basePos = Variables.sorter1OuttakePosition; break;   // share
            case 1: basePos = Variables.sorter2OuttakePosition; break;   // options
            case 2: basePos = Variables.sorter3OuttakePosition; break;   // guide
        }
        sorterLeftServo.setPosition(basePos);
        sorterRightServo.setPosition(basePos + Variables.sorterOffset);
    }

    // -----------------------------
    // ACTIONS: MOVE + WAIT BY VOLTAGE
    // -----------------------------

    public Action moveIntakeSlotAndWait(int slotIndex) {
        return new MoveSorterAndWaitAction(true, slotIndex);
    }

    public Action moveOuttakeSlotAndWait(int slotIndex) {
        return new MoveSorterAndWaitAction(false, slotIndex);
    }

    private class MoveSorterAndWaitAction implements Action {
        private final boolean intakeMode;
        private final int slot;
        private boolean commanded = false;
        private final ElapsedTime t = new ElapsedTime();

        MoveSorterAndWaitAction(boolean intakeMode, int slot) {
            this.intakeMode = intakeMode;
            this.slot = slot;
        }

        private double targetV() {
            if (intakeMode) {
                switch (slot) {
                    default:
                    case 0: return INTAKE_1_V;
                    case 1: return INTAKE_2_V;
                    case 2: return INTAKE_3_V;
                }
            } else {
                switch (slot) {
                    default:
                    case 0: return OUTTAKE_1_V;
                    case 1: return OUTTAKE_2_V;
                    case 2: return OUTTAKE_3_V;
                }
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!commanded) {
                if (intakeMode) moveSorterToIntakeSlot(slot);
                else moveSorterToOuttakeSlot(slot);
                t.reset();
                commanded = true;
            }

            double v = getSorterVoltage();
            double tv = targetV();

            packet.put("sorterV", v);
            packet.put("sorterTargetV", tv);
            packet.put("sorterMode", intakeMode ? "INTAKE" : "OUTTAKE");
            packet.put("sorterSlot", slot);

            if (Math.abs(v - tv) <= SORTER_TOL_V) return false;
            if (t.seconds() > SORTER_TIMEOUT_S) return false;

            return true;
        }
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

    public String getPatternString() {
        return "" + colourToChar(ball1) + colourToChar(ball2) + colourToChar(ball3);
    }

    // --------------------------------------------------
    // SIMPLE ACTIONS YOU ALREADY HAD
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

    public Action loadedBalls() { return new Sorter.loadedBalls(); }

    public Action intakeAndLoadThree() { return new IntakeAndLoadThreeAction(); }

    public class preset implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            reset();
            return false;
        }
    }

    public Action preset() { return new Sorter.preset(); }

    // --------------------------------------------------
    // MAIN AUTONOMOUS ACTION: intake and load 3 balls
    // --------------------------------------------------

    private class IntakeAndLoadThreeAction implements Action {
        private boolean initialized = false;
        private final ElapsedTime timeoutTimer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (!initialized) {
                reset();
                intakeMotor.setPower(1.0);
                timeoutTimer.reset();
                initialized = true;
            }

            if (timeoutTimer.seconds() > 3.0) {
                intakeMotor.setPower(0.0);
                packet.put("TIMEOUT", true);
                packet.put("pattern", getPatternString());
                return false;
            }

            if (sorterState >= 3) {
                intakeMotor.setPower(0.0);
                packet.put("pattern", getPatternString());
                return false;
            }

            double distF = sensorDistanceFront.getDistance(DistanceUnit.CM);
            double distB = sensorDistanceBack.getDistance(DistanceUnit.CM);

            boolean ballDetected =
                    (distF < BALL_DETECT_DISTANCE_CM) || (distB < BALL_DETECT_DISTANCE_CM);
            boolean allowNewBall = ballTimer.milliseconds() > BALL_COOLDOWN_MS;

            boolean newBall = ballDetected && !lastBallPresent && allowNewBall;

            if (newBall && !waitingForSorterMove && sorterState < 3) {
                pendingColor = detectColourFromTwoSensors();
                pendingSorterState = sorterState;
                waitingForSorterMove = true;
                sorterDelayTimer.reset();
            }

            if (waitingForSorterMove && sorterDelayTimer.milliseconds() > 50) {

                switch (pendingSorterState) {
                    case 0:
                        ball1 = pendingColor;
                        moveSorterToIntakeSlot(1);  // intake second
                        sorterState = 1;
                        break;

                    case 1:
                        ball2 = pendingColor;
                        moveSorterToIntakeSlot(2);  // intake third
                        sorterState = 2;
                        break;

                    case 2:
                        ball3 = pendingColor;
                        sorterState = 3;
                        intakeMotor.setPower(0.0);
                        break;
                }

                ballTimer.reset();
                waitingForSorterMove = false;
            }

            lastBallPresent = ballDetected;

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
            packet.put("sorterAnalogV", getSorterVoltage());

            return true;
        }
    }

    // --------------------------------------------------
    // COLOUR HELPERS
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
