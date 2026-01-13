package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Variables;

public class Shooter {

    private final DcMotorEx shooterLeft, shooterRight;
    private final Servo shooterAngleServo;

    // Timers
    private final ElapsedTime actionTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    // PIDF state
    private double integral = 0.0;
    private double lastError = 0.0;

    // --- Motor constants (goBILDA 6000RPM) ---
    private static final double MAX_RPM = 6000.0;

    // goBILDA / REV through FTC SDK "getVelocity()" is typically in ticks/sec
    // For goBILDA Yellow Jacket encoder: 28 "counts" per motor rev (common FTC convention).
    private static final double TICKS_PER_REV = 28.0;

    // Finish conditions (you can move to Variables if you want)
    private static final double READY_RPM_TOL = 75;     // tolerance in RPM
    private static final double READY_TIMEOUT_S = 1.0;  // seconds

    // Anti-windup clamp (power-ish scale after multiplying by kI)
    private static final double INTEGRAL_CLAMP = 3000.0; // clamp raw integral (RPM*s). Tune if needed.

    public Shooter(HardwareMap hardwareMap) {
        shooterAngleServo = hardwareMap.get(Servo.class, "shooterAngleServo");

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterLeft.setDirection(DcMotor.Direction.REVERSE);

        // Custom control => RUN_WITHOUT_ENCODER (we only read encoder for measurement)
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        loopTimer.reset();
    }

    // -------------------- Helpers --------------------

    /** Left motor RPM from encoder velocity. */
    private double getLeftRpm() {
        // ticks/sec -> rev/sec -> RPM
        double ticksPerSecond = shooterLeft.getVelocity();
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    /**
     * If your right motor encoder is reliable too, you can average them.
     * For simplicity + robustness, we regulate based on left RPM.
     */
    private double getShooterRpm() {
        return getLeftRpm();
    }

    private void resetPid() {
        integral = 0.0;
        lastError = 0.0;
        loopTimer.reset();
    }

    private void setShooterPower(double pwr) {
        pwr = Range.clip(pwr, 0.0, 1.0);
        shooterLeft.setPower(pwr);
        shooterRight.setPower(pwr);
    }

    private boolean atSpeed(double targetRpm) {
        return Math.abs(getShooterRpm() - targetRpm) <= READY_RPM_TOL;
    }

    /**
     * Custom PIDF that outputs motor power [0..1].
     * Uses Variables.shooter_kP/kI/kD/kF
     *
     * IMPORTANT: Make sure you define these in Variables:
     *  public static double shooter_kP = ...;
     *  public static double shooter_kI = ...;
     *  public static double shooter_kD = ...;
     *  public static double shooter_kF = ...;
     */
    private double updatePidf(double targetRpm) {
        double currentRpm = getShooterRpm();
        double error = targetRpm - currentRpm;

        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt < 1e-4) dt = 1e-4;

        // Integral (RPM*s), clamp to prevent windup
        integral += error * dt;
        integral = Range.clip(integral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

        // Derivative (RPM/s)
        double derivative = (error - lastError) / dt;
        lastError = error;

        // Feedforward: map target RPM into base power
        // If MAX_RPM is accurate for your build, kF ~ 1 is a good start.
        double ff = (targetRpm / MAX_RPM);

        double out =
                (Variables.shooter_kP * error) +
                        (Variables.shooter_kI * integral) +
                        (Variables.shooter_kD * derivative) +
                        (Variables.shooter_kF * ff);

        return Range.clip(out, 0.0, 1.0);
    }

    private double speedToTargetRpm(double speed01) {
        speed01 = Range.clip(speed01, 0.0, 1.0);
        return MAX_RPM * speed01;
    }

    // -------------------- Actions --------------------

    private class ShooterSpinToSpeed implements Action {
        private final double speed01;
        private final double anglePos;

        private boolean initialized = false;
        private double targetRpm = 0.0;

        ShooterSpinToSpeed(double speed01, double anglePos) {
            this.speed01 = speed01;
            this.anglePos = anglePos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                shooterAngleServo.setPosition(anglePos);

                targetRpm = speedToTargetRpm(speed01);

                resetPid();
                actionTimer.reset();
                initialized = true;
            }

            double power = updatePidf(targetRpm);
            setShooterPower(power);

            // Telemetry for tuning
            packet.put("Shooter TargetRPM", targetRpm);
            packet.put("Shooter RPM", getShooterRpm());
            packet.put("Shooter ErrorRPM", targetRpm - getShooterRpm());
            packet.put("Shooter Power", power);

            // End when ready or timed out
            if (atSpeed(targetRpm) || actionTimer.seconds() > READY_TIMEOUT_S) {
                return false;
            }
            return true;
        }
    }

    /** Near shot (uses Variables.shooterSpeedNear, Variables.shooterAngleMid unless you have a Near angle). */

    /** Mid shot (your old ShooterOn). */
    public Action ShooterOn() {
        return new ShooterSpinToSpeed(Variables.shooterSpeedAuto, Variables.shooterAngleAuto);
    }

    /** Far shot. */
    public Action ShooterOnFar() {
        return new ShooterSpinToSpeed(Variables.shooterSpeedFar, Variables.shooterAngleFar);
    }

    public class ShooterOff implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                shooterAngleServo.setPosition(0);
                setShooterPower(0.0);
                resetPid();
                initialized = true;
            }
            return false;
        }
    }

    public Action ShooterOff() {
        return new ShooterOff();
    }
}
