package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Variables;

public class Shooter {
    private DcMotorEx shooterLeft, shooterRight;
    private Servo shooterAngleServo;
    private ElapsedTime timer = new ElapsedTime();
    public Shooter(HardwareMap hardwareMap) {
        shooterAngleServo = hardwareMap.get(Servo.class, "shooterAngleServo");

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setVelocityPIDFCoefficients(50, 0, 0.001, 11.7);

        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public class ShooterOn implements Action {
        private boolean initialized = false;
        private double targetVelocity;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                shooterAngleServo.setPosition(Variables.shooterAngleMid);
                targetVelocity = 2800 * Variables.shooterSpeedMid;
                double power = targetVelocity / 2800;  // simple feedforward
                power = Math.max(0, Math.min(1, power));
                shooterLeft.setVelocity(targetVelocity);
                shooterRight.setPower(power);
                timer.reset();
                initialized = true;
            }
            double Lv = Math.abs(shooterLeft.getVelocity()-targetVelocity);

            if (Lv<50  || timer.seconds() > 1.5) {
                return false;
            } else {
                return true;
            }
        }
    }
    public  Action ShooterOn() {
        return new Shooter.ShooterOn();
    }
    public class ShooterOff implements Action {
        private boolean initialized = false;
        private double targetVelocity;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                shooterAngleServo.setPosition(0);
                shooterLeft.setVelocity(0);
                shooterRight.setPower(0);
                //timer.reset();
                initialized = true;
            }
                return false;
        }
    }
    public  Action ShooterOff() {
        return new Shooter.ShooterOff();
    }

    public class ShooterOnFar implements Action {
        private boolean initialized = false;
        private double targetVelocity;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                shooterAngleServo.setPosition(Variables.shooterAngleFar);
                targetVelocity = 2800 * Variables.shooterSpeedFar;
                double power = targetVelocity / 2800;  // simple feedforward
                power = Math.max(0, Math.min(1, power));
                shooterLeft.setVelocity(targetVelocity);
                shooterRight.setPower(power);
                timer.reset();
                initialized = true;
            }


            double Lv = Math.abs(shooterLeft.getVelocity()-targetVelocity);

            if (Lv<50 || timer.seconds() > 1.5) {
                return false;
            } else {
                return true;
            }
        }
    }
    public  Action ShooterOnFar() {
        return new Shooter.ShooterOnFar();
    }
}