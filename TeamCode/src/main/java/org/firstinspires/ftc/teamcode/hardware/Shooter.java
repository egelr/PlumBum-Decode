package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Variables;

public class Shooter {
    private DcMotorEx shooterLeft, shooterRight;
    private ElapsedTime timer = new ElapsedTime();
    public Shooter(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        shooterLeft.setVelocityPIDFCoefficients(50, 0.0, 0.001, 11.7);
        shooterRight.setVelocityPIDFCoefficients(50, 0.0, 0.001, 11.7);
    }
    public class ShooterOn implements Action {
        private boolean initialized = false;
        private double targetVelocity;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                targetVelocity = 2800 * 0.329;
                //targetVelocity = Math.max(0, Math.min(MAX_TICKS_PER_SEC, targetVelocity));
                shooterLeft.setVelocity(targetVelocity);
                shooterRight.setVelocity(targetVelocity);
                timer.reset();
                initialized = true;
            }
            double Lv = Math.abs(shooterLeft.getVelocity()-targetVelocity);
            double Rv = Math.abs(shooterRight.getVelocity()-targetVelocity);

            if (Lv<50 && Rv < 50 || timer.seconds() > 1.5) {
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
                shooterLeft.setVelocity(0);
                shooterRight.setVelocity(0);
                //timer.reset();
                initialized = true;
            }
                return false;
        }
    }
    public  Action ShooterOff() {
        return new Shooter.ShooterOff();
    }
}