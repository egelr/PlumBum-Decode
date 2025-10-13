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

public class Intake {
    private DcMotorEx intakeMotor;
    private Servo transferBlockServo;

    private ElapsedTime timer = new ElapsedTime();
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        transferBlockServo = hardwareMap.get(Servo.class, "transferBlockServo");
    }

    public class IntakeOff implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intakeMotor.setPower(0);
                transferBlockServo.setPosition(0.38);
                //timer.reset();
                initialized = true;
            }
            return false;
        }
    }
    public  Action IntakeOff() {
        return new Intake.IntakeOff();
    }
    public class IntakeOn implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intakeMotor.setPower(1);
                transferBlockServo.setPosition(0.6);
                //timer.reset();
                initialized = true;
            }
            return false;
        }
    }
    public  Action IntakeOn() {
        return new Intake.IntakeOn();
    }
    public class preset implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            transferBlockServo.setPosition(0.38);
            return false;
        }

    }

    public Action preset() {
        return new Intake.preset();
    }
}