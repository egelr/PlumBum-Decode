package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Variables;
public class Turret {
    private Servo turretServo;
    private ElapsedTime timer = new ElapsedTime();
    private boolean initialized = false;
    public Turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
    }

    public class right implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo.setPosition(1);
            return false;
        }
    }

    public Action right() {
        return new Turret.right();
    }
    public class halfLeft implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo.setPosition(0.9);
            return false;
        }
    }

    public Action halfLeft() {
        return new Turret.halfLeft();
    }
    public class left implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo.setPosition(0);
            return false;
        }
    }

    public Action left() {
        return new Turret.right();
    }
    public class preset implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo.setPosition(0.5);
            return false;
        }

    }

    public Action preset() {
        return new Turret.preset();
    }
}