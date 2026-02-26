package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Variables;
public class Turret {
    private Servo turretServo1, turretServo2;
    private ElapsedTime timer = new ElapsedTime();
    private boolean initialized = false;
    public Turret(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");

    }

    public class left implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo1.setPosition(0.88); //1
            turretServo2.setPosition(0.88);
            return false;
        }
    }

    public Action left() {
        return new Turret.left();
    }
    public class farLeft implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo1.setPosition(0.4175); // 0.335
            turretServo2.setPosition(0.4175); // 0.335
            return false;
        }
    }

    public Action farLeft() {
        return new Turret.farLeft();
    }
    public class farRight implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo1.setPosition(0.5825); //0.665
            turretServo2.setPosition(0.5825); //0.665
            return false;
        }
    }

    public Action farRight() {
        return new Turret.farRight();
    }
    public class halfRight1 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo1.setPosition(0.695); //0.86
            turretServo2.setPosition(0.695); //0.86
            return false;
        }
    }

    public Action halfRight1() {
        return new Turret.halfRight1();
    }
    public class halfRight3 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo1.setPosition(0.685); //0.87
            turretServo2.setPosition(0.685); //0.87
            return false;
        }
    }

    public Action halfRight3() {
        return new Turret.halfRight3();
    }
    public class halfLeft1 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo1.setPosition(0.31); //0.13
            turretServo2.setPosition(0.31); //0.13

            return false;
        }
    }

    public Action halfLeft1() {
        return new Turret.halfLeft1();
    }
    public class halfLeft3 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo2.setPosition(0.31); //0.14
            turretServo2.setPosition(0.31); //0.14
            return false;
        }
    }

    public Action halfLeft3() {
        return new Turret.halfLeft3();
    }
    public class halfRight2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo1.setPosition(0.675); //0.83
            turretServo2.setPosition(0.675); //0.83
            return false;
        }
    }

    public Action halfRight2() {
        return new Turret.halfRight2();
    }
    public class halfLeft2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo1.setPosition(0.33); //0.16
            turretServo2.setPosition(0.33); //0.16
            return false;
        }
    }

    public Action halfLeft2() {
        return new Turret.halfLeft2();
    }
    public class right implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo1.setPosition(0.24); //0.12
            turretServo2.setPosition(0.24); //0
            return false;
        }
    }

    public Action right() {
        return new Turret.right();
    }
    public class farRightNew implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo1.setPosition(0.255); //0.12
            turretServo2.setPosition(0.255); //0
            return false;
        }
    }

    public Action farRightNew() {
        return new Turret.farRightNew();
    }
    public class preset implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo1.setPosition(0.5);
            turretServo2.setPosition(0.5);
            return false;
        }

    }

    public Action preset() {
        return new Turret.preset();
    }
    public class farAutoRedPreset implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turretServo1.setPosition(0.1);
            turretServo2.setPosition(0.1);
            return false;
        }

    }

    public Action farAutoRedPreset() {
        return new Turret.farAutoRedPreset();
    }
}