package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Variables;
public class Transfer {
    private Servo transferOutputServo;
    private ElapsedTime timer = new ElapsedTime();
    private boolean initialized = false;
    public Transfer(HardwareMap hardwareMap) {
        transferOutputServo = hardwareMap.get(Servo.class, "transferOutputServo");
    }

    public class launch implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            transferOutputServo.setPosition(Variables.transferUpPosition);
            return false;
        }
    }

    public Action launch() {
        return new Transfer.launch();
    }
    public class preset implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            transferOutputServo.setPosition(Variables.transferDownPosition);
            return false;
        }

    }

    public Action preset() {
        return new Transfer.preset();
    }
}
