package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Variables;
public class TransferArm {
    private Servo transferBoxServo;
    private ElapsedTime timer = new ElapsedTime();
    private boolean initialized = false;
    public TransferArm(HardwareMap hardwareMap) {
        transferBoxServo = hardwareMap.get(Servo.class, "transferBoxServo");
    }

    public class launch implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            transferBoxServo.setPosition(0.71);
            return false;
        }
    }

    public Action launch() {
        return new TransferArm.launch();
    }
    public class preset implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            transferBoxServo.setPosition(0.87);
            return false;
        }

    }

    public Action preset() {
        return new TransferArm.preset();
    }
}
