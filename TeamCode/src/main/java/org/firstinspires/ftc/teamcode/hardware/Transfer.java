package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Transfer {

    private final Servo kickerTopServo;
    private final Servo kickerBottomServo;

    private final AnalogInput kickerAnalog;

    // positions you gave
    private static final double KICKER_UP_POS = 0.88;
    private static final double KICKER_DOWN_POS = 0.99;

    // measured voltages
    private static final double KICKER_DOWN_V = 3.0025;
    private static final double KICKER_UP_V   = 2.7495;

    // simple tolerance + timeout
    private static final double TOL_V = 0.04;       // +/- 0.04V
    private static final double TIMEOUT_S = 1.0;    // 1 second max wait

    public Transfer(HardwareMap hardwareMap) {
        kickerTopServo = hardwareMap.get(Servo.class, "kickerTopServo");
        kickerBottomServo = hardwareMap.get(Servo.class, "kickerBottomServo");
        kickerAnalog = hardwareMap.get(AnalogInput.class, "kickerAnalog");
    }

    private double getKickerVoltage() {
        return kickerAnalog.getVoltage();
    }

    public Action launch() { // UP
        return new Action() {
            private boolean commanded = false;
            private final ElapsedTime t = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!commanded) {
                    kickerTopServo.setPosition(KICKER_UP_POS);
                    kickerBottomServo.setPosition(KICKER_UP_POS);
                    t.reset();
                    commanded = true;
                }

                double v = getKickerVoltage();
                packet.put("kickerV", v);

                if (Math.abs(v - KICKER_UP_V) <= TOL_V) return false;
                if (t.seconds() > TIMEOUT_S) return false;

                return true;
            }
        };
    }

    public Action preset() { // DOWN
        return new Action() {
            private boolean commanded = false;
            private final ElapsedTime t = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!commanded) {
                    kickerTopServo.setPosition(KICKER_DOWN_POS);
                    kickerBottomServo.setPosition(KICKER_DOWN_POS);
                    t.reset();
                    commanded = true;
                }

                double v = getKickerVoltage();
                packet.put("kickerV", v);

                if (Math.abs(v - KICKER_DOWN_V) <= TOL_V) return false;
                if (t.seconds() > TIMEOUT_S) return false;

                return true;
            }
        };
    }
}
