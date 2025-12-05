package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret{

    private final DcMotorEx turretMotor;

    // 45° offset equals 232 ticks → convert degrees to ticks
    public static final double TICKS_PER_DEGREE = 232.0 / 90.0;

    // Motor movement tuning
    private static final int POSITION_TOLERANCE = 5;
    private static final double MOVE_POWER = 0.4;

    public Turret(DcMotorEx turretMotor) {
        this.turretMotor = turretMotor;

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPositionTolerance(POSITION_TOLERANCE);
    }

    // ----------- INTERNAL MOVE METHOD (do not call directly in Auto) -----------
    private Action moveToAngleInternal(double degrees) {
        int targetTicks = (int) Math.round(degrees * TICKS_PER_DEGREE);

        return new Action() {

            boolean init = false;

            @Override
            public boolean run(TelemetryPacket packet) {

                if (!init) {
                    turretMotor.setTargetPosition(targetTicks);
                    turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turretMotor.setPower(MOVE_POWER);
                    init = true;
                }

                if (packet != null) {
                    packet.put("Turret Target", targetTicks);
                    packet.put("Turret Current", turretMotor.getCurrentPosition());
                    packet.put("Busy", turretMotor.isBusy());
                }

                if (!turretMotor.isBusy()) {
                    turretMotor.setPower(0);
                    return false; // finished
                }

                return true; // keep going
            }
        };
    }

    // ----------- PUBLIC ACTIONS YOU WILL CALL IN AUTO -----------

    public Action turretAngle0() {
        return moveToAngleInternal(0);
    }

    public Action turretAngle45() {
        return moveToAngleInternal(45);
    }

    public Action turretAngleMinus45() {
        return moveToAngleInternal(-45);
    }

}
