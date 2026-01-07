package org.firstinspires.ftc.teamcode.subsystemTesting;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Variables;

@TeleOp(name = "Turret test")
public class TurretControlTest extends LinearOpMode {

    private Servo turretServo;


    @Override
    public void runOpMode() {
        turretServo = hardwareMap.get(Servo.class, "turretServo");

        // ---------- TURRET SETUP ----------

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.square) {
                turretServo.setPosition(0.5);
            }
            if (gamepad1.circle) {
                turretServo.setPosition(0.2);
            }
            if (gamepad1.triangle) {
                turretServo.setPosition(0);
            }




        }
    }
}
