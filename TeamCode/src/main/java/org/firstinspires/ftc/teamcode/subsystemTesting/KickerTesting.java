package org.firstinspires.ftc.teamcode.subsystemTesting;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Variables;

@TeleOp(name = "KickerTesting")
public class KickerTesting extends LinearOpMode {

    private Servo kickerTopServo, kickerBottomServo;


    @Override
    public void runOpMode() {
        kickerTopServo = hardwareMap.get(Servo.class, "kickerTopServo");
        kickerBottomServo = hardwareMap.get(Servo.class, "kickerBottomServo");


        // ---------- TURRET SETUP ----------

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.triangle) {
                kickerTopServo.setPosition(0.85);
                kickerBottomServo.setPosition(0.85);
            }
            if (gamepad1.circle) {
                kickerTopServo.setPosition(0.98);
                kickerBottomServo.setPosition(0.98);
            }
            if (gamepad1.square) {
                kickerTopServo.setPosition(1);
                kickerBottomServo.setPosition(1);
            }




        }
    }
}
