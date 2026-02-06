package org.firstinspires.ftc.teamcode.subsystemTesting;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "aTsiplem", group = "Vision")
public class turretAngleTest extends LinearOpMode {

    private Servo turretServo1;
    private Servo turretServo2;
    @Override
    public void runOpMode() {
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");

        waitForStart();

        while (opModeIsActive()) {

            // manual test (optional)
            if (gamepad1.square){ turretServo1.setPosition(0.5);
            turretServo2.setPosition(0.5);}
            if (gamepad1.circle){ turretServo1.setPosition(0.2);
            turretServo2.setPosition(0.2);}
            if (gamepad1.triangle) {turretServo1.setPosition(0.0);
            turretServo2.setPosition(0.0);}
        }
    }
}
