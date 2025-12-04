package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Turret MoveToPosition SIMPLE")
public class TurretControlTest extends LinearOpMode {

    private Motor turretMotor;

    @Override
    public void runOpMode() {

        turretMotor = new Motor(hardwareMap, "turretPositionMotor");


        turretMotor.setPositionCoefficient(0.05);
        turretMotor.setPositionTolerance(50);
        turretMotor.resetEncoder();
        turretMotor.setTargetPosition(0);
        turretMotor.setRunMode(Motor.RunMode.PositionControl);

        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        ElapsedTime timer = new ElapsedTime();


        waitForStart();

        while (opModeIsActive()) {

            // Move to Position 1
            if (gamepad1.a) {
                turretMotor.setTargetPosition(0);
                turretMotor.setRunMode(Motor.RunMode.PositionControl);
                timer.reset();
                while((!turretMotor.atTargetPosition()) && timer.seconds()<1) {
                    turretMotor.set(0.4);
                }
            }

            // Move to Position 2
            if (gamepad1.b) {
                turretMotor.setTargetPosition(232);
                turretMotor.setRunMode(Motor.RunMode.PositionControl);
                timer.reset();
                while((!turretMotor.atTargetPosition()) && timer.seconds()<1) {
                    turretMotor.set(0.4);
                }
            }

            // Move to Position 3
            if (gamepad1.y) {
                turretMotor.setTargetPosition(-232);
                turretMotor.setRunMode(Motor.RunMode.PositionControl);
                timer.reset();
                while((!turretMotor.atTargetPosition()) && timer.seconds()<1) {
                    turretMotor.set(0.4);
                }
            }

            turretMotor.set(0);
            telemetry.addData("Status: ", turretMotor.getCurrentPosition());
            telemetry.addData("StatusIntake", turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
