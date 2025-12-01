package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.Variables;

@TeleOp(name = "ShooterTesting")
public class ShooterTesting extends LinearOpMode {
    private DcMotorEx shooterLeft, shooterRight;

    private double x;

    // Motor specs
    private static final int CPR = 28;             // encoder counts per rev (GoBILDA 6000RPM)
    private static final int MAX_RPM = 6000;
    private static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;  // ~2800

    // Start at 0% power
    private double targetVelocity = 0;

    @Override
    public void runOpMode() throws InterruptedException {


        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        Servo shooterAngleServo = hardwareMap.get(Servo.class, "shooterAngleServo");


        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // PIDF coefficients tuned for 6000 RPM
        shooterLeft.setVelocityPIDFCoefficients(50, 0.0, 0.001, 11.7);


        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        while (!isStopRequested()) {



            if(gamepad1.square){
                targetVelocity = MAX_TICKS_PER_SEC * 0.31;
            }
            if (gamepad1.dpad_up) {
                targetVelocity += 0.5;  // increase by 50 ticks/sec
            }
            if (gamepad1.dpad_down) {
                targetVelocity -= 0.5;  // decrease by 50 ticks/sec
            }

            if (gamepad1.guide) {
                targetVelocity = 0;
                x = 0;
                shooterAngleServo.setPosition(x);
            }

            if(gamepad1.share) {
                x = 0.5;
                shooterAngleServo.setPosition(x);
            }
            if(gamepad1.triangle && x < 0.45){
                x = x + 0.05;
                shooterAngleServo.setPosition(x);
                sleep( 300);
            }
            if(gamepad1.cross && x > 0.05){
                x = x - 0.05;
                shooterAngleServo.setPosition(x);
                sleep( 300);
            }


            // Clamp target
            targetVelocity = Math.max(0, Math.min(MAX_TICKS_PER_SEC, targetVelocity));

            // Apply velocity
            shooterLeft.setVelocity(targetVelocity);
            shooterRight.setVelocity(targetVelocity);

            // Telemetry for driver feedback
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Target RPM", (targetVelocity / CPR) * 60);
            telemetry.addData("Left Vel", shooterLeft.getVelocity());
            telemetry.addData("Right Vel", shooterRight.getVelocity());
            telemetry.addData("servo angle position: ", x);
            telemetry.update();
        }

    }
}