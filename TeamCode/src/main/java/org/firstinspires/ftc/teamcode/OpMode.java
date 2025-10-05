package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name = "TeleOpMode")
public class OpMode extends LinearOpMode {

    int liftLastPosition;
    //Creating the variables for Motors
    private Motor fL, fR, bL, bR;
    private Motor intakeMotor, shooterMotorRight, shooterMotorLeft;
    //Creating drive speed variable
    public double drive_speed = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        //Creating Drivetrain Motors and Setting their behaviour to "brake"
        fL = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //Creating the Mecanum Drivetrain
        MecanumDrive drive = new MecanumDrive(fL, fR, bL, bR);

        intakeMotor = new Motor(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_1150);
        shooterMotorRight = new Motor(hardwareMap, "shooterMotorRight", Motor.GoBILDA.RPM_6000); //cpr??
        shooterMotorLeft = new Motor(hardwareMap, "shooterMotorLeft", Motor.GoBILDA.RPM_6000);

        double velocityR = shooterMotorRight.getVelocity(); // only for MotorEx
        double correctedR = shooterMotorRight.getCorrectedVelocity();
        Motor.Encoder encoderR = shooterMotorRight.encoder;
        double revolutionsR = shooterMotorRight.encoder.getRevolutions();

        double velocityL = shooterMotorLeft.getVelocity(); // only for MotorEx
        double correctedL = shooterMotorLeft.getCorrectedVelocity();
        Motor.Encoder encoderL = shooterMotorLeft.encoder;
        double revolutionsL = shooterMotorLeft.encoder.getRevolutions();

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        while (!isStopRequested()) {


            //Drivetrain controls
            drive.driveRobotCentric(
                    -gamepad1.left_stick_x * drive_speed,
                    gamepad1.left_stick_y * drive_speed,
                    -gamepad1.right_stick_x * drive_speed,
                    false

            );
            //Drivetrain Motors speed Change controls
            if (gamepad1.right_trigger > 0.5) {
                drive_speed = 0.45;
            } else {
                drive_speed = 1;
            }
            if (gamepad1.triangle){
                intakeMotor.set(1);
            }
            if(gamepad1.circle){
                intakeMotor.set(0);
            }

        }

    }
}
