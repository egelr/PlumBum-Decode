package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.TransferArm;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Autonomous(name = "AutoShootingRed", group = "Autonomous")
public class AutoShootingRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        TransferArm transferArm = new TransferArm(hardwareMap);



        TrajectoryActionBuilder firstShootingTrajectory = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(55,-25,Math.toRadians(-225)), Math.toRadians(0));
        Action firstShootingTrajectoryAction;

        firstShootingTrajectoryAction = firstShootingTrajectory.build();

        Actions.runBlocking(transferArm.launch());

        while (!isStopRequested() && !opModeIsActive()) {

            waitForStart();

            if (isStopRequested()) return;


            Actions.runBlocking(
                    new SequentialAction(

                            new ParallelAction(
                            firstShootingTrajectoryAction,
                            shooter.ShooterOn()
                    ),
                            transferArm.launch(),
                            new SleepAction(30)
                    ));

        }
    }
}
