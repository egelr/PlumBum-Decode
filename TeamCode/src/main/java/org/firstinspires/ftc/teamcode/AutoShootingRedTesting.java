package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
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
@Autonomous(name = "AutoShootingRedTesting", group = "Autonomous")
public class AutoShootingRedTesting extends LinearOpMode {

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
        TrajectoryActionBuilder firstIntakeTrajectory1 = firstShootingTrajectory.endTrajectory().fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(45.0, -8.0),Math.toRadians(-90.0)); // +90??
        TrajectoryActionBuilder firstIntakeTrajectory2 = firstIntakeTrajectory1.endTrajectory().fresh()
                .setReversed(true)
                .strafeTo(new Vector2d(45, -3))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(45, 2))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(45, 9));
        TrajectoryActionBuilder secondShootingTrajectory = firstIntakeTrajectory2.endTrajectory().fresh()
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(55.0, -25.0),Math.toRadians(-225.0));
        TrajectoryActionBuilder secondIntakeTrajectory1 = secondShootingTrajectory.endTrajectory().fresh()
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(78.5, -8),Math.toRadians(-90)); // +90??
        TrajectoryActionBuilder secondIntakeTrajectory2 = secondIntakeTrajectory1.endTrajectory().fresh()
                .setReversed(true)
                .strafeTo(new Vector2d(78.5, -3))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(78.5, 2))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(78.5, 9));
        TrajectoryActionBuilder thirdShootingTrajectory = secondIntakeTrajectory2.endTrajectory().fresh()
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(55.0, -25.0),Math.toRadians(-225.0));
        TrajectoryActionBuilder parkingTrajectory = thirdShootingTrajectory.endTrajectory().fresh()
                .setReversed(false)
                .strafeTo(new Vector2d(55, -20));


        Action firstShootingTrajectoryAction;
        Action firstIntakeTrajectory1Action;
        Action firstIntakeTrajectory2Action;
        Action secondShootingTrajectoryAction;
        Action secondIntakeTrajectory1Action;
        Action secondIntakeTrajectory2Action;
        Action thirdShootingTrajectoryAction;
        Action parkingTrajectoryAction;


        firstShootingTrajectoryAction = firstShootingTrajectory.build();
        firstIntakeTrajectory1Action = firstIntakeTrajectory1.build();
        firstIntakeTrajectory2Action = firstIntakeTrajectory2.build();
        secondShootingTrajectoryAction = secondShootingTrajectory.build();
        secondIntakeTrajectory1Action = secondIntakeTrajectory1.build();
        secondIntakeTrajectory2Action = secondIntakeTrajectory2.build();
        thirdShootingTrajectoryAction = thirdShootingTrajectory.build();
        parkingTrajectoryAction = parkingTrajectory.build();


        Actions.runBlocking(transferArm.preset());
        Actions.runBlocking(intake.preset());

        while (!isStopRequested() && !opModeIsActive()) {

            waitForStart();

            if (isStopRequested()) return;


            Actions.runBlocking(
                    new SequentialAction(

                            new ParallelAction(
                                    shooter.ShooterOn(),
                                    new SequentialAction(
                                            firstShootingTrajectoryAction,

                                            transferArm.launch(),
                                            new SleepAction(Variables.sleepAfterLaunch),
                                            transferArm.preset(),
                                            new SleepAction(Variables.sleepAfterPreset),

                                            transferArm.launch(),
                                            new SleepAction(Variables.sleepAfterLaunch),
                                            transferArm.preset(),
                                            new SleepAction(Variables.sleepAfterPreset),

                                            transferArm.launch(),
                                            new SleepAction(Variables.sleepAfterLaunch),
                                            transferArm.preset(),
                                            new SleepAction(Variables.sleepAfterPreset)
                                    )
                            ),
                            shooter.ShooterOff(),

                            new ParallelAction(
                                    firstIntakeTrajectory1Action,
                                    intake.IntakeOn()
                            ),

                            firstIntakeTrajectory2Action,

                            new ParallelAction(
                                    secondShootingTrajectoryAction,
                                    intake.IntakeOff(),
                                    shooter.ShooterOn()
                            ),

                            new SleepAction(0.5), //galima keist

                            transferArm.launch(),
                            new SleepAction(Variables.sleepAfterLaunch),
                            transferArm.preset(),
                            new SleepAction(Variables.sleepAfterPreset),

                            transferArm.launch(),
                            new SleepAction(Variables.sleepAfterLaunch),
                            transferArm.preset(),
                            new SleepAction(Variables.sleepAfterPreset),

                            transferArm.launch(),
                            new SleepAction(Variables.sleepAfterLaunch),
                            transferArm.preset(),
                            new SleepAction(Variables.sleepAfterPreset),

                            shooter.ShooterOff(),

                            new ParallelAction(
                                    secondIntakeTrajectory1Action,
                                    intake.IntakeOn()
                            ),
                            secondIntakeTrajectory2Action,
                            
                            new ParallelAction(
                                    shooter.ShooterOn(),
                                    thirdShootingTrajectoryAction,
                                    intake.IntakeOff()
                            ),

                            new SleepAction(0.5), //galima keist

                            transferArm.launch(),
                            new SleepAction(Variables.sleepAfterLaunch),
                            transferArm.preset(),
                            new SleepAction(Variables.sleepAfterPreset),

                            transferArm.launch(),
                            new SleepAction(Variables.sleepAfterLaunch),
                            transferArm.preset(),
                            new SleepAction(Variables.sleepAfterPreset),

                            transferArm.launch(),
                            new SleepAction(Variables.sleepAfterLaunch),
                            transferArm.preset(),
                            new SleepAction(Variables.sleepAfterPreset),

                            new ParallelAction(
                                    intake.IntakeOff(),
                                    parkingTrajectoryAction
                                    )
                    )
            );

        }
    }
}