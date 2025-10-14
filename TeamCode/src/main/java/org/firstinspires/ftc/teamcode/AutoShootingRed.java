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
                .strafeToLinearHeading(new Vector2d(55, -25),Math.toRadians(-225.0));
        TrajectoryActionBuilder firstIntakeTrajectory = firstShootingTrajectory.endTrajectory().fresh()
               .setReversed(false)
                .strafeToLinearHeading(new Vector2d(45.0, -8.0),Math.toRadians(-90.0));
        TrajectoryActionBuilder firstIntakeTrajectory2 = firstIntakeTrajectory.endTrajectory().fresh()
                .setReversed(true)
                .strafeTo(new Vector2d(45, -3))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(45, 2))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(45, 9));
        TrajectoryActionBuilder secondShootingTrajectory = firstIntakeTrajectory2.endTrajectory().fresh()
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(55.0, -25.0),Math.toRadians(-225.0));
        TrajectoryActionBuilder secondIntakeTrajectory1 = secondShootingTrajectory.endTrajectory().fresh()
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(78.5, -8.0),Math.toRadians(-90.0));


        Action firstShootingTrajectoryAction;
        Action firstIntakeTrajectoryAction;
        Action firstIntakeTrajectory2Action;
        Action secondShootingTrajectoryAction;
        Action secondIntakeTrajectory1Action;


        firstShootingTrajectoryAction = firstShootingTrajectory.build();
        firstIntakeTrajectoryAction = firstIntakeTrajectory.build();
        firstIntakeTrajectory2Action = firstIntakeTrajectory2.build();
        secondShootingTrajectoryAction = secondShootingTrajectory.build();
        secondIntakeTrajectory1Action = secondIntakeTrajectory1.build();

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
                            firstIntakeTrajectoryAction,
                            
                            new ParallelAction(
                                    intake.IntakeOn(),
                                    firstIntakeTrajectory2Action
                                    ),
                            secondShootingTrajectoryAction,
                            intake.IntakeOff(),

                            new ParallelAction(
                                    shooter.ShooterOn(),
                                    new SequentialAction(
                                            firstShootingTrajectoryAction,
                                            new SleepAction(0.5),

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
                                    secondIntakeTrajectory1Action,
                                    intake.IntakeOn()
                            )




                    )
            );

        }
    }
}
