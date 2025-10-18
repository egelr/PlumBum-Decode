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
@Autonomous(name = "Blue", group = "Autonomous")
public class AutoShootingBlue extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        TransferArm transferArm = new TransferArm(hardwareMap);

        // y: -25 -> 25, heading: -225 -> 225
        TrajectoryActionBuilder firstShootingTrajectory = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(54, 25, Math.toRadians(222)), Math.toRadians(0));

        TrajectoryActionBuilder firstIntakeTrajectory = firstShootingTrajectory.endTrajectory().fresh()
                .setReversed(false)
                .turn(Math.toRadians(-132)); // 135 -> -135

        TrajectoryActionBuilder firstIntakeTrajectory2 = firstIntakeTrajectory.endTrajectory().fresh()
                .setReversed(true)
                .strafeTo(new Vector2d(45.5, 12)); // -8 -> 8

        TrajectoryActionBuilder firstIntakeTrajectory3 = firstIntakeTrajectory2.endTrajectory().fresh()
                .setReversed(true)
                .strafeTo(new Vector2d(45.5, 7))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(45.5, 3))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(45.5, -3));

        TrajectoryActionBuilder secondShootingTrajectory = firstIntakeTrajectory3.endTrajectory().fresh()
                .setReversed(false)
                .strafeTo(new Vector2d(54, 25))      // -25 -> 25
                .turnTo(Math.toRadians(222));        // -225 -> 225

        TrajectoryActionBuilder secondIntakeTrajectory1 = secondShootingTrajectory.endTrajectory().fresh()
                .setReversed(false)
                .turn(Math.toRadians(-132)); // 135 -> -135

        TrajectoryActionBuilder secondIntakeTrajectory2 = secondIntakeTrajectory1.endTrajectory().fresh()
                .setReversed(true)
                .strafeTo(new Vector2d(79, 12)); // -8 -> 8

        TrajectoryActionBuilder secondIntakeTrajectory3 = secondIntakeTrajectory2.endTrajectory().fresh()
                .setReversed(true)
                .strafeTo(new Vector2d(79, 8))    // -3 -> 3
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(79, 3))   // 2 -> -2
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(79, -5));  // 9 -> -9

        TrajectoryActionBuilder parkingTrajectory = secondIntakeTrajectory3.endTrajectory().fresh()
                .setReversed(false)
                .turnTo(Math.toRadians(225))
                .strafeTo(new Vector2d(55, 20));   // -20 -> 20



        Action firstShootingTrajectoryAction;
        Action firstIntakeTrajectoryAction;
        Action firstIntakeTrajectory2Action;
        Action firstIntakeTrajectory3Action;
        Action secondShootingTrajectoryAction;
        Action secondIntakeTrajectory1Action;
        Action secondIntakeTrajectory2Action;
        Action secondIntakeTrajectory3Action;
        Action parkingTrajectoryAction;


        firstShootingTrajectoryAction = firstShootingTrajectory.build();
        firstIntakeTrajectoryAction = firstIntakeTrajectory.build();
        firstIntakeTrajectory2Action = firstIntakeTrajectory2.build();
        firstIntakeTrajectory3Action = firstIntakeTrajectory3.build();
        secondShootingTrajectoryAction = secondShootingTrajectory.build();
        secondIntakeTrajectory1Action = secondIntakeTrajectory1.build();
        secondIntakeTrajectory2Action = secondIntakeTrajectory2.build();
        secondIntakeTrajectory3Action = secondIntakeTrajectory3.build();
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
                            firstIntakeTrajectoryAction,

                            new ParallelAction(
                                    firstIntakeTrajectory2Action,
                                    intake.IntakeOn()
                            ),

                            firstIntakeTrajectory3Action,

                            new ParallelAction(
                                    secondShootingTrajectoryAction,
                                    shooter.ShooterOn()
                            ),

                            intake.IntakeOff(),

                            new SleepAction(0.8), //galima keisti


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
                            secondIntakeTrajectory1Action,
                            new ParallelAction(
                                    secondIntakeTrajectory2Action,
                                    intake.IntakeOn()
                            ),
                            secondIntakeTrajectory3Action,
                            parkingTrajectoryAction,
                            intake.IntakeOff()
                            /*new ParallelAction(
                                    shooter.ShooterOn(),
                                    new SequentialAction(
                                            new SleepAction(0.4), //galima keisti

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
                            )*/

                    )
            );

        }
    }
}