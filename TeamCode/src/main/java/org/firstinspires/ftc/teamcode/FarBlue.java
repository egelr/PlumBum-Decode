package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.TransferArm;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Intake;


@Config
@Autonomous(name = "FarBlue", group = "Autonomous")
public class FarBlue extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        TransferArm transferArm = new TransferArm(hardwareMap);

        TrajectoryActionBuilder parkingTrajectory = drive.actionBuilder(initialPose)
                .setReversed(false)
                .turn(Math.toRadians(-20))
                .strafeTo(new Vector2d(0, 35));

        Action parkingTrajectoryAction;



        parkingTrajectoryAction = parkingTrajectory.build();
        Actions.runBlocking(transferArm.preset());
        Actions.runBlocking(intake.preset());

        while (!isStopRequested() && !opModeIsActive()) {

            waitForStart();

            if (isStopRequested()) return;


            Actions.runBlocking(
                    new SequentialAction(
                            shooter.ShooterOnFar(),

                            new SleepAction(3),

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

                            parkingTrajectoryAction
                    )
            );

        }
    }
}