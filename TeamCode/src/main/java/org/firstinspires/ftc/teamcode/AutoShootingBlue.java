package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Sorter;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.PatternShooter;

@Config
@Autonomous(name = "Blue", group = "Autonomous")
public class AutoShootingBlue extends LinearOpMode {

    // Dashboard input: desired output pattern, e.g. "PPG", "PGP", "GPG"
    public static String TARGET_PATTERN = "PPG";

    @Override
    public void runOpMode() {
        // -------------------- INITIAL POSE --------------------
        Pose2d initialPose = new Pose2d(0, 0, 0);

        // Road Runner drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Subsystems
        Sorter sorter = new Sorter(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        PatternShooter patternShooter = new PatternShooter(shooter, transfer, sorter);

        // -------------------- TRAJECTORY --------------------
        // Simple: drive straight to x = 24
        TrajectoryActionBuilder firstShootingTrajectory = drive.actionBuilder(initialPose)
                .setReversed(false)
                .lineToX(8)
                .waitSeconds(0.1)
                .lineToX(16)
                .waitSeconds(0.1)
                .lineToX(24);

        Action firstShootingTrajectoryAction = firstShootingTrajectory.build();



        // -------------------- WAIT FOR START --------------------
        waitForStart();
        if (isStopRequested()) return;

        // -------------------- STEP 1: DRIVE + LOAD BALLS --------------------
        // Parallel:
        //  - follow trajectory
        //  - intake + sort up to 3 balls (with timeout) via Sorter
        Actions.runBlocking(
                new SequentialAction(
                new ParallelAction(
                        firstShootingTrajectoryAction,
                        sorter.intakeAndLoadThree()
                ),
                new SleepAction(2))
        );

        // -------------------- STEP 2: PREP PATTERN --------------------
        // Desired pattern from Dashboard
        String desired = TARGET_PATTERN;
        if (desired == null) desired = "PPG";
        desired = desired.toUpperCase();
        if (desired.length() != 3) desired = "PPG";

        // Just for info while testing
        telemetry.addData("Desired pattern", desired);
        telemetry.addData("Loaded pattern", sorter.getPatternString());
        telemetry.addData("Ball1", sorter.getBall1String());
        telemetry.addData("Ball2", sorter.getBall2String());
        telemetry.addData("Ball3", sorter.getBall3String());
        telemetry.update();

        // -------------------- STEP 3: SHOOT IN DESIRED PATTERN --------------------
        // PatternShooter:
        //  - reads loaded pattern from sorter
        //  - computes slot firing order to match 'desired'
        //  - spins shooter up, shoots 3 balls, spins shooter down
        Actions.runBlocking(
                patternShooter.shootPatternMid(desired)
        );

        // -------------------- STEP 4: (OPTIONAL) PARK --------------------
        // Add another trajectory here if you want to park.
    }
}
