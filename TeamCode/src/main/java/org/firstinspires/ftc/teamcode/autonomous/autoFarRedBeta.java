package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Sorter;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.PatternShooter;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.hardware.AprilTagDetector;

@Config
@Autonomous(name = "beta far red", group = "Autonomous")
public class autoFarRedBeta extends LinearOpMode {

    // Fallback desired output pattern if no tag is mapped
    public static String TARGET_PATTERN = "PPG";

    // How long to scan for tags (ms)
    public static long DETECT_TIMEOUT_MS = 300;

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
        Turret turret = new Turret(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // -------------------- LIMELIGHT (WORKING-STYLE DETECTOR) --------------------
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        AprilTagDetector detector = new AprilTagDetector(limelight, telemetry);
        //detector.init(); // same as your old working setup: pollRate, pipelineSwitch, start

        // -------------------- TRAJECTORY (SIMPLE TEST) --------------------
        TrajectoryActionBuilder cameraDetectionTrajectory = drive.actionBuilder(initialPose)
                .setReversed(false)
                // Blue: strafeToLinearHeading(new Vector2d(-50, -18), +90°)
                // Red:  strafeToLinearHeading(new Vector2d(-50,  18), -90°)
                .strafeToLinearHeading(new Vector2d(25, -20), Math.toRadians(-86));
        TrajectoryActionBuilder ballIntakeTrajectory = cameraDetectionTrajectory.endTrajectory().fresh()
                .setReversed(false)
                .lineToY(-24)
                .waitSeconds(0.1)
                .lineToY(-30)
                .waitSeconds(0.1)
                .lineToY(-36);
        TrajectoryActionBuilder shooting1Trajectory = ballIntakeTrajectory.endTrajectory().fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(10, -5), Math.toRadians(-10));
        TrajectoryActionBuilder ballIntake2Trajectory = shooting1Trajectory.endTrajectory().fresh()
                .setReversed(false)
                .turn(Math.toRadians(-91));
        TrajectoryActionBuilder ballIntake2Trajectory2 = ballIntake2Trajectory.endTrajectory().fresh()
                .setReversed(false)
                .lineToY(-45);
        TrajectoryActionBuilder shooting2Trajectory = ballIntake2Trajectory2.endTrajectory().fresh()
                .setReversed(true)
                .lineToY(-5)
                .turn(Math.toRadians(93));
        TrajectoryActionBuilder parkingTrajectory = shooting2Trajectory.endTrajectory().fresh()
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(10, -30), Math.toRadians(0));




        Action cameraDetectionTrajectoryAction = cameraDetectionTrajectory.build();
        Action ballIntakeTrajectoryAction = ballIntakeTrajectory.build();
        Action shooting1TrajectoryAction = shooting1Trajectory.build();
        Action ballIntake2TrajectoryAction = ballIntake2Trajectory.build();
        Action ballIntake2Trajectory2Action = ballIntake2Trajectory2.build();
        Action shooting2TrajectoryAction = shooting2Trajectory.build();
        Action parkingTrajectoryAction = parkingTrajectory.build();


        // Preset transfer
        Actions.runBlocking(transfer.preset());
        Actions.runBlocking(turret.preset());

        // -------------------- INIT LOOP (PREVIEW) --------------------
        while (!isStarted() && !isStopRequested()) {
            //Integer previewId = detector.getTagIdFromLimelightOnce();
            //String previewPattern = AprilTagDetector.mapTagIdToPattern(previewId);

            //telemetry.addData("Preview Tag ID", previewId);
            //telemetry.addData("Preview Pattern (mapped)", previewPattern);
            telemetry.addData("Fallback Pattern", TARGET_PATTERN);

            // Also show globals (should be null in init unless you call detect)
            telemetry.addData("Global lastTagId", AprilTagDetector.lastTagId);
            telemetry.addData("Global lastPattern", AprilTagDetector.lastPattern);

            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // -------------------- STEP 1: MOVE + TURRET --------------------
        Actions.runBlocking(
                new ParallelAction(
                        shooter.ShooterOnFar(),
                        sorter.loadedBalls(),
                        detector.detectWithTimeout(4)
                )
        );

        // -------------------- STEP 2: DETECT (SAME STYLE AS YOUR WORKING CODE) --------------------


        // Fallback logic (same idea as before)
        if (AprilTagDetector.lastPattern == null) AprilTagDetector.lastPattern = TARGET_PATTERN;
        //if (desired == null) desired = "PPG";
        //desired = desired.toUpperCase();
        if (AprilTagDetector.lastPattern.length() != 3) AprilTagDetector.lastPattern = "PPG";

        Integer usedTagId = AprilTagDetector.lastTagId;

        telemetry.addData("Tag ID used", usedTagId);
        telemetry.addData("Desired pattern (final)", AprilTagDetector.lastPattern);

        telemetry.addData("Loaded pattern", sorter.getPatternString());
        telemetry.addData("Ball1", sorter.getBall1String());
        telemetry.addData("Ball2", sorter.getBall2String());
        telemetry.addData("Ball3", sorter.getBall3String());
        telemetry.update();

        // -------------------- STEP 3: SHOOT ONCE (TEST) --------------------
        Actions.runBlocking(
                new SequentialAction(
                        turret.farRight(),
                        intake.IntakeHolding(),
                        patternShooter.shootPatternFar(AprilTagDetector.lastPattern, "PGP"),
                        cameraDetectionTrajectoryAction,
                        new ParallelAction(
                                sorter.intakeAndLoadThree(),
                                ballIntakeTrajectoryAction),
                        new ParallelAction(
                                shooter.ShooterOnFar(),
                                shooting1TrajectoryAction
                                ),
                        intake.IntakeHolding(),
                        patternShooter.shootPatternFar(AprilTagDetector.lastPattern, "PPG"),
                        ballIntake2TrajectoryAction,
                        new ParallelAction(
                                sorter.intakeAndLoadThree(),
                                ballIntake2Trajectory2Action
                        ),
                        shooting2TrajectoryAction,
                        intake.IntakeHolding(),
                        patternShooter.shootPatternFar(AprilTagDetector.lastPattern, "PGP"),
                        parkingTrajectoryAction
                )
        );

        telemetry.addData("DONE", true);
        telemetry.addData("desired", AprilTagDetector.lastPattern);
        telemetry.addData("tag", AprilTagDetector.lastTagId);
        telemetry.update();
    }
}
