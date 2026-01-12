package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
@Autonomous(name = "autotest", group = "Autonomous")
public class autotest extends LinearOpMode {

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
                .setReversed(true)
                // Blue: strafeToLinearHeading(new Vector2d(-50, -18), +90°)
                // Red:  strafeToLinearHeading(new Vector2d(-50,  18), -90°)
                .strafeToLinearHeading(new Vector2d(-48, 18), Math.toRadians(-90));

        TrajectoryActionBuilder firstIntakingTrajectory = cameraDetectionTrajectory.endTrajectory().fresh()
                .setReversed(false)
                // Blue: -6, 1, 9  -> Red: 6, -1, -9
                .lineToY(6)
                .waitSeconds(0.1)
                .lineToY(-1)
                .waitSeconds(0.1)
                .lineToY(-10);

        TrajectoryActionBuilder secondShootingTrajectory = firstIntakingTrajectory.endTrajectory().fresh()
                .setReversed(true)
                // Blue: -17 -> Red: 17
                .lineToY(17);

        TrajectoryActionBuilder secondIntakingTrajectory = secondShootingTrajectory.endTrajectory().fresh()
                .setReversed(false)
                // Blue: (-77, -14, 92°) -> Red: (-77, 14, -92°)
                .strafeToLinearHeading(new Vector2d(-77, 14), Math.toRadians(-92));

        TrajectoryActionBuilder secondIntakingTrajectory2 = secondIntakingTrajectory.endTrajectory().fresh()
                .setReversed(false)
                // Blue: -8, -3, 8 -> Red: 8, 3, -8
                .lineToY(8)
                .waitSeconds(0.1)
                .lineToY(3)
                .waitSeconds(0.1)
                .lineToY(-8);

        TrajectoryActionBuilder thirdShootingTrajectory = secondIntakingTrajectory2.endTrajectory().fresh()
                .setReversed(true)
                // Blue: (-50, -17) -> Red: (-50, 17)
                .strafeTo(new Vector2d(-50, 17));

        TrajectoryActionBuilder parkingTrajectory = thirdShootingTrajectory.endTrajectory().fresh()
                .setReversed(true)
                // y=0 stays the same
                .strafeTo(new Vector2d(-50, 0));

        Action cameraDetectionTrajectoryAction = cameraDetectionTrajectory.build();
        Action firstIntakingTrajectoryAction = firstIntakingTrajectory.build();
        Action secondShootingTrajectoryAction = secondShootingTrajectory.build();
        Action secondIntakingTrajectoryAction = secondIntakingTrajectory.build();
        Action secondIntakingTrajectory2Action = secondIntakingTrajectory2.build();
        Action thirdShootingTrajectoryAction = thirdShootingTrajectory.build();
        Action parkingTrajectoryAction = parkingTrajectory.build();

        // Preset transfer
        Actions.runBlocking(transfer.preset());
        Actions.runBlocking(turret.left());

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
                        shooter.ShooterOn(),
                        sorter.loadedBalls(),
                        cameraDetectionTrajectoryAction,
                                detector.detectWithTimeout(2)
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
                        turret.halfLeft(),
                        intake.IntakeHolding(),
                        patternShooter.shootPatternMid(AprilTagDetector.lastPattern, "PGP"),
                        sorter.preset(),
                        new ParallelAction(
                                firstIntakingTrajectoryAction,
                                sorter.intakeAndLoadThree()
                                //,shooter.ShooterOn()
                        ),
                        new ParallelAction(
                                intake.IntakeBack(),
                                sorter.preset(),
                                secondShootingTrajectoryAction
                        ),
                        intake.IntakeOff(),
                        patternShooter.shootPatternMid(AprilTagDetector.lastPattern, "PPG")
                )
        );

        telemetry.addData("DONE", true);
        telemetry.addData("desired", AprilTagDetector.lastPattern);
        telemetry.addData("tag", AprilTagDetector.lastTagId);
        telemetry.update();
    }
}
