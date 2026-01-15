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
@Autonomous(name = "BLUE", group = "Autonomous")
public class autoBlue extends LinearOpMode {

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

        TrajectoryActionBuilder cameraDetectionTrajectory = drive.actionBuilder(initialPose)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-50, -18), Math.toRadians(90));

        TrajectoryActionBuilder firstIntakingTrajectory = cameraDetectionTrajectory.endTrajectory().fresh()
                .setReversed(false)
                .lineToY(-7)
                .waitSeconds(0.1)
                .lineToY(0)
                .waitSeconds(0.1)
                .lineToY(8);

        TrajectoryActionBuilder secondShootingTrajectory = firstIntakingTrajectory.endTrajectory().fresh()
                .setReversed(true)
                .lineToY(-17);

        TrajectoryActionBuilder secondIntakingTrajectory = secondShootingTrajectory.endTrajectory().fresh()
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-75, -14), Math.toRadians(93.5));

        TrajectoryActionBuilder secondIntakingTrajectory2 = secondIntakingTrajectory.endTrajectory().fresh()
                .setReversed(false)
                .lineToY(-8)
                .waitSeconds(0.1)
                .lineToY(-3.5)
                .waitSeconds(0.1)
                .lineToY(8);

        TrajectoryActionBuilder thirdShootingTrajectory = secondIntakingTrajectory2.endTrajectory().fresh()
                .setReversed(true)
                .strafeTo(new Vector2d(-46, -20));

        TrajectoryActionBuilder thirdIntakingTrajectory = thirdShootingTrajectory.endTrajectory().fresh()
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-98, -14), Math.toRadians(95));

        TrajectoryActionBuilder thirdIntakingTrajectory2 = thirdIntakingTrajectory.endTrajectory().fresh()
                .setReversed(false)
                .lineToY(-10)
                .waitSeconds(0.1)
                .lineToY(-4)
                .waitSeconds(0.1)
                .lineToY(5);

        Action cameraDetectionTrajectoryAction = cameraDetectionTrajectory.build();
        Action firstIntakingTrajectoryAction = firstIntakingTrajectory.build();
        Action secondShootingTrajectoryAction = secondShootingTrajectory.build();
        Action secondIntakingTrajectoryAction = secondIntakingTrajectory.build();
        Action secondIntakingTrajectory2Action = secondIntakingTrajectory2.build();
        Action thirdShootingTrajectoryAction = thirdShootingTrajectory.build();
        Action thirdIntakingTrajectoryAction = thirdIntakingTrajectory.build();
        Action thirdIntakingTrajectory2Action = thirdIntakingTrajectory2.build();

        // Presets
        Actions.runBlocking(transfer.preset());
        Actions.runBlocking(turret.left());

        // -------------------- INIT LOOP (PREVIEW) --------------------
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Fallback Pattern", TARGET_PATTERN);

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

        // -------------------- STEP 2: DETECT --------------------
        if (AprilTagDetector.lastPattern == null) AprilTagDetector.lastPattern = TARGET_PATTERN;
        if (AprilTagDetector.lastPattern.length() != 3) AprilTagDetector.lastPattern = "PPG";

        Integer usedTagId = AprilTagDetector.lastTagId;

        telemetry.addData("Tag ID used", usedTagId);
        telemetry.addData("Desired pattern (final)", AprilTagDetector.lastPattern);

        telemetry.addData("Loaded pattern", sorter.getPatternString());
        telemetry.addData("Ball1", sorter.getBall1String());
        telemetry.addData("Ball2", sorter.getBall2String());
        telemetry.addData("Ball3", sorter.getBall3String());
        telemetry.update();

        // -------------------- STEP 3: RUN SEQUENCE --------------------
        Actions.runBlocking(
                new SequentialAction(
                        turret.halfRight1(),
                        intake.IntakeHolding(),
                        patternShooter.shootPatternMid(AprilTagDetector.lastPattern, "PGP"),
                        sorter.preset(),
                        new ParallelAction(
                                firstIntakingTrajectoryAction,
                                sorter.intakeAndLoadThree(),
                                turret.halfRight2()
                        ),
                        new ParallelAction(
                                intake.IntakeHolding(),
                                sorter.preset(),
                                secondShootingTrajectoryAction
                        ),
                        patternShooter.shootPatternMid(AprilTagDetector.lastPattern, "PPG"),
                        intake.IntakeOff(),
                        new ParallelAction(
                                secondIntakingTrajectoryAction,
                                sorter.preset()
                        ),
                        new ParallelAction(
                                secondIntakingTrajectory2Action,
                                sorter.intakeAndLoadThree(),
                                shooter.ShooterOn(),
                                turret.halfRight3()
                        ),
                        new ParallelAction(
                                intake.IntakeHolding(),
                                thirdShootingTrajectoryAction
                        ),
                        patternShooter.shootPatternMid(AprilTagDetector.lastPattern, "PGP"),
                        intake.IntakeOff(),
                        new ParallelAction(
                                thirdIntakingTrajectoryAction,
                                sorter.preset()
                        ),
                        new ParallelAction(
                                thirdIntakingTrajectory2Action,
                                sorter.intakeAndLoadThree()
                        )
                )
        );

        telemetry.addData("DONE", true);
        telemetry.addData("desired", AprilTagDetector.lastPattern);
        telemetry.addData("tag", AprilTagDetector.lastTagId);
        telemetry.update();
    }
}
