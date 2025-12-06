package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Sorter;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.PatternShooter;
import org.firstinspires.ftc.teamcode.hardware.Turret;

// make sure you have this drive class:

import java.util.List;

@Config
@Autonomous(name = "autored", group = "Autonomous")
public class autoRed extends LinearOpMode {

    // Dashboard input: fallback desired output pattern, e.g. "PPG", "PGP", "GPG"
    public static String TARGET_PATTERN = "PPG";

    private Limelight3A limelight;

    // Values decided during detection phase
    private String detectedPattern = null;
    private Integer detectedTagId = null;

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

        // -------------------- LIMELIGHT SETUP --------------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);      // how often we poll for data
        limelight.pipelineSwitch(1);       // your AprilTag pipeline index
        limelight.start();                 // start vision

        // -------------------- TRAJECTORY (MIRRORED) --------------------
        // Mirror across X axis: (x, y, θ) -> (x, -y, -θ)

        TrajectoryActionBuilder cameraDetectionTrajectory = drive.actionBuilder(initialPose)
                .setReversed(true)
                // Blue: new Pose2d(-41, -15, +90°)
                // Red:  new Pose2d(-41, +15, -90°)
                .splineToSplineHeading(
                        new Pose2d(-39.5, 15, Math.toRadians(-90)),
                        Math.toRadians(0)
                );

        TrajectoryActionBuilder firstIntakingTrajectory = cameraDetectionTrajectory.endTrajectory().fresh()
                .setReversed(false)
                // Blue: -6, 1, 9  -> Red: 6, -1, -9
                .lineToY(6)
                .waitSeconds(0.1)
                .lineToY(-1)
                .waitSeconds(0.1)
                .lineToY(-9);

        TrajectoryActionBuilder secondShootingTrajectory = firstIntakingTrajectory.endTrajectory().fresh()
                .setReversed(true)
                // Blue: -17 -> Red: 17
                .lineToY(20);

        TrajectoryActionBuilder secondIntakingTrajectory = secondShootingTrajectory.endTrajectory().fresh()
                .setReversed(false)
                // Blue: new Pose2d(-51, -10, +90°)
                // Red:  new Pose2d(-51, +10, -90°)
                .splineToSplineHeading(
                        new Pose2d(-52, 14, Math.toRadians(-90)),
                        Math.toRadians(180)
                );

        TrajectoryActionBuilder secondIntakingTrajectory2 = secondIntakingTrajectory.endTrajectory().fresh()
                .setReversed(false)
                // Blue: -8, -3, 5 -> Red: 8, 3, -5
                .lineToY(8)
                .waitSeconds(0.1)
                .lineToY(3)
                .waitSeconds(0.1)
                .lineToY(-5);

        TrajectoryActionBuilder thirdShootingTrajectory = secondIntakingTrajectory2.endTrajectory().fresh()
                .setReversed(true)
                // Blue: (-33, -25) -> Red: (-33, 25)
                .strafeTo(new Vector2d(-30, 25));

        TrajectoryActionBuilder parkingTrajectory = thirdShootingTrajectory.endTrajectory().fresh()
                .setReversed(true)
                // Blue: (-50, 0) -> Red: (-50, 0) (y=0 stays the same)
                .strafeTo(new Vector2d(-50, 0));

        Action cameraDetectionTrajectoryAction = cameraDetectionTrajectory.build();
        Action firstIntakingTrajectoryAction = firstIntakingTrajectory.build();
        Action secondShootingTrajectoryAction = secondShootingTrajectory.build();
        Action secondIntakingTrajectoryAction = secondIntakingTrajectory.build();
        Action secondIntakingTrajectory2Action = secondIntakingTrajectory2.build();
        Action thirdShootingTrajectoryAction = thirdShootingTrajectory.build();
        Action parkingTrajectoryAction = parkingTrajectory.build();

        Actions.runBlocking(transfer.preset());

        // -------------------- INIT LOOP (OPTIONAL PREVIEW) --------------------
        while (!isStarted() && !isStopRequested()) {
            // Just to preview what tag it currently sees (not final decision)
            Integer previewId = getTagIdFromLimelightOnce();
            String previewPattern = mapTagIdToPattern(previewId);

            telemetry.addData("Preview Tag ID", previewId);
            telemetry.addData("Preview Pattern (from tag)", previewPattern);
            telemetry.addData("Fallback Dashboard Pattern", TARGET_PATTERN);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // -------------------- STEP 1: DETECTION PHASE (BEFORE INTAKE) --------------------
        Actions.runBlocking(
                new SequentialAction(
                        sorter.loadedBalls(),
                        cameraDetectionTrajectoryAction,
                        // Blue had turret.turretAngleMinus90()
                        // Red mirror -> turret.turretAngle90() (you may need to implement this)
                        turret.turretAngle90()
                )
        );

        String desired = detectPatternFromTags(300);

        telemetry.addData("Tag ID used", detectedTagId);
        telemetry.addData("Desired pattern (final)", desired);

        // Update telemetry with sorter state after loading
        telemetry.addData("Loaded pattern", sorter.getPatternString());
        telemetry.addData("Ball1", sorter.getBall1String());
        telemetry.addData("Ball2", sorter.getBall2String());
        telemetry.addData("Ball3", sorter.getBall3String());
        telemetry.update();

        Actions.runBlocking(
                new SequentialAction(
                        // Blue: turretAngleMinus48() -> Red: turretAngle48() (mirrored)
                        turret.turretAngle48(),
                        patternShooter.shootPatternMid(desired, "PGP"),
                        sorter.preset(),
                        new ParallelAction(
                                firstIntakingTrajectoryAction,
                                sorter.intakeAndLoadThree()
                        ),
                        new ParallelAction(
                                intake.IntakeBack(),
                                sorter.preset(),
                                secondShootingTrajectoryAction),
                        intake.IntakeOff(),
                        patternShooter.shootPatternMid(desired, "PPG"),
                        secondIntakingTrajectoryAction,
                        sorter.preset(),
                        new ParallelAction(
                                secondIntakingTrajectory2Action,
                                sorter.intakeAndLoadThree()
                        ),
                        new ParallelAction(
                                intake.IntakeBack(),
                                sorter.preset(),
                                thirdShootingTrajectoryAction),
                        intake.IntakeOff(),
                        // Blue: turretAngleMinus50() -> Red: turretAngle50()
                        turret.turretAngle48(),
                        patternShooter.shootPatternMid(desired, "PGP"),
                        new ParallelAction(
                                parkingTrajectoryAction,
                                shooter.ShooterOff()
                        )
                )
        );

        telemetry.addData("Loaded pattern", sorter.getPatternString());
        telemetry.addData("Ball1", sorter.getBall1String());
        telemetry.addData("Ball2", sorter.getBall2String());
        telemetry.addData("Ball3", sorter.getBall3String());
        telemetry.addData("desired", desired);
        telemetry.update();
    }

    /**
     * Detection loop that runs AFTER start.
     * Tries to find a valid tag for up to timeoutMs.
     * If a mapped tag is found (21/22/23), sets detectedPattern & detectedTagId.
     */
    private String detectPatternFromTags(long timeoutMs) {
        long startTime = System.currentTimeMillis();
        detectedPattern = null;
        detectedTagId = null;
        String pattern = null;

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < timeoutMs) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    int id = tags.get(0).getFiducialId();
                    pattern = mapTagIdToPattern(id);
                    if (pattern != null) {
                        detectedTagId = id;
                        detectedPattern = pattern;

                        telemetry.addData("Detected Tag ID", detectedTagId);
                        telemetry.addData("Detected Pattern", detectedPattern);
                        telemetry.update();
                        // stop as soon as we have a valid mapping
                    }
                }
            }
        }

        telemetry.addLine("No valid AprilTag detected in time, using fallback pattern.");
        telemetry.update();
        return pattern;
    }

    /**
     * Map AprilTag ID to pattern:
     * 21 -> "GPP", 22 -> "PGP", 23 -> "PPG"
     */
    private String mapTagIdToPattern(Integer id) {
        if (id == null) return null;

        switch (id) {
            case 21:
                return "GPP";
            case 22:
                return "PGP";
            case 23:
                return "PPG";
            default:
                return null;
        }
    }

    /**
     * Helper used only in the init loop for preview.
     * Single read, no timeout logic.
     */
    private Integer getTagIdFromLimelightOnce() {
        if (limelight == null) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;

        return tags.get(0).getFiducialId();
    }
}
