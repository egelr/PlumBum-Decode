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

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.hardware.Sorter;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.PatternShooter;

import java.util.List;

@Config
@Autonomous(name = "autoblue", group = "Autonomous")
public class autoBlue extends LinearOpMode {

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

        // -------------------- LIMELIGHT SETUP --------------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);      // how often we poll for data
        limelight.pipelineSwitch(1);       // your AprilTag pipeline index
        limelight.start();                 // start vision

        // -------------------- TRAJECTORY --------------------
        TrajectoryActionBuilder firstShootingTrajectory = drive.actionBuilder(initialPose)
                .setReversed(false)
                .lineToX(8)
                .waitSeconds(0.1)
                .lineToX(16)
                .waitSeconds(0.1)
                .lineToX(24);

        Action firstShootingTrajectoryAction = firstShootingTrajectory.build();

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
        // Robot stands still, we give Limelight some time to see a tag.
        // Example: 1500 ms window – tweak as you like.
        detectPatternFromTags(1500);

        // Decide final desired pattern: tag -> pattern, else Dashboard fallback
        String desired = detectedPattern;
        if (desired == null) {
            desired = TARGET_PATTERN;
        }
        if (desired == null) desired = "PPG";
        desired = desired.toUpperCase();
        if (desired.length() != 3) desired = "PPG";

        telemetry.addData("Tag ID used", detectedTagId);
        telemetry.addData("Desired pattern (final)", desired);
        telemetry.update();

        // (Optional) a tiny pause after detection before moving/intaking, if you want:
        sleep(200); // 0.2s – adjust or remove if not needed

        // -------------------- STEP 2: DRIVE + LOAD BALLS --------------------
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                firstShootingTrajectoryAction,
                                sorter.intakeAndLoadThree()
                        ),
                        new SleepAction(2)
                )
        );

        // Update telemetry with sorter state after loading
        telemetry.addData("Loaded pattern", sorter.getPatternString());
        telemetry.addData("Ball1", sorter.getBall1String());
        telemetry.addData("Ball2", sorter.getBall2String());
        telemetry.addData("Ball3", sorter.getBall3String());
        telemetry.update();

        // -------------------- STEP 3: SHOOT IN DESIRED PATTERN --------------------
        Actions.runBlocking(
                patternShooter.shootPatternMid(desired)
        );

        // -------------------- STEP 4: (OPTIONAL) PARK --------------------
        // Add another trajectory here if you want to park.
    }

    /**
     * Detection loop that runs AFTER start.
     * Tries to find a valid tag for up to timeoutMs.
     * If a mapped tag is found (21/22/23), sets detectedPattern & detectedTagId.
     */
    private void detectPatternFromTags(long timeoutMs) {
        long startTime = System.currentTimeMillis();
        detectedPattern = null;
        detectedTagId = null;

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < timeoutMs) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    int id = tags.get(0).getFiducialId();
                    String pattern = mapTagIdToPattern(id);
                    if (pattern != null) {
                        detectedTagId = id;
                        detectedPattern = pattern;

                        telemetry.addData("Detected Tag ID", detectedTagId);
                        telemetry.addData("Detected Pattern", detectedPattern);
                        telemetry.update();
                        return; // stop as soon as we have a valid mapping
                    }
                }
            }

            // Small delay to avoid spamming
            sleep(10);
        }

        // If we get here, no valid tag was mapped in time
        telemetry.addLine("No valid AprilTag detected in time, using fallback pattern.");
        telemetry.update();
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
