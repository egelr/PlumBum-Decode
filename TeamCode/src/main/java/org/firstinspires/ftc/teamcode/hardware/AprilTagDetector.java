package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Config
public class AprilTagDetector {

    // Pipelines
    public static int PATTERN_PIPELINE_INDEX = 1;   // your current pattern pipeline
    public static int APRILTAG_PIPELINE_INDEX = 0;  // april tag pipeline

    // GLOBAL results
    public static volatile Integer lastTagId = null;
    public static volatile String lastPattern = null;

    // NEW: GLOBAL angle from AprilTag (horizontal error, degrees)
    // This is equivalent to tag.getTargetXDegrees() (tx)
    public static volatile Double lastTagTxDeg = null;

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    public AprilTagDetector(Limelight3A limelight, Telemetry telemetry) {
        this.limelight = limelight;
        this.telemetry = telemetry;
    }

    public static String mapTagIdToPattern(int id) {
        switch (id) {
            case 21: return "GPP";
            case 22: return "PGP";
            case 23: return "PPG";
            default: return null;
        }
    }

    // -------------------- NEW ACTION: switch pipeline 1 -> 0 --------------------
    /**
     * One-shot Action to switch Limelight to AprilTag pipeline (0).
     * Call this in auto before you start aiming / reading tx.
     */
    public Action switchToAprilTagPipeline() {
        return new SwitchPipelineOnceAction(APRILTAG_PIPELINE_INDEX);
    }

    /**
     * One-shot Action to switch Limelight to Pattern pipeline (1) if you ever need to go back.
     */
    public Action switchToPatternPipeline() {
        return new SwitchPipelineOnceAction(PATTERN_PIPELINE_INDEX);
    }

    private class SwitchPipelineOnceAction implements Action {
        private final int pipelineIndex;
        private boolean done = false;

        SwitchPipelineOnceAction(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!done) {
                limelight.setPollRateHz(100);
                limelight.pipelineSwitch(pipelineIndex);
                limelight.start(); // safe to call even if already started

                packet.put("pipelineSwitchedTo", pipelineIndex);
                telemetry.addData("Limelight pipeline", pipelineIndex);
                telemetry.update();

                done = true;
            }
            return false; // one-shot
        }
    }

    // -------------------- YOUR EXISTING ACTION (pattern pipeline) --------------------
    /**
     * Action:
     * - checks Limelight repeatedly
     * - exits when tag found OR timeout reached
     *
     * Uses PATTERN_PIPELINE_INDEX (1)
     */
    public Action detectWithTimeout(double timeoutSeconds) {
        return new DetectOnceWithTimeoutAction(timeoutSeconds);
    }

    private class DetectOnceWithTimeoutAction implements Action {

        private final long timeoutMs;
        private long startTime = -1;

        DetectOnceWithTimeoutAction(double timeoutSeconds) {
            this.timeoutMs = (long) (timeoutSeconds * 1000);
        }

        @Override
        public boolean run(TelemetryPacket packet) {

            if (startTime < 0) {
                startTime = System.currentTimeMillis();
                limelight.setPollRateHz(100);
                limelight.pipelineSwitch(PATTERN_PIPELINE_INDEX);
                limelight.start();
            }

            long elapsed = System.currentTimeMillis() - startTime;
            packet.put("elapsedMs", elapsed);

            // -------- FAILSAFE TIMEOUT --------
            if (elapsed >= timeoutMs) {
                packet.put("timeout", true);
                telemetry.addLine("AprilTag detect TIMEOUT");
                telemetry.update();
                return false;
            }

            // -------- SINGLE CAMERA READ --------
            LLResult result = limelight.getLatestResult();
            if (result == null) {
                packet.put("ll", "no result");
                return true;
            }

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (tags == null || tags.isEmpty()) {
                packet.put("tags", 0);
                return true;
            }

            int id = tags.get(0).getFiducialId();
            String pattern = mapTagIdToPattern(id);

            telemetry.addData("Seen Tag ID", id);
            telemetry.update();

            if (pattern != null) {
                lastTagId = id;
                lastPattern = pattern;

                packet.put("detected", true);
                packet.put("tagId", lastTagId);
                packet.put("pattern", lastPattern);

                return false; // FOUND
            }

            packet.put("ignoredTag", id);
            return true;
        }
    }

    // -------------------- NEW ACTION: read AprilTag + store tx angle --------------------
    /**
     * Action:
     * - ensures AprilTag pipeline (0)
     * - reads first seen tag (or you can filter later if you want)
     * - stores lastTagId and lastTagTxDeg
     * - exits when found OR timeout
     */
    public Action detectAprilTagAngleWithTimeout(double timeoutSeconds) {
        return new DetectAprilTagAngleWithTimeoutAction(timeoutSeconds);
    }

    private class DetectAprilTagAngleWithTimeoutAction implements Action {

        private final long timeoutMs;
        private long startTime = -1;

        DetectAprilTagAngleWithTimeoutAction(double timeoutSeconds) {
            this.timeoutMs = (long) (timeoutSeconds * 1000);
        }

        @Override
        public boolean run(TelemetryPacket packet) {

            if (startTime < 0) {
                startTime = System.currentTimeMillis();
                limelight.setPollRateHz(100);
                limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
                limelight.start();
            }

            long elapsed = System.currentTimeMillis() - startTime;
            packet.put("elapsedMs", elapsed);

            if (elapsed >= timeoutMs) {
                packet.put("timeout", true);
                telemetry.addLine("AprilTag ANGLE detect TIMEOUT");
                telemetry.update();
                return false;
            }

            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                packet.put("ll", "no/invalid result");
                return true;
            }

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) {
                packet.put("fiducials", 0);
                return true;
            }

            // pick first tag (same style as your teleop example)
            LLResultTypes.FiducialResult tag = fiducials.get(0);

            int id = tag.getFiducialId();
            double txDeg = tag.getTargetXDegrees(); // horizontal angle error (degrees)

            lastTagId = id;
            if (id == 24){
                lastTagTxDeg = txDeg +2;
            }
            else {
                lastTagTxDeg = txDeg -2;
            }


            packet.put("detected", true);
            packet.put("tagId", id);
            packet.put("txDeg", txDeg);

            telemetry.addData("AprilTag ID", id);
            telemetry.addData("txDeg", "%.3f", txDeg);
            telemetry.update();

            return false; // FOUND -> exit
        }
    }
}