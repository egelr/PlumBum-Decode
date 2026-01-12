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

    public static int PIPELINE_INDEX = 1;
    public static double POLL_RATE_HZ = 100;

    // GLOBAL results
    public static volatile Integer lastTagId = null;
    public static volatile String lastPattern = null;

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    public AprilTagDetector(Limelight3A limelight, Telemetry telemetry) {
        this.limelight = limelight;
        this.telemetry = telemetry;
    }

  /*  public void init() {
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.start();
    }*/

    public static String mapTagIdToPattern(int id) {
        switch (id) {
            case 21: return "GPP";
            case 22: return "PGP";
            case 23: return "PPG";
            default: return null;
        }
    }

    /**
     * Action:
     * - checks Limelight ONCE per run()
     * - exits when tag found OR timeout reached
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
                limelight.pipelineSwitch(PIPELINE_INDEX);
                limelight.start();
            }

            long elapsed = System.currentTimeMillis() - startTime;
            packet.put("elapsedMs", elapsed);

            // -------- FAILSAFE TIMEOUT --------
            if (elapsed >= timeoutMs) {
                packet.put("timeout", true);
                telemetry.addLine("AprilTag detect TIMEOUT");
                telemetry.update();
                return false; // ✅ exit action
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

            // EXACT same logic as your working code
            int id = tags.get(0).getFiducialId();
            String pattern = mapTagIdToPattern(id);

            telemetry.addData("Seen Tag ID", id);
            telemetry.update();

            if (pattern != null) {
                lastTagId = id;
                lastPattern = pattern;

       //         telemetry.addData("Detected Tag", lastTagId);
       //         telemetry.addData("Detected Pattern", lastPattern);
       //         telemetry.update();

                packet.put("detected", true);
                packet.put("tagId", lastTagId);
                packet.put("pattern", lastPattern);

                return false; // ✅ FOUND → exit immediately
            }

            // Tag seen but not one we care about
            packet.put("ignoredTag", id);
            return true; // keep running
        }
    }
}
