package org.firstinspires.ftc.teamcode.subsystemTesting;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "Turret Aim At AprilTag (Simple)", group = "Vision")
public class TurretControlTest extends LinearOpMode {

    private Servo turretServo;
    private Limelight3A limelight;

    // turretDeg = servoDeg * (35/84)  => servoDeg = turretDeg / (35/84)
    private static final double GEAR_RATIO = 35.0 / 84.0;

    // Flip if turret moves opposite direction
    private static final double DIR = +1.0;

    // Optional: choose a specific tag (set to -1 to aim at the first seen tag)
    private static final int TARGET_TAG_ID = -1; // e.g. 20 or 24

    @Override
    public void runOpMode() {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0); // you said you use pipeline 0
        limelight.start();

        telemetry.addLine("Ready.");
        telemetry.addLine("Hold CROSS (X) = snap aim at AprilTag (uses tx degrees)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // manual test (optional)
            if (gamepad1.square) turretServo.setPosition(0.5);
            if (gamepad1.circle) turretServo.setPosition(0.2);
            if (gamepad1.triangle) turretServo.setPosition(0.0);

            if (gamepad1.cross) {
                aimAtTagSnap();
                sleep(1000);
            }

           // telemetry.addData("ServoPosNow", "%.4f", turretServo.getPosition());
           // telemetry.update();
        }

        try { limelight.stop(); } catch (Exception ignored) {}
    }

    private void aimAtTagSnap() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addLine("No valid Limelight result.");
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addLine("No fiducials.");
            return;
        }

        // Pick tag
        LLResultTypes.FiducialResult tag = null;
        if (TARGET_TAG_ID < 0) {
            tag = fiducials.get(0);
        } else {
            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f != null && f.getFiducialId() == TARGET_TAG_ID) {
                    tag = f;
                    break;
                }
            }
        }

        if (tag == null) {
            telemetry.addData("Target tag not found", TARGET_TAG_ID);
            return;
        }

        // Horizontal angle error to tag (degrees). Want tx -> 0 when centered.
        double txDeg = tag.getTargetXDegrees();

        // Convert txDeg into a turret correction. If tx is +, target is to the right (usually),
        // so turret should rotate right to reduce tx -> 0, hence the minus sign.
        double turretDeg = (-txDeg) * DIR;
        int tagid = tag.getFiducialId();
        if (tagid == 20) turretDeg -=4;   // if it goes the wrong way, change to: double desiredAngleDeg = tx;
        if (tagid == 24) turretDeg +=4;   // if it goes the wrong way, change to: double desiredAngleDeg = tx;

        // turretDeg -> servoDeg using gear ratio
        double servoDeg = turretDeg / GEAR_RATIO/360;

        // 360° servo: degrees -> position 0..1
        double servoPos = turretServo.getPosition() +servoDeg;

        turretServo.setPosition(servoPos);

        telemetry.addData("TagID", tag.getFiducialId());
        telemetry.addData("txDeg", "%.4f", txDeg);
        telemetry.addData("Degrees to move", "%.4f", turretDeg);
        telemetry.addData("servoPos(cmd)", "%.4f", servoPos);
        telemetry.update();
    }
}
