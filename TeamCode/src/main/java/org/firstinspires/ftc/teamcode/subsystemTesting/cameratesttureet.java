package org.firstinspires.ftc.teamcode.subsystemTesting;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "cameratesttureet")
public class cameratesttureet extends LinearOpMode {

    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    // Encoder positions for each target (about +/-90deg)
    // Based on your note:
    //  - negative = 90° right (clockwise)
    //  + positive = 90° left  (counterclockwise)
    private static final int POS_CENTER = 0;
    private static final int POS_RIGHT  = -232;  // 90° right
    private static final int POS_LEFT   = 232;   // 90° left

    // Angle <-> ticks mapping
    private static final double MAX_ANGLE_DEG = 90.0;
    private static final double TICKS_PER_DEGREE = 232.0 / 90.0; // ≈ 2.58 ticks/deg

    // How close is “good enough” (ticks)
    private static final int POSITION_TOLERANCE = 5;

    // PID gain for internal position control (tune this!)
    public static double POSITION_P = 5.0;   // try 2–10 range to start

    // Max power while moving to position
    private static final double MOVE_POWER = 0.5;

    @Override
    public void runOpMode() {

        // ---------- TURRET SETUP ----------
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretPositionMotor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(POS_CENTER);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPositionTolerance(POSITION_TOLERANCE);
        turretMotor.setPositionPIDFCoefficients(POSITION_P);

        int targetPosition = POS_CENTER;

        // ---------- LIMELIGHT SETUP ----------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        limelight.setPollRateHz(100);    // how often we poll for data
        limelight.pipelineSwitch(0);     // your AprilTag pipeline index
        limelight.start();               // start polling

        waitForStart();

        while (opModeIsActive()) {

            // ==============================
            // 1. Read Limelight AprilTag data
            // ==============================
            LLResult result = limelight.getLatestResult();
            boolean hasTarget = result != null && result.isValid();

            if (hasTarget) {
                double tx = result.getTx();   // horizontal offset (deg)

                // Limelight: tx > 0  => tag is to the RIGHT of crosshair
                // Your turret: negative ticks = RIGHT, positive = LEFT
                // We want to rotate so the tag becomes centered, so:
                double desiredAngleDeg = -tx;   // <-- if backwards, change to +tx

                // Clamp to physical limits
                if (desiredAngleDeg > MAX_ANGLE_DEG)  desiredAngleDeg = MAX_ANGLE_DEG;
                if (desiredAngleDeg < -MAX_ANGLE_DEG) desiredAngleDeg = -MAX_ANGLE_DEG;

                // Convert angle -> encoder ticks
                targetPosition = (int) Math.round(desiredAngleDeg * TICKS_PER_DEGREE);

                turretMotor.setTargetPosition(targetPosition);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(MOVE_POWER);

            } else {
                // =================================
                // 2. No target – use manual buttons
                // =================================

                if (gamepad1.a) {
                    targetPosition = POS_CENTER;
                    turretMotor.setTargetPosition(targetPosition);
                    turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turretMotor.setPower(MOVE_POWER);
                }

                if (gamepad1.b) {
                    targetPosition = POS_RIGHT;
                    turretMotor.setTargetPosition(targetPosition);
                    turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turretMotor.setPower(MOVE_POWER);
                }

                if (gamepad1.y) {
                    targetPosition = POS_LEFT;
                    turretMotor.setTargetPosition(targetPosition);
                    turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turretMotor.setPower(MOVE_POWER);
                }
            }

            // Optional: cut power when we’ve reached target
            if (!turretMotor.isBusy()) {
                turretMotor.setPower(0);
            }

            // ---------- TELEMETRY ----------
            int currentPos = turretMotor.getCurrentPosition();
            int error = targetPosition - currentPos;

            telemetry.addData("Has Target", hasTarget);
            telemetry.addData("Current Pos", currentPos);
            telemetry.addData("Target Pos", targetPosition);
            telemetry.addData("Error", error);
            telemetry.update();
        }
    }
}