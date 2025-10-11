package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResultTypes.BarcodeResult;

@TeleOp(name = "Limelight QR Align", group = "Drive")
public class LimelightQRAlign extends LinearOpMode {

    // Drivetrain
    private Motor fL, fR, bL, bR;
    private MecanumDrive drive;
    private double drive_speed = 1.0;

    // Limelight
    private Limelight3A limelight;

    // Driver control
    private boolean alignEnabled = false;
    private boolean bWasPressed = false;

    // Alignment tuning
    private static final double kP_TURN = 0.03;   // turn power per degree of tx (start around 0.02–0.05)
    private static final double kI_TURN = 0.0;    // optional integral (usually 0 for this)
    private static final double kD_TURN = 0.002;  // small D can help settle
    private static final double MAX_TURN = 0.5;   // limit turn power
    private static final double TX_DEADBAND_DEG = 0.7; // within this = “good enough”
    private static final long   MAX_STALENESS_MS = 150; // only use fresh vision

    private double prevError = 0.0;
    private double integral = 0.0;
    private ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Motors (names must match your RC config) ---
        fL = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(fL, fR, bL, bR);


        // --- Limelight setup ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // ask LL for data 100Hz
        limelight.start();

        // If your Barcode pipeline is not already active, switch to it here.
        // Change the index to whatever slot your Barcode pipeline uses.
        limelight.pipelineSwitch(0); // 0 = example; set to your Barcode pipeline index

        telemetry.addLine("Limelight QR Align: ready");
        telemetry.addLine("B: toggle auto-align to QR");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {

            // --- Driver speed toggle (same as your snippet) ---
            if (gamepad1.right_trigger > 0.5) {
                drive_speed = 0.45;
            } else {
                drive_speed = 1.0;
            }

            // --- Toggle align mode with B (edge-detect) ---
            if (gamepad1.b && !bWasPressed) {
                alignEnabled = !alignEnabled;
            }
            bWasPressed = gamepad1.b;

            // --- Fetch latest Limelight result ---
            LLResult result = limelight.getLatestResult();

            // Default driver inputs
            double x = -gamepad1.left_stick_x * drive_speed;
            double y =  gamepad1.left_stick_y * drive_speed;
            double rx = -gamepad1.right_stick_x * drive_speed;

            String qrData = "—";

            if (alignEnabled && result != null && result.isValid()) {
                long ageMs = result.getStaleness();

                // Try to read QR data (optional but nice for debugging)
                List<BarcodeResult> barcodes = result.getBarcodeResults();
                if (barcodes != null && !barcodes.isEmpty()) {
                    qrData = barcodes.get(0).getData();
                }

                // Use tx to turn toward the code (center the crosshair)
                // Positive tx = target to the right => turn right (positive rx)
                if (ageMs <= MAX_STALENESS_MS) {
                    double error = result.getTx(); // degrees left/right
                    double dt = loopTimer.seconds();
                    loopTimer.reset();

                    // Simple PID (mostly P + D)
                    integral += error * dt;
                    double derivative = (error - prevError) / Math.max(dt, 1e-3);
                    prevError = error;

                    double turnCmd = kP_TURN * error + kI_TURN * integral + kD_TURN * derivative;

                    // Deadband & clamp
                    if (Math.abs(error) < TX_DEADBAND_DEG) {
                        turnCmd = 0.0;
                        integral = 0.0; // prevent windup
                    }
                    turnCmd = clamp(turnCmd, -MAX_TURN, MAX_TURN);

                    // When aligning, we usually don't want driver rotation input fighting us.
                    // Let driver still strafe/drive forward slowly if desired:
                    double assistX = x * 0.4;
                    double assistY = y * 0.4;

                    drive.driveRobotCentric(
                            assistX,
                            assistY,
                            turnCmd,
                            false
                    );

                    telemetry.addData("Align", "ON");
                    telemetry.addData("QR Data", qrData);
                    telemetry.addData("tx (deg)", "%.2f", error);
                    telemetry.addData("turnCmd", "%.3f", turnCmd);
                    telemetry.addData("staleness (ms)", ageMs);
                } else {
                    // Data too old—fall back to driver control
                    drive.driveRobotCentric(x, y, rx, false);
                    telemetry.addData("Align", "HOLD (stale data)");
                    telemetry.addData("staleness (ms)", ageMs);
                }

            } else {
                // Normal driver control
                drive.driveRobotCentric(x, y, rx, false);

                // Reset PID state when not aligning
                prevError = 0.0;
                integral = 0.0;

                telemetry.addData("Align", alignEnabled ? "WAITING (no target)" : "OFF");
                if (result != null && result.isValid()) {
                    telemetry.addData("tx (deg)", "%.2f", result.getTx());
                    telemetry.addData("staleness (ms)", result.getStaleness());
                }
            }

            // Some extra debug details for QR (optional)
            if (result != null && result.isValid()) {
                List<BarcodeResult> barcodes = result.getBarcodeResults();
                if (barcodes != null && !barcodes.isEmpty()) {
                    telemetry.addData("QR Data", qrData);
                    telemetry.addData("QR Family", barcodes.get(0).getFamily());
                } else {
                    telemetry.addData("QR", "none");
                }
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.update();
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
