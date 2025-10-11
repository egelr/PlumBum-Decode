package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name="Camera Test")
public class CameraTest extends OpMode {

    // --- Drivetrain ---
    private Motor fL, fR, bL, bR;
    private MecanumDrive drive;

    // --- Limelight 3A ---
    private Limelight3A limelight;

    // --- FTCLib PID (turn only) ---
    private final PIDController turnPID = new PIDController(0.02, 0.0, 0.002);

    // --- Constants ---
    private static final double TX_TARGET = 0.0; // Center horizontally
    private static final double MAX_TURN = 0.5;
    private static final double TX_TOL_DEG = 1.0;

    private enum Mode { MANUAL, ALIGNING }
    private Mode mode = Mode.MANUAL;

    // Latest vision data for telemetry
    private boolean tagVisible = false;
    private int seenTagId = -1;
    private double tx = 0, ty = 0, ta = 0;
    private Double planar_m = null;
    private long stalenessMs = -1;

    @Override
    public void init() {
        // Motors: GoBILDA 435 RPM, BRAKE
        fL = new Motor(hardwareMap, "leftFront",  Motor.GoBILDA.RPM_435);
        fR = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        bL = new Motor(hardwareMap, "leftBack",   Motor.GoBILDA.RPM_435);
        bR = new Motor(hardwareMap, "rightBack",  Motor.GoBILDA.RPM_435);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // No inversion — use default rotation directions
        drive = new MecanumDrive(fL, fR, bL, bR);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        // PID setup
        turnPID.setSetPoint(TX_TARGET);
        turnPID.setTolerance(TX_TOL_DEG);

        telemetry.addLine("Init OK. Hold SQUARE to turn-align to Tag 20/24. Planar distance shown when visible.");
    }

    @Override
    public void loop() {
        boolean square = gamepad1.square;
        if (square) mode = Mode.ALIGNING;
        else if (mode == Mode.ALIGNING) mode = Mode.MANUAL;

        // Always read vision and show telemetry when visible
        readVisionAndPrepareTelemetry();

        if (mode == Mode.ALIGNING && tagVisible) {
            // PID correction applied to rotation (rx), not strafe
            double turnCmd = clamp(turnPID.calculate(tx), -MAX_TURN, MAX_TURN);
            drive.driveRobotCentric(0.0, 0.0, turnCmd);

            telemetry.addData("Mode", "ALIGNING (hold □)");
            telemetry.addData("cmd turn", "%.2f", turnCmd);

            if (turnPID.atSetPoint()) gamepad1.rumble(100);
        } else if (mode == Mode.MANUAL) {
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;
            drive.driveRobotCentric(x, y, rx);
            telemetry.addData("Mode", "MANUAL");
        } else {
            drive.stop();
            telemetry.addData("Mode", "ALIGNING (hold □)");
        }

        // Telemetry if tag visible
        if (tagVisible) {
            boolean alignedHoriz = Math.abs(tx - TX_TARGET) <= TX_TOL_DEG;
            telemetry.addData("Tag 20/24 visible", true);
            telemetry.addData("Tag ID", seenTagId);
            telemetry.addData("tx / ty / ta", "%.2f / %.2f / %.2f", tx, ty, ta);
            telemetry.addData("Aligned (turn)", alignedHoriz);
            if (planar_m != null) {
                telemetry.addData("Planar distance (m)", "%.3f", planar_m);
                telemetry.addData("Planar distance (cm)", "%.1f", planar_m * 100.0);
            }
            telemetry.addData("staleness (ms)", stalenessMs);
        } else {
            telemetry.addData("Tag 20/24 visible", false);
        }

        telemetry.update();
    }

    /** Reads Limelight, selects first tag 20/24, caches values for telemetry (always runs). */
    private void readVisionAndPrepareTelemetry() {
        tagVisible = false;
        seenTagId = -1;
        planar_m = null;
        stalenessMs = -1;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        FiducialResult target = firstTag20or24(result.getFiducialResults());
        if (target == null) return;

        tagVisible = true;
        seenTagId = target.getFiducialId();
        tx = target.getTargetXDegrees();
        ty = target.getTargetYDegrees();
        ta = result.getTa();
        stalenessMs = result.getStaleness();

        // Planar distance only (camera and tag on same plane)
        Pose3D poseTagSpace = target.getRobotPoseTargetSpace();
        if (poseTagSpace != null && poseTagSpace.getPosition() != null) {
            double px = poseTagSpace.getPosition().x;
            double py = poseTagSpace.getPosition().y;
            planar_m = Math.hypot(px, py);
        }
    }

    private FiducialResult firstTag20or24(List<FiducialResult> list) {
        if (list == null) return null;
        for (FiducialResult f : list) {
            int id = f.getFiducialId();
            if (id == 20 || id == 24) return f;
        }
        return null;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
