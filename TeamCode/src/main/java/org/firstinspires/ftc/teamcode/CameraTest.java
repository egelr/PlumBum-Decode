package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "CameraTest", group = "Vision")
public class CameraTest extends LinearOpMode {

    private MecanumDrive drive;
    private Limelight3A limelight;
    private PIDController turnPID;

    // PID constants (tune as needed)
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.02;
    private static final double MIN_TURN_POWER = 0.1;

    private double tx = 0.0;
    private double distanceM = -1;
    private int currentTagId = -1;
    private boolean tagVisible = false;
    private double K = 0.6;
    @Override
    public void runOpMode() throws InterruptedException {

        // === Initialize drivetrain ===
        MotorEx frontLeft  = new MotorEx(hardwareMap, "leftFront");
        MotorEx frontRight = new MotorEx(hardwareMap, "rightFront");
        MotorEx backLeft   = new MotorEx(hardwareMap, "leftBack");
        MotorEx backRight  = new MotorEx(hardwareMap, "rightBack");

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // === Initialize Limelight ===
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // === PID Controller ===
        turnPID = new PIDController(kP, kI, kD);
        turnPID.setTolerance(3.0); // degrees

        telemetry.addLine("Ready — Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            tagVisible = false;
            currentTagId = -1;
            tx = 0;
            distanceM = -1;

            // === Find first relevant tag (20 or 24) ===
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId();
                    if (id == 20 || id == 24) {
                        tagVisible = true;
                        currentTagId = id;
                        tx = fiducial.getTargetXDegrees();
                        distanceM =  K /Math.sqrt(fiducial.getTargetArea());
                        }
                        break; // first valid tag
                    }
                }


            // === PID-controlled rotation to center tag ===
            if (tagVisible) {
                double turnPower = turnPID.calculate(tx, 0);
                turnPower = Range.clip(turnPower, -0.3, 0.3);

                if (!turnPID.atSetPoint() && Math.abs(turnPower) < MIN_TURN_POWER) {
                    turnPower = Math.copySign(MIN_TURN_POWER, turnPower);
                }

                if (!turnPID.atSetPoint()) {
                    drive.driveRobotCentric(0, 0, turnPower);
                } else {
                    drive.stop();
                }
            } else {
                drive.stop(); // No tag visible
            }

            // === Telemetry ===
            telemetry.addLine("---- Limelight Telemetry ----");
            telemetry.addData("Tag Visible", tagVisible);
            telemetry.addData("Tag ID", currentTagId);
            telemetry.addData("tx (° offset)", "%.2f", tx);
            telemetry.addData("Forward Distance (m)", "%.4f", distanceM);
            telemetry.addData("PID At Setpoint", turnPID.atSetPoint());
            telemetry.addLine("-----------------------------");

           /* if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    Pose3D pose = fiducial.getRobotPoseTargetSpace();
                    if (pose != null) {
                        telemetry.addData("Fiducial " + fiducial.getFiducialId(),
                                String.format("Y=%.2f m | Xdeg=%.1f° Ydeg=%.1f°",
                                       // pose.getY(),
                                        fiducial.getTargetXDegrees(),
                                        fiducial.getTargetYDegrees()));
                    }
                }
            }
*/
            telemetry.update();
        }
    }
}