package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ShooterTesting")
public class ShooterTesting extends LinearOpMode {

    private DcMotorEx shooterLeft, shooterRight;

    // Motor specs
    private static final int CPR = 28;             // encoder counts per rev (GoBILDA 6000RPM)
    private static final int MAX_RPM = 6000;
    private static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;  // ~2800

    // Start at ~70% power
    private double targetVelocity = MAX_TICKS_PER_SEC * 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        // PIDF coefficients tuned for 6000 RPM
        shooterLeft.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);
        shooterRight.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);

        waitForStart();

        while (opModeIsActive()) {

            // --- Gamepad controls ---
            if (gamepad1.dpad_up) {
                targetVelocity += 50;  // increase by 50 ticks/sec
            }
            if (gamepad1.dpad_down) {
                targetVelocity -= 50;  // decrease by 50 ticks/sec
            }
            if (gamepad1.a) {
                targetVelocity = 0;    // stop shooter
            }
            if (gamepad1.b) {
                targetVelocity = MAX_TICKS_PER_SEC; // full speed
            }

            // Clamp target
            targetVelocity = Math.max(0, Math.min(MAX_TICKS_PER_SEC, targetVelocity));

            // Apply velocity
            shooterLeft.setVelocity(targetVelocity);
            shooterRight.setVelocity(targetVelocity);

            // Telemetry for driver feedback
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Target RPM", (targetVelocity / CPR) * 60);
            telemetry.addData("Left Vel", shooterLeft.getVelocity());
            telemetry.addData("Right Vel", shooterRight.getVelocity());
            telemetry.update();
        }
    }
}