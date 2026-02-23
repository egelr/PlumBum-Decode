package org.firstinspires.ftc.teamcode.subsystemTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Variables;

@TeleOp(name = "AnalogTesting")
public class AnalogTesting extends LinearOpMode {

    // Servos
    private Servo kickerTopServo, kickerBottomServo;
    private Servo sorterLeftServo, sorterRightServo;

    // TWO analog feedback inputs (configure these names in RC config!)
    private AnalogInput kickerAnalog;
    private AnalogInput sorterAnalog;

    @Override
    public void runOpMode() {

        // ---- Hardware map (names MUST match your Robot Configuration) ----
        kickerTopServo = hardwareMap.get(Servo.class, "kickerTopServo");
        kickerBottomServo = hardwareMap.get(Servo.class, "kickerBottomServo");

        sorterLeftServo  = hardwareMap.get(Servo.class, "sorterLeftServo");
        sorterRightServo = hardwareMap.get(Servo.class, "sorterRightServo");

        // Control Hub -> Analog Input -> name them exactly like this (or change here):
        kickerAnalog = hardwareMap.get(AnalogInput.class, "kickerAnalog");
        sorterAnalog = hardwareMap.get(AnalogInput.class, "sorterAnalog");

        waitForStart();

        while (opModeIsActive()) {

            // ---------------- KICKER TESTING ----------------
            //up
            if (gamepad1.triangle) {
                kickerTopServo.setPosition(0.85);
                kickerBottomServo.setPosition(0.85);
            }
            //down
            if (gamepad1.circle) {
                kickerTopServo.setPosition(0.98);
                kickerBottomServo.setPosition(0.98);
            }
            if (gamepad1.square) {
                kickerTopServo.setPosition(1.0);
                kickerBottomServo.setPosition(1.0);
            }

            // ---------------- SORTER TESTING (example controls) ----------------
//intake first
            if (gamepad1.dpad_down) {
                sorterLeftServo.setPosition(0.0);
                sorterRightServo.setPosition(0.0 + Variables.sorterOffset);
            }
            //intake second
            if (gamepad1.dpad_left) {
                sorterLeftServo.setPosition( 0.19);
                sorterRightServo.setPosition( 0.19 + Variables.sorterOffset);
            }//intake third
            if (gamepad1.dpad_right) {
                sorterLeftServo.setPosition(0.38);
                sorterRightServo.setPosition(0.38 + Variables.sorterOffset);
            }
            //first sorter shooting slot
            if (gamepad1.share) {
                sorterLeftServo.setPosition(0.27);
                sorterRightServo.setPosition(0.27 + Variables.sorterOffset);
            }
            //second sorter shooting slot
            if (gamepad1.options) {
                sorterLeftServo.setPosition(0.465);
                sorterRightServo.setPosition(0.465 + Variables.sorterOffset);
            }
            //third sorter shooting slot
            if (gamepad1.guide) {
                sorterLeftServo.setPosition(0.08);
                sorterRightServo.setPosition(0.08 + Variables.sorterOffset);
            }



            // ---------------- ANALOG READINGS ----------------
            // ---------------- ANALOG READINGS ----------------
            double kickerVoltage = kickerAnalog.getVoltage();
            double sorterVoltage = sorterAnalog.getVoltage();

            telemetry.addLine("=== ANALOG FEEDBACK ===");
            telemetry.addData("Kicker Voltage (V)", "%.4f", kickerVoltage);
            telemetry.addData("Sorter Voltage (V)", "%.4f", sorterVoltage);


            telemetry.addLine("=== SERVO POSITIONS (commanded) ===");
            telemetry.addData("Kicker Top", "%.4f", kickerTopServo.getPosition());
            telemetry.addData("Kicker Bottom", "%.4f", kickerBottomServo.getPosition());
            telemetry.addData("Sorter Left", "%.4f", sorterLeftServo.getPosition());
            telemetry.addData("Sorter Right", "%.4f", sorterRightServo.getPosition());

            telemetry.update();
        }
    }
}
