package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpMode")
public class OpMode extends LinearOpMode {

    int liftLastPosition;
    //Creating the variables for Motors
    private Motor fL, fR, bL, bR;
    private Motor intakeMotor;
    //Creating drive speed variable
    public double drive_speed = 1;

    private DcMotorEx shooterLeft, shooterRight;

    // Motor specs
    private static final int CPR = 28;             // encoder counts per rev (GoBILDA 6000RPM)
    private static final int MAX_RPM = 6000;
    private static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;  // ~2800

    // Start at 0% power
    private double targetVelocity = 0;
    private double targetVelocity1 = 0;

    private double x = 0.005;

    @Override
    public void runOpMode() throws InterruptedException {

        //Creating Drivetrain Motors and Setting their behaviour to "brake"
        fL = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        fR = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        bL = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        bR = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //Creating the Mecanum Drivetrain
        MecanumDrive drive = new MecanumDrive(fL, fR, bL, bR);

        //Creating subsystem Motors
        intakeMotor = new Motor(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_435);
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // PIDF coefficients tuned for 6000 RPM
        shooterLeft.setVelocityPIDFCoefficients(50, 0.0, 0.001, 11.7);
        shooterRight.setVelocityPIDFCoefficients(50, 0.0, 0.001, 11.7);

        //Creating Servos
        Servo transferOutputServo = hardwareMap.get(Servo.class, "transferOutputServo");
        Servo sorterLeftServo = hardwareMap.get(Servo.class, "sorterLeftServo");
        Servo sorterRightServo = hardwareMap.get(Servo.class, "sorterRightServo");


        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        while (!isStopRequested()) {


            //Drivetrain controls
            drive.driveRobotCentric(
                    -gamepad1.left_stick_x * drive_speed,
                    gamepad1.left_stick_y * drive_speed,
                    -gamepad1.right_stick_x * drive_speed,
                    false

            );
            //Drivetrain Motors speed Change controls
            if (gamepad1.right_trigger > 0.5) {
                drive_speed = 0.45;
            } else {
                drive_speed = 1;
            }
            if (gamepad1.triangle) {
                intakeMotor.set(1);
            }
            if (gamepad1.circle) {
                intakeMotor.set(0);
            }
            if(gamepad1.left_bumper){
                intakeMotor.set(-1);
            }
            if (gamepad1.square) {
                transferOutputServo.setPosition(0.08);
            }
            if(gamepad1.cross){
                transferOutputServo.setPosition(0.18);
            }

            if(gamepad1.cross){
                targetVelocity = MAX_TICKS_PER_SEC * 0.52;
            }
            /*if (gamepad1.dpad_up) {
                targetVelocity += 0.5;  // increase by 50 ticks/sec
            }
            if (gamepad1.dpad_down) {
                targetVelocity -= 0.5;  // decrease by 50 ticks/sec
            }*/
            if(gamepad1.dpad_up){
                sorterRightServo.setPosition(0.375 + x);
                sorterLeftServo.setPosition(0.375);
            }
            if(gamepad1.dpad_down){
                sorterRightServo.setPosition(0.76 + x);
                sorterLeftServo.setPosition(0.76);
            }

            if (gamepad1.guide) {
                targetVelocity = 0;
                sorterRightServo.setPosition(0 + x);
                sorterLeftServo.setPosition(0);
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