package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.Variables;


@TeleOp(name = "SensorTesting")
public class SensorTesting extends LinearOpMode {

    ColorSensor sensorColorFront;
    DistanceSensor sensorDistanceFront;

    ColorSensor sensorColorBack;
    DistanceSensor sensorDistanceBack;


    // Default value (in case neither condition matches)
    int colour = -1;
    @Override
    public void runOpMode() throws InterruptedException {


        sensorColorFront = hardwareMap.get(ColorSensor.class, "colourSensorFront");
        sensorDistanceFront = hardwareMap.get(DistanceSensor.class, "colourSensorFront");
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        sensorColorBack = hardwareMap.get(ColorSensor.class, "colourSensorBack");
        sensorDistanceBack = hardwareMap.get(DistanceSensor.class, "colourSensorBack");
        float hsvValuesB[] = {0F, 0F, 0F};
        final float valuesB[] = hsvValuesB;
        final double SCALE_FACTORB = 255;
        int relativeLayoutIdB = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayoutB = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        waitForStart();

        while (!isStopRequested()) {

            double gOverRFront = (double) sensorColorFront.green() / sensorColorFront.red();
            double gOverBFront = (double) sensorColorFront.green() / sensorColorFront.blue();


            double gOverRBack = (double) sensorColorBack.green() / sensorColorBack.red();
            double gOverBBack = (double) sensorColorBack.green() / sensorColorBack.blue();


            // Detect GREEN
            if ((sensorColorFront.green()  > sensorColorFront.red() && sensorColorFront.green() > sensorColorFront.blue() &&
                    gOverRFront > 1.7 &&
                    gOverBFront > 1.15)
                    || (sensorColorBack.green()  > sensorColorBack.red() && sensorColorBack.green() > sensorColorBack.blue() &&
                    gOverRBack > 1.9 &&
                    gOverBBack > 1.2)) {

                colour = 1;
            }
// Detect PURPLE
            else if ( (sensorColorFront.green() < sensorColorFront.blue() &&
                    gOverRFront < 1.4 &&
                    gOverBFront < 0.8) || (sensorColorBack.green() < sensorColorBack.blue() &&
                    gOverRBack < 1.4 &&
                    gOverBBack < 0.8)) {

                colour = 0;
            }
            else {
                colour = -1;
            };

            telemetry.addData("Distance F (cm)",
                    String.format(Locale.US, "%.02f",
                            sensorDistanceFront.getDistance(DistanceUnit.CM)));
            telemetry.addData("Distance B (cm)",
                    String.format(Locale.US, "%.02f",
                            sensorDistanceBack.getDistance(DistanceUnit.CM)));

            telemetry.addData("RedF ", sensorColorFront.red());
            telemetry.addData("GreenF", sensorColorFront.green());
            telemetry.addData("BlueF ", sensorColorFront.blue());
            telemetry.addData("Red B ", sensorColorBack.red());
            telemetry.addData("GreenB", sensorColorBack.green());
            telemetry.addData("BlueB ", sensorColorBack.blue());
            telemetry.addData("Green - 1; Purple - 0", colour);

            telemetry.update();

        }

    }
}