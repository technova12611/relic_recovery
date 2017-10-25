package org.firstinspires.ftc.teamcode.TechNova2017;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Locale;

@TeleOp(name = "Sensor Test", group = "Sensor")
public class SensorTest extends LinearOpMode {
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    ModernRoboticsI2cRangeSensor xRangeSensor;
    ModernRoboticsI2cRangeSensor yRangeSensor;

    @Override
    public void runOpMode() {

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "jewelColor");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "jewelColor");
        xRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "x1Range");
        //yRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "x2Range");

        VuMarkVision vuMarkVision = new VuMarkVision(hardwareMap, telemetry);

        // wait for the start button to be pressed.
        waitForStart();

        vuMarkVision.activate();

        RelicRecoveryVuMark vuMark = vuMarkVision.detect(telemetry);

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // send the info back to driver station using telemetry function.
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("(x,y): ", "(%.2f, 0)",
                    xRangeSensor.getDistance(DistanceUnit.CM));
//            telemetry.addData("(x1,x2): ", "(%.2f, %.2f)",
//                    xRangeSensor.getDistance(DistanceUnit.CM), yRangeSensor.getDistance(DistanceUnit.CM));

            if(vuMark == RelicRecoveryVuMark.UNKNOWN) {
                vuMark = vuMarkVision.detect(null);
            }

            String message =String.format("%s visible", vuMark);
            telemetry.addData("VuMark", message);

            telemetry.update();

            sleep(100);
        }
    }
}
