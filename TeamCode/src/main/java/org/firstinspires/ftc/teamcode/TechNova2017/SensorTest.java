package org.firstinspires.ftc.teamcode.TechNova2017;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@TeleOp(name = "Sensor Test", group = "Sensor")
public class SensorTest extends LinearOpMode {
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    ModernRoboticsI2cRangeSensor xRangeSensor;
    ModernRoboticsI2cRangeSensor yRangeSensor;

    AnalogInput rangeSensor;
    DistanceSensor glyphDistance;

    @Override
    public void runOpMode() {

        MecanumRobot robot = new MecanumRobot(this, hardwareMap, telemetry, null);

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "jewelColor");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "jewelColor");
        try {
            xRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "x1Range");
            yRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "x2Range");
        }catch(Exception e) {
        }

        double maxVoltage = 0.0;
        try {
            rangeSensor = hardwareMap.analogInput.get("range");
            maxVoltage= rangeSensor.getMaxVoltage();
        }catch(Exception e) {
        }

        glyphDistance = hardwareMap.get(DistanceSensor.class, "glyphColorDistance");

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

            if(rangeSensor != null) {
                telemetry.addData("Max Vol: ", "%.2f", maxVoltage);
                telemetry.addData("Range Vol: ", "%.5f", rangeSensor.getVoltage());

                Log.i("Range Sensor", String.format("%.5f",rangeSensor.getVoltage() ));
                sleep(100);
            }


            telemetry.addData("(x1,x2): ", "(%.2f, %.2f)",
                    xRangeSensor!= null?xRangeSensor.getDistance(DistanceUnit.CM):0.0,
                    yRangeSensor != null?yRangeSensor.getDistance(DistanceUnit.CM):0.0);

            if(vuMark == RelicRecoveryVuMark.UNKNOWN) {
                vuMark = vuMarkVision.detect(null);
                robot.turnOffBlueLed();
            } else {
                robot.turnOnBlueLed();
            }

            String message =String.format("%s visible", vuMark);
            telemetry.addData("VuMark", message);

            telemetry.update();

            if(gamepad1.left_stick_y > 0.5) {
                robot.turnOnBlueLed();
            } else if(gamepad1.left_stick_y < -0.5) {
                robot.turnOffBlueLed();
            }

            if(gamepad2.right_stick_y <0) {
                robot.setGlyphLiftStopperPosition(Range.clip(robot.getGlyphLiftStopperPosition() + 0.02, 0.0, 1.00));
                sleep(50);
            }

            if(gamepad2.right_stick_y >0) {
                robot.setGlyphLiftStopperPosition(Range.clip(robot.getGlyphLiftStopperPosition() - 0.02, 0.0, 1.00));
                sleep(50);
            }

            telemetry.addData("Glyph Distance: ", String.format("%.2f",glyphDistance.getDistance(DistanceUnit.INCH)));
            telemetry.addData("Servo Position: ", String.format("%.2f",robot.getGlyphLiftStopperPosition()));
        }
    }
}
