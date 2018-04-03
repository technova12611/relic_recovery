package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@TeleOp(name = "Sensor Test", group = "Sensor")
public class SensorTest extends LinearOpMode {
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    ModernRoboticsI2cRangeSensor xRangeSensor;
    ModernRoboticsI2cRangeSensor yRangeSensor;

    AnalogInput rangeSensor;
    DistanceSensor glyphDistance;

    double servoPosition = 0.0;
    ElapsedTime servoTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        TileRunnerRobot robot = new TileRunnerRobot(this, hardwareMap, telemetry, AllianceColor.RED);

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "jewelColor");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "jewelColor");

        //glyphDistance = hardwareMap.get(DistanceSensor.class, "glyphColorDistance");

        VuMarkVision vuMarkVision = new VuMarkVision(hardwareMap, telemetry, false);

        // wait for the start button to be pressed.
        waitForStart();

        vuMarkVision.activate();

        RelicRecoveryVuMark vuMark = vuMarkVision.detect(telemetry);

        ElapsedTime timer2 = new ElapsedTime();

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // send the info back to driver station using telemetry function.
            telemetry.addData("(x1,x2): ", "(%.2f, %.2f)", robot.getX1Distance(), robot.getColDistance());
            telemetry.addData("IMU: ", "%.2f", robot.getHeadingAngle());

            if(robot.proximitySensor != null){
                telemetry.addData("Proximity: ", robot.columnDetected());
            }

            vuMark = vuMarkVision.detect(null, true);

            if(vuMark == RelicRecoveryVuMark.UNKNOWN) {
                robot.turnOffBlueLed();
            } else {
                robot.turnOnBlueLed();
            }

            String message = String.format("%s visible", vuMark);
            telemetry.addData("VuMark", message);

            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Alpha ", sensorColor.alpha());
            telemetry.addData("RGB ", sensorColor.argb());

            if(gamepad1.left_stick_y > 0.5) {
                robot.turnOnBlueLed();
            } else if(gamepad1.left_stick_y < -0.5) {
                robot.turnOffBlueLed();
            }

            if(gamepad1.a && !gamepad1.start) {
                robot.extendDistanceSensorArmServo();
            }

            if(gamepad1.b) {
                robot.resetDistanceSensorServoArm();
            }

            if(gamepad1.x) {
                timer2.reset();

                robot.pushGlyph();

                while(timer2.milliseconds() < 500.0) {
                    idle();
                }

                robot.holdPusher();
            }

            if(gamepad1.right_trigger > 0.5 && gamepad1.left_trigger > 0.5) {
                robot.stopIntake();
            }
            else if(gamepad1.right_trigger > 0.5) {
                robot.collectGlyph();
            } else if(gamepad1.left_trigger > 0.5) {
                robot.reverseGlyph();
            }

            if(gamepad2.a) {
                robot.openGlyphBlocker();
                robot.raiseGlyphTrayup2();
            }
            else if(gamepad2.b && !gamepad2.start) {
                robot.openGlyphBlocker();
                robot.raiseGlyphTrayup1();
            }
            else if(gamepad2.x) {
                robot.openGlyphBlocker();
                robot.dumpGlyphsFromTray();
                robot.holdPusher();
            }
            else if(gamepad2.left_bumper && gamepad2.y) {
                robot.resetGlyphTray();
                robot.holdPusher();
                //collectGlyph();
            } else if(gamepad2.y) {
                robot.openGlyphBlocker();
            }

            servoPosition = robot.distSensorServo.getPosition();
            if (gamepad1.right_stick_y < 0) {
                servoPosition = Range.clip(servoPosition + 0.05, 0.01, 1.0);
                robot.setServoPosition(robot.distSensorServo, servoPosition);
                servoTimer.reset();
            } else if (gamepad1.right_stick_y > 0) {
                servoPosition = Range.clip(servoPosition - 0.05, 0.01, 1.0);
                robot.setServoPosition(robot.distSensorServo, servoPosition);
                servoTimer.reset();
            }

            telemetry.addData("Servo Position: ", "%.1f", servoPosition);
            telemetry.update();
        }
    }
}
