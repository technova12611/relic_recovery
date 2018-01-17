package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Glyph Pickup", group = "Test")
public class TestGlyphCollector extends LinearOpMode {
    TileRunnerRobot robot;

    boolean forward = false;
    boolean backward = false;
    double previousFlipper = 0.0;
    double previousPusher = 0.0;

    @Override
    public void runOpMode() {

        robot = new TileRunnerRobot(this,hardwareMap,telemetry, null);

        telemetry.addData("Robot is Ready.", "Press START ..........");
        telemetry.update();

        // wait for the start button to be pressed.
        waitForStart();

        robot.onStart();

        while (opModeIsActive()) {

            float power = gamepad1.left_stick_y;
//            if(power == 0.0) {
//                power = -0.30f;
//            }

            if(gamepad1.x || forward) {
                power = 0.30f;
                forward = true;
                backward = false;
            }

            if(gamepad1.y || backward) {
                power = -0.30f;
                forward = false;
                backward = true;
            }

            if(gamepad1.a) {
                power = 0.0f;
                forward = false;
                backward = false;
            }

            robot.setIntakePower(power);

            if(gamepad1.right_stick_y > 0) {
                robot.pushGlyph();
            } else if(gamepad1.right_stick_y < 0) {
                robot.moveUpGlyphPusher();
            }

            telemetry.addData("motor power:", String.format("%.2f", power));

            telemetry.update();
        }

    }
}
