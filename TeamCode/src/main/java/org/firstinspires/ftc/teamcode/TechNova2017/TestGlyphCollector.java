package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test Glyph Pickup", group = "Test")
@Disabled
public class TestGlyphCollector extends LinearOpMode {
    TileRunnerRobot robot;

    boolean forward = false;
    boolean backward = false;

    ElapsedTime pusherTimer = new ElapsedTime();
    double pusherPosition = 0.0;

    ElapsedTime flipperTimer = new ElapsedTime();
    double flipperPosition = 0.0;

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

            if (pusherTimer.milliseconds() > 20) {
                if (gamepad2.right_stick_y < 0) {
                    pusherPosition = Range.clip(pusherPosition + (gamepad2.right_stick_y < -0.8 ? 0.03 : 0.01), 0.01, 1.0);
                    robot.setGlyphPusherPosition(pusherPosition);
                    pusherTimer.reset();
                } else if (gamepad2.right_stick_y > 0) {
                    pusherPosition = Range.clip(pusherPosition - (gamepad2.right_stick_y > 0.8 ? 0.03 : 0.01), 0.01, 1.0);
                    robot.setGlyphPusherPosition(pusherPosition);
                    pusherTimer.reset();
                }
            }

            if (flipperTimer.milliseconds() > 20) {
                if (gamepad2.left_stick_y < 0) {
                    flipperPosition = Range.clip(flipperPosition + (gamepad2.left_stick_y < -0.8 ? 0.03 : 0.01), 0.01, 1.0);
                    robot.setGlyphFlipperPosition(flipperPosition);
                    flipperTimer.reset();
                } else if (gamepad2.left_stick_y > 0) {
                    flipperPosition = Range.clip(flipperPosition - (gamepad2.left_stick_y > 0.8 ? 0.03 : 0.01), 0.01, 1.0);
                    robot.setGlyphFlipperPosition(flipperPosition);
                    flipperTimer.reset();
                }
            }

            if(gamepad1.left_trigger > 0.10) {
                robot.moveGlyphLift(gamepad1.left_trigger);
            } else if(gamepad1.right_trigger > 0.10) {
                robot.moveGlyphLift(-gamepad1.right_trigger);
            } else {
                robot.moveGlyphLift(0.0);
            }

            telemetry.addData("Intake motor power:", String.format("%.2f", power));
            telemetry.addData("Pusher position:", String.format("%.2f", pusherPosition));
            telemetry.addData("Flipper position:", String.format("%.2f", flipperPosition));

            telemetry.update();
        }

    }
}
