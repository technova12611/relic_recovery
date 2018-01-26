package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Test Glyph Pickup Reverse", group = "Test")
@Disabled
public class TestGlyphCollector3 extends LinearOpMode {

    TileRunnerRobot robot;

    boolean forward = false;
    boolean backward = false;
    int previousEncoder = 0;

    MovingAverage intakeLeftEncoder = new MovingAverage(5);
    MovingAverage intakeRightEncoder = new MovingAverage(5);

    ElapsedTime intakeStuckTimer = new ElapsedTime();
    boolean stuckDetected = false;

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
            if(Math.abs(power) > 0.2) {
                forward = false;
                backward = false;
            }

            if(gamepad1.x || forward) {
                power = 0.30f;
                forward = true;
                backward = false;
                stuckDetected = false;
            }

            if(gamepad1.y || backward) {
                power = -0.30f;
                forward = false;
                backward = true;
                stuckDetected = false;
            }

            if(gamepad1.a) {
                power = 0.0f;
                forward = false;
                backward = false;
                stuckDetected = false;
            }
            
            int leftPosition = robot.intakeLeft.getCurrentPosition();
            double avgLeftPosition = intakeLeftEncoder.next(leftPosition);
            if(Math.abs(power) > 0.0 && !stuckDetected) {
                if( Math.abs(avgLeftPosition) < Math.abs(leftPosition) + 10) {
                    stuckDetected = true;
                    intakeStuckTimer.reset();
                    //power = -power;
                }
            }

            if(stuckDetected && intakeStuckTimer.time(TimeUnit.SECONDS) > 1.0 ) {
                stuckDetected = false;
                //power = -power;
            }

            if(stuckDetected) {
                robot.intakeLeft.setPower(power);
                robot.intakeRight.setPower(-power);
            } else {
                robot.intakeLeft.setPower(power);
                robot.intakeRight.setPower(power);
            }

            if(gamepad1.right_stick_x > 0.9) {
                robot.dumpGlyphsFromTray();
            } else if(gamepad1.right_stick_x > 0.5) {
                robot.raiseGlyphTrayup1();
            } else if(gamepad1.right_stick_x < -0.9) {
                robot.resetGlyphTray();
            }
            else if(gamepad1.right_stick_x < -0.5) {
                robot.raiseGlyphTrayup2();
            }

            telemetry.addData("motor power:", String.format("%.2f", power) + " | encoder: " + robot.intakeLeft.getCurrentPosition()
            + " | avg encoder: " + String.format("%.2f",avgLeftPosition));

            telemetry.addData("Stuck Detected: ", stuckDetected + " | elapsedTime: " +
                    String.format("%.1f",intakeStuckTimer.seconds()));

            telemetry.update();
        }
    }
}
