package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Test Glyph Pickup Encoder", group = "Test")
public class TestGlyphCollector2 extends LinearOpMode {

    DcMotor glyphLeft, glyphRight;
    boolean forward = false;
    boolean backward = false;
    int previousEncoder = 0;

    MovingAverage intakeLeftEncoder = new MovingAverage(5);
    MovingAverage intakeRightEncoder = new MovingAverage(5);

    ElapsedTime intakeStuckTimer = new ElapsedTime();
    boolean stuckDetected = false;

    @Override
    public void runOpMode() {

        glyphLeft = hardwareMap.dcMotor.get("glyph_left");
        glyphLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        glyphLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        glyphRight = hardwareMap.dcMotor.get("glyph_right");
        glyphRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Robot is Ready.", "Press START ..........");
        telemetry.update();

        // wait for the start button to be pressed.
        waitForStart();

        glyphLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glyphRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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


            int leftPosition = glyphLeft.getCurrentPosition();
            double avgLeftPosition = intakeLeftEncoder.next(leftPosition);
            if(Math.abs(power) > 0.0 && !stuckDetected) {
                if( Math.abs(avgLeftPosition) < Math.abs(leftPosition) + 10) {
                    stuckDetected = true;
                    intakeStuckTimer.reset();
                    power = -power;
                }
            }

            if(stuckDetected && intakeStuckTimer.time(TimeUnit.SECONDS) > 1.0 ) {
                stuckDetected = false;
                power = -power;
            }

            glyphLeft.setPower(power);
            glyphRight.setPower(power);

            telemetry.addData("motor power:", String.format("%.2f", power) + " | encoder: " + glyphLeft.getCurrentPosition()
            + " | avg encoder: " + String.format("%.2f",avgLeftPosition));

            telemetry.addData("Stuck Detected: ", stuckDetected + " | elapsedTime: " +
                    String.format("%.1f",intakeStuckTimer.seconds()));

            telemetry.update();
        }

    }
}
