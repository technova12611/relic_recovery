package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test Glyph Pickup", group = "Test")
public class TestGlyphCollector extends LinearOpMode {

    DcMotor glyphLeft, glyphRight;
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

        glyphLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glyphRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            glyphLeft.setPower(gamepad1.left_stick_y);
            glyphRight.setPower(gamepad1.left_stick_y);

            telemetry.update();
        }

    }
}
