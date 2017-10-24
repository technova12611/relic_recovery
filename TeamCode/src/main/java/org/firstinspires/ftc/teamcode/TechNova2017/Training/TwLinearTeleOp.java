package org.firstinspires.ftc.teamcode.TechNova2017.Training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Terry LinearTeleOp", group="Training TeleOp")
@Disabled
public class TwLinearTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            double left = -gamepad1.left_stick_y;
            double right = -gamepad1.right_stick_y;

            robot.tankDrive(left, right);
        }
    }
}
