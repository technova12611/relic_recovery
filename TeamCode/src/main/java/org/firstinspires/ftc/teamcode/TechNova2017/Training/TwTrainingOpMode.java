package org.firstinspires.ftc.teamcode.TechNova2017.Training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Terry TeleOp", group="Training TeleOp")
@Disabled
public class TwTrainingOpMode extends OpMode {

    protected Robot robot = null;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.init();
    }

    @Override
    public void loop() {
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;

        robot.tankDrive(left, right);
    }
}
