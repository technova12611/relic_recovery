package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Arcade Teleop", group = "Test")
public class ArcadeTeleop extends OpMode {
    private MecanumRobot robot;
    private Controller g1;

    @Override
    public void init() {
        robot = new MecanumRobot(null, hardwareMap, telemetry, null);
        g1 = new Controller(gamepad1);
    }

    @Override
    public void init_loop() {
        telemetry.update();
    }

    @Override
    public void start() {
        robot.onStart();
        robot.resetHeading();
    }

    @Override
    public void loop() {
        g1.update();

        if (g1.A() && g1.B()) {
            robot.resetHeading();
        }

        double lx = g1.left_stick_x, ly = - g1.left_stick_y;
        double v = Math.sqrt(lx * lx + ly * ly);
        double theta = Math.atan2(lx, ly);
        double current = Math.toRadians(robot.getRawHeadingDegrees());
        robot.drive(theta + current, v, g1.right_stick_x);
    }

}
