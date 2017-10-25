package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Relic Recovery TeleOps (Iterative)", group = "Mecanum")
@Disabled
public class RelicRecoveryTeleopIterative extends OpMode {
    private MecanumRobot robot = null;
    private Controller g1, g2;
    private boolean debug_mode = false;


    @Override
    public void init() {
        robot = new MecanumRobot(hardwareMap, telemetry, null);

        g1 = new Controller(gamepad1);
        g2 = new Controller(gamepad2);
    }

    @Override
    public void init_loop() {
        g1.update();
        if (g1.AOnce()) {
            debug_mode = ! debug_mode;
        }
        telemetry.addData("Debug? (a)", debug_mode ? "on" : "off");
        telemetry.addData("Ready?", "YES.");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.onStart();
    }

    @Override
    public void stop() {
        robot.onStop();
    }

    private void g1Loop(Controller g) {

        // driving the robot
        DriveHelper.drive(g, robot, telemetry);

        // grab glyph (open and close servo)
        // ---------------------------------------
        if(g1.A() || g2.A()) {
            robot.openGlyphGripper();
        }

        if(g1.B() || g2.B()) {
            robot.closeGlyphGripper();
        }

        if((g1.A() && g1.leftBumper()) ||  (g2.A() && g2.leftBumper())) {
            robot.openGlyphGripperWider();
        }

        // move the glyph lift up and down
        //---------------------------------------------------------------------
        if(g1.left_trigger >0.0) {
            robot.moveGlyphLift(g1.left_trigger);
        } else if(g1.right_trigger >0.0){
            robot.moveGlyphLift(-g1.right_trigger);
        } else if(g2.left_trigger > 0.0) {
            robot.moveGlyphLift(g2.left_trigger);
        } else if(g2.right_trigger > 0.0){
            robot.moveGlyphLift(-g2.right_trigger);
        } else if(g2.leftBumper() && g2.right_trigger > 0.25) {
            robot.forceMoveGlyphLiftDown(-g2.right_trigger);
        }
        else {
            robot.moveGlyphLift(0.0);
        }
    }

    @Override
    public void loop() {
        g1.update();
        g2.update();
        //robot.loop();

        g1Loop(g1);

        if (debug_mode) {
            robot.updateSensorTelemetry();
            telemetry.update();
        }
    }
}
