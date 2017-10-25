package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Relic Recovery TeleOps (Linear)", group = "Competition")
public class RelicRecoveryTeleOpsLinear extends LinearOpMode {
    private MecanumRobot robot = null;
    private Controller g1, g2;
    private boolean debug_mode = false;

    @Override
    public void runOpMode() {

        robot = new MecanumRobot(hardwareMap, telemetry, null);

        g1 = new Controller(gamepad1);
        g2 = new Controller(gamepad2);

        g1.update();
        if (g1.AOnce()) {
            debug_mode = ! debug_mode;
        }

        telemetry.addData("Robot is Ready.", "Press START ..........");
        telemetry.update();

        // wait for the start button to be pressed.
        waitForStart();

        robot.onStart();

        // loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            g1.update();
            g2.update();
            //robot.loop();

            gamepadLoop(g1);

            if (debug_mode) {
                robot.updateSensorTelemetry();
                telemetry.update();
            }

            // check the status of the x button on gamepad1.
        }
    }

    private void gamepadLoop(Controller g) {

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

        if(g2.XOnce()) {
            robot.openUpperGlyphGripper();
        }
        else if(g2.X()) {
            robot.openUpperGlyphGripperWider();
        }
        else if(g2.Y()) {
            robot.closeUpperGlyphGripper();
        }

        // move the glyph lift up and down
        //---------------------------------------------------------------------
        if(g1.left_trigger >0.0) {
            robot.moveGlyphLift(g1.left_trigger);
        }
        else if(g1.right_trigger >0.0){
            robot.moveGlyphLift(-g1.right_trigger);
        }
        else if(g2.left_trigger > 0.0) {
            robot.moveGlyphLift(g2.left_trigger);
        }
        else if(g2.right_trigger > 0.0){
            robot.moveGlyphLift(-g2.right_trigger);
        }
        else if(g2.leftBumper() && g2.right_trigger > 0.25) {
            robot.forceMoveGlyphLiftDown(-g2.right_trigger);
        }
        else {
            robot.moveGlyphLift(0.0);
        }
    }
}
