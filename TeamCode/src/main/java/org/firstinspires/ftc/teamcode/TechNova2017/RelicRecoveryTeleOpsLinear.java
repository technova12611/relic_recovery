package org.firstinspires.ftc.teamcode.TechNova2017;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_ELBOW_INITIAL_POSITION;

@TeleOp(name = "Relic Recovery TeleOps (Linear)", group = "Competition")
public class RelicRecoveryTeleOpsLinear extends LinearOpMode {
    private MecanumRobot robot = null;
    private Controller g1, g2;
    private boolean debug_mode = false;

    private double relicElbowPosition = RELIC_ELBOW_INITIAL_POSITION;

    private int GLYPH_ARM_OPEN_POSITION_CLOSED   = 0;
    private int GLYPH_ARM_OPEN_POSITION_WIDE_OPEN = 3;

    private int glyphGrabberState = GLYPH_ARM_OPEN_POSITION_WIDE_OPEN;

    private boolean glyphLiftStopperClosed = false;

    @Override
    public void runOpMode() {

        robot = new MecanumRobot(this,hardwareMap, telemetry, null);

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
        }
    }

    private void gamepadLoop(Controller g) {

        // driving the robot
        DriveHelper.drive(g, robot, telemetry);

        // grab glyph (open and close servo)
        // ---------------------------------------
        if(g1.A() || g2.A()) {
            robot.openGlyphGripper();
            telemetry.addData("Glyph lift Position: ", robot.getGlyphLiftPosition());
            Log.i(this.getClass().getSimpleName(), "Glyph lift position at release: " + robot.getGlyphLiftPosition());
        }

        if(g1.B() || g2.B()) {
            glyphGrabberState = GLYPH_ARM_OPEN_POSITION_CLOSED;
            robot.closeGlyphGripper();
        }

        if((g1.A() && g1.leftBumper()) ||  (g2.A() && g2.leftBumper())) {
            robot.openGlyphGripperWider();
        }

        if((g1.A() && g1.rightBumper()) ||  (g2.A() && g2.rightBumper())) {
            robot.openGlyphGripperWider();
        }

        if(g2.X() ) {
            robot.openUpperGlyphGripper();
        }
        else if(g2.X() && g2.leftBumper()) {
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
        else {
            robot.moveGlyphLift(0.0);
        }

        if(Math.abs(g2.left_stick_y)>0) {
            robot.moveRelicSlider(-1.0*g2.left_stick_y);
        } else {
            robot.moveRelicSlider(0.0);
        }

        if(g2.dpadUp()) {
            robot.closeGlyphLiftStopper();
            glyphLiftStopperClosed = true;
            telemetry.addData("Glyph Lower Arm: ", "CLOSED");
        } else if(g2.dpadDown()) {
            robot.openGlyphLiftStopper();
            glyphLiftStopperClosed = true;
            telemetry.addData("Glyph Lower Arm: ", "OPEN");
        }

        if(glyphLiftStopperClosed) {
            robot.turnOnBlueLed();
        } else {
            robot.turnOffBlueLed();
        }

        if(g2.dpadLeft()) {
            robot.releaseRelic();
        } else if(g2.dpadRight()) {
            robot.grabRelic();
        }

        if(g2.right_stick_y <0) {
            relicElbowPosition += 0.03;
            robot.setRelicElbowPosition(relicElbowPosition);
            sleep(100);
        }

        if(g2.right_stick_y >0) {
            relicElbowPosition -= 0.03;
            robot.setRelicElbowPosition(relicElbowPosition);
            sleep(100);
        }

        if (g2.rightBumper() && g2.leftBumper()) {
            robot.releaseClaw();
        }

        telemetry.addData("Relic Elbow Position: ", String.format("%.1f",robot.getRelicElbowPosition()));

        telemetry.update();
    }
}
