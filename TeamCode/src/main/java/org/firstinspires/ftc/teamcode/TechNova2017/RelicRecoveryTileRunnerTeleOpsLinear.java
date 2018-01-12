package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_ELBOW_INITIAL_POSITION;

@TeleOp(name = "Tile Runner (New) TeleOps ", group = "Competition")
public class RelicRecoveryTileRunnerTeleOpsLinear extends LinearOpMode {
    private TileRunnerRobot robot = null;
    private Controller g1, g2;
    private boolean debug_mode = false;

    private double relicElbowPosition = RELIC_ELBOW_INITIAL_POSITION;

    private ElapsedTime relicElbowTimer = new ElapsedTime();

    Boolean glyphLiftInAutoMode = null;
    ElapsedTime glyphLiftTimer = new ElapsedTime();
    int glyphLastPosition = 0;

    boolean relicClawLocked = true;
    boolean clawClosed = false;

    @Override
    public void runOpMode() {

        robot = new TileRunnerRobot(this,hardwareMap, telemetry, null);

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
            gamepadLoop();
            telemetry.update();
        }
    }

    private void gamepadLoop()
    {

        // operator controller left joystick moves the relic slider
        // ----------------------------------------------------------
        if (Math.abs(g2.left_stick_y) > 0 && !relicClawLocked) {
            robot.moveRelicSlider(-1.0 * g2.left_stick_y);
        } else {
            robot.moveRelicSlider(0.0);
        }

        // switch to Relic grabbing/landing handling
        //----------------------------------------------
        if(g2.dpadUp() || g2.right_stick_x > 0.9) {
            robot.raiseRelicOverTheWall();
        }

        if(g2.dpadDown() && !clawClosed) {
            robot.closeRelicClawHolder();
            relicClawLocked = true;
        }

        // operator controller DPAD left/right to grab/release RELIC
        //------------------------------------------------------------
        if(g2.dpadLeft()) {
            robot.releaseRelic();
            clawClosed = false;
        } else if(g2.dpadRight()) {
            robot.grabRelic();
            clawClosed = true;
        }

        // operator controller right joystick Y to move Relic Elbow up and down
        // move the arm gradually to avoid sudden stop
        // Allow the slide to move only if the relic claw is released.
        //-----------------------------------------------------------------------
        if(!relicClawLocked) {
            if (relicElbowTimer.milliseconds() > (g2.right_stick_y<0?35:20)) {
                if (g2.right_stick_y < 0) {
                    relicElbowPosition = Range.clip(relicElbowPosition + (g2.right_stick_y < -0.8 ? 0.03 : 0.01), 0.10, 0.85);
                    robot.setRelicElbowPosition(relicElbowPosition);
                    relicElbowTimer.reset();
                } else if (g2.right_stick_y > 0) {
                    relicElbowPosition = Range.clip(relicElbowPosition - (g2.right_stick_y > 0.8 ? 0.03 : 0.01), 0.10, 0.85);
                    robot.setRelicElbowPosition(relicElbowPosition);
                    relicElbowTimer.reset();
                }
            }
        }

        relicElbowPosition = robot.getRelicElbowPosition();

        // operator controller left+right bumper at once to release the RELIC claw
        // this is not reversible, use with caution
        // if accidentally release, need to move it up
        //-------------------------------------------------------------------------
        if (g2.rightBumper() && g2.leftBumper()) {
            robot.releaseClaw();
            relicClawLocked = false;
        }

        if(g2.left_trigger > 0.1) {
            robot.moveGlyphLift(g2.left_trigger);
        } else if(g2.right_trigger > 0.1) {
            robot.moveGlyphLift(-g2.right_trigger);
        }

        if(g2.Y()) {
            robot.dumpGlyphsFromTray();
        }

        if(g2.A()) {
            robot.raiseGlyphTrayup1();
        }

        if(g2.B()) {
            robot.raiseGlyphTrayup2();
        }

        if(g2.X()) {
            robot.resetGlyphTray();
        }

        if(g1.left_trigger > 0.5) {
            robot.collectGlyph(g1.left_trigger);
        } else if(g1.right_trigger > 0.1) {
            robot.reverseGlyph(-g1.right_trigger);
        }

        // driving the robot
        //------------------------------------------------------
        TileRunnerDriveHelper.drive(g1, robot, telemetry);

        telemetry.addData("relicElbowPosition: ", String.format("%.2f",relicElbowPosition));
        telemetry.addData("Relic Elbow Position: ", String.format("%.2f",robot.getRelicElbowPosition()));
        telemetry.addData("Glyph Lift Count: ", robot.getGlyphLiftPosition());
        telemetry.addData("Glyph Touched: ", robot.isGlyphTouched() + " | Distance: " + String.format("%.2f",robot.getGlyphDistance()));
    }

    private boolean isGlyphLiftManualModeAllowed() {
        if(this.glyphLiftInAutoMode == null || !this.glyphLiftInAutoMode ||
                (this.glyphLiftInAutoMode && glyphLiftTimer.seconds() > 1.0)) {
            return true;
        }
        return false;
    }
}
