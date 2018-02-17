package org.firstinspires.ftc.teamcode.TechNova2017;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_ELBOW_INITIAL_POSITION;

@TeleOp(name = "Tile Runner (New) TeleOps", group = "Competition")
public class RelicRecoveryTileRunnerTeleOpsLinear extends LinearOpMode {
    private TileRunnerRobot robot = null;
    private Controller g1, g2;
    private boolean debug_mode = false;

    private double relicElbowPosition = RELIC_ELBOW_INITIAL_POSITION;

    private ElapsedTime relicElbowTimer = new ElapsedTime();

    boolean relicClawLocked = false;
    boolean clawClosed = false;

    ElapsedTime intakeStuckTimer = new ElapsedTime();
    boolean stuckDetected = false;
    double previousRightIntakePosition = 0.0;

    boolean intakeForward = false;
    boolean intakeBackward = false;

    Boolean glyphLiftInAutoMode = null;
    ElapsedTime glyphLiftTimer = new ElapsedTime();
    int glyphLastPosition = 0;

    int numOfTrips = 0;

    ElapsedTime tripTimer = new ElapsedTime();

    ElapsedTime intakeSwitchTimer = new ElapsedTime();

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

        tripTimer.reset();

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
        // using x^3 function to slow down at the low range
        // ----------------------------------------------------------
        if (Math.abs(g2.left_stick_y) > 0.05 && !relicClawLocked) {
            robot.moveRelicSlider(-1.0 * Math.pow(g2.left_stick_y, 3.0));
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
            robot.closeIntakeWheels();
            robot.pushGlyph();
            //relicClawLocked = true;
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

        if(glyphLiftInAutoMode != null &&  glyphLiftInAutoMode) {
            if(robot.isGlyphLiftTargetReached()) {
                glyphLiftInAutoMode = null;
                robot.resetGlyphTray();
            }
        }
        else {
            robot.setGlyphLiftToRunEncoderMode();
        }

        if (g2.rightBumper() && g2.leftBumper()) {
            if(g2.rightBumper_long() && g2.leftBumper_long()) {
                robot.releaseClaw();
                robot.closeIntakeWheels();
                robot.pushGlyph();
                relicClawLocked = false;

                Log.i("Release Relic Claw:", "Recovery starts ...");
            }
        } else if((g2.leftBumper() || g2.rightBumper()) && g2.right_trigger > 0.2) {
            robot.moveGlyphLift(0);
            glyphLiftInAutoMode = Boolean.TRUE;
        }
        else if(g2.leftBumperOnce() && !g2.rightBumper()){
           robot.holdGlyph();
        }
        else if(g2.rightBumperOnce() || g2.rightBumper()){
            robot.pushGlyph();
        }

        // use gamepad 1 Y button to move the pusher all the way up
        //--------------------------------------------------------------
        if(g1.YOnce()) {
            if(robot.pusherStateClosed == null) {
                robot.pushGlyph();
            } else {
                robot.moveUpPusher();
            }
        }

        // use gamepad2 triggers to move the glyph slider lift
        // up/down
        //----------------------------------------------------
        else if(g2.left_trigger > 0.1) {
            robot.setGlyphLiftToRunEncoderMode();
            robot.moveGlyphLift(g2.left_trigger);
            glyphLiftInAutoMode = null;
        }
        else if(g2.right_trigger > 0.1) {
            robot.setGlyphLiftToRunEncoderMode();
            robot.moveGlyphLift(-g2.right_trigger);
            glyphLiftInAutoMode = null;
        }
        else if(glyphLiftInAutoMode == null || !glyphLiftInAutoMode){
            glyphLiftInAutoMode = Boolean.FALSE;
            robot.setGlyphLiftToRunEncoderMode();
            robot.stopGlyphLiftMotor();
        }

        // use gamepad 2 A/B/X/Y to move the glyph tray
        // also sync the intake and pusher
        //-----------------------------------------------

        if(g2.A()) {
            robot.openGlyphBlocker();
            robot.raiseGlyphTrayup2();
            stopIntake();
        }
        else if(g2.B() && !gamepad2.start) {
            robot.openGlyphBlocker();
            robot.raiseGlyphTrayup2();
            stopIntake();
        }
        else if(g2.X()) {
            robot.openGlyphBlocker();
            robot.dumpGlyphsFromTray();
            robot.holdGlyph();
            stopIntake();

            Log.i("Place Glyph:", "Trip #:" + (++numOfTrips) + " | " + String.format("%.1f", tripTimer.seconds()));
            tripTimer.reset();
        }
        else if(g2.leftBumper() && g2.Y()) {
            robot.resetGlyphTray();
            robot.holdGlyph();
            //collectGlyph();
        } else if(g2.YOnce()) {
            robot.openGlyphBlocker();
        }

        // use gamepad 1 triggers to control the intake wheels
        // use button A to stop the intake
        //-----------------------------------------------------
        if(g1.left_trigger > 0.5) {
            intakeForward = false;
            intakeBackward = true;
        } else if(g1.right_trigger > 0.5) {
            collectGlyph();
        } else if(g1.A()) {
            robot.stopIntake();
            stopIntake();
            stuckDetected = false;
        }

        // intake stuck detection
        // every 1 second, check the encoder against the previous measurement
        // if it's not increase enough, it's stuck
        //---------------------------------------------------------------------
        if(intakeForward && !stuckDetected
                && intakeStuckTimer.seconds() > 1.00
                && intakeSwitchTimer.seconds() > 2.2) {

            int rightPosition = robot.intakeRight.getCurrentPosition();

            // encoder should have changed by 80, otherwise it's stuck
            //---------------------------------------------------------------
            if (Math.abs(rightPosition - previousRightIntakePosition) < 150)
            {
                Log.i("Intake Detection:" , "Current: " + rightPosition
                                  + " | Previous: " + previousRightIntakePosition);
                stuckDetected = true;
                robot.isIntakeStuck = true;
            }

            previousRightIntakePosition = rightPosition;
            intakeStuckTimer.reset();
        }

        // intake stuck detected
        // set to run intake in reverse for 2.5 seconds
        //---------------------------------------------------------------
        if(stuckDetected) {
            intakeBackward = true;
            intakeForward = false;

            if(intakeStuckTimer.time(TimeUnit.SECONDS) > 2.5) {
                robot.isIntakeStuck = false;
                collectGlyph();
            }
        }

        // run intake wheels based on the setting
        //  intake or outtake
        //------------------------------------------
        if(intakeBackward) {
            robot.reverseGlyph();
        } else if(intakeForward) {
            robot.collectGlyph();
        } else {
            robot.stopIntake();
        }

        if(!intakeForward) {
            previousRightIntakePosition = robot.intakeRight.getCurrentPosition();
        }

        // driving the robot
        //------------------------------------------------------
        TileRunnerDriveHelper.drive(g1, robot, telemetry);

        telemetry.addData("Intake Counts:", + robot.intakeRight.getCurrentPosition() + " | "
                        + String.format("%.1f", intakeStuckTimer.seconds()));
        telemetry.addData("Intake Stucked:", robot.isIntakeStuck);
        telemetry.addData("relicElbowPosition: ", String.format("%.2f",relicElbowPosition));
        telemetry.addData("Relic Elbow Position: ", String.format("%.2f",robot.getRelicElbowPosition()));
        telemetry.addData("Glyph Lift Count: ", robot.getGlyphLiftPosition());
    }

    private void stopIntake() {
        intakeForward = false;
        intakeBackward = false;
    }

    private void collectGlyph() {
        intakeForward = true;
        intakeBackward = false;
        intakeStuckTimer.reset();
        intakeSwitchTimer.reset();
        stuckDetected = false;
    }
}
