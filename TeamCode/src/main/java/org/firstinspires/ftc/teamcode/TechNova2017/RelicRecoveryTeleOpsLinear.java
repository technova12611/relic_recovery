package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_ELBOW_INITIAL_POSITION;

@TeleOp(name = "Relic Recovery TeleOps (Linear)", group = "Competition")
public class RelicRecoveryTeleOpsLinear extends LinearOpMode {
    private MecanumRobot robot = null;
    private Controller g1, g2;
    private boolean debug_mode = false;

    private double relicElbowPosition = RELIC_ELBOW_INITIAL_POSITION;

    private ElapsedTime relicElbowTimer = new ElapsedTime();

    boolean glyphLiftInAutoMode = false;

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
            gamepadLoop();
            telemetry.update();
        }
    }

    private void gamepadLoop()
    {
        // gamepad 1 A/B controls both grab glyph (open and close servo)
        // ---------------------------------------------------------
        if(g1.A() && (g1.leftBumper() || g1.rightBumper())) {
            robot.openGlyphGripperWider();
        }
        else if(g1.A()) {
            robot.openGlyphGripper();
        }
        else if(g1.B()) {
            robot.closeGlyphGripper();
        }

        // Gamepad 2 A/B controls lower glyph gripper
        //-----------------------------------------------
        if((g2.A() && (g2.leftBumper() || g2.rightBumper()))) {
            robot.openLowerGlyphGripperWide();
        }
        else if(g2.B()) {
            robot.closeLowerGlyphGripper();
        }
        else if(g2.A()) {
            robot.openLowerGlyphGripper();
        }

        // Gamepad 2 X/Y controls Upper glyph gripper
        //-----------------------------------------------
        if(g2.X() && (g2.leftBumper()|| g2.rightBumper())) {
            robot.openUpperGlyphGripperMidWide();
        }
        else if(g2.X() ) {
            robot.openUpperGlyphGripper();
        }
        else if(g2.Y() && (g2.leftBumper()|| g2.rightBumper())) {
            robot.alignStackedGlyphs();
        }
        else if(g2.Y()) {
            robot.closeUpperGlyphGripper();
        }

        // move the glyph lift up and down
        //---------------------------------------------------------------------
        if(glyphLiftInAutoMode && robot.isGlyphLiftTargetReached()) {
            glyphLiftInAutoMode = false;
        }
        else {
            robot.setGlyphLiftToRunEncoderMode();
        }

        if(g2.left_trigger > 0.5 && (g2.leftBumper()|| g2.rightBumper())) {
            robot.setGlyphLiftToPosition(2250);
            glyphLiftInAutoMode = true;
        }
        else if(g2.right_trigger > 0.5 && (g2.leftBumper()|| g2.rightBumper())) {
            robot.setGlyphLiftToPosition(0);
            glyphLiftInAutoMode = true;
        }
        else if(g1.left_trigger >0.0) {
            robot.setGlyphLiftToRunEncoderMode();
            robot.moveGlyphLift(g1.left_trigger);
            glyphLiftInAutoMode = false;
        }
        else if(g1.right_trigger >0.0){
            robot.setGlyphLiftToRunEncoderMode();
            robot.moveGlyphLift(-g1.right_trigger);
            glyphLiftInAutoMode = false;
        }
        else if(g2.left_trigger > 0.0) {
            robot.setGlyphLiftToRunEncoderMode();
            robot.moveGlyphLift(g2.left_trigger);
            glyphLiftInAutoMode = false;
        }
        else if(g2.right_trigger > 0.0){
            robot.setGlyphLiftToRunEncoderMode();
            robot.moveGlyphLift(-g2.right_trigger);
            glyphLiftInAutoMode = false;
        }
        else if(!glyphLiftInAutoMode){
            robot.setGlyphLiftToRunEncoderMode();
            robot.moveGlyphLift(0.0);
            glyphLiftInAutoMode = false;
        }

        // operator controller left joystick moves the relic slider
        // ----------------------------------------------------------
        if(Math.abs(g2.left_stick_y)>0) {
            robot.moveRelicSlider(-1.0*g2.left_stick_y);
        } else {
            robot.moveRelicSlider(0.0);
        }
        // switch to Relic grabbing/landing handling
        //----------------------------------------------
        if(g2.dpadUp() || g2.right_stick_x > 0.9) {
            robot.raiseRelicOverTheWall();
        }

        if(g2.dpadDown()) {
            robot.closeRelicClawHolder();
        }

        // operator controller DPAD left/right to grab/release RELIC
        //------------------------------------------------------------
        if(g2.dpadLeft()) {
            robot.releaseRelic();
        } else if(g2.dpadRight()) {
            robot.grabRelic();
        }

        // operator controller right joystick Y to move Relic Elbow up and down
        // move the arm gradually to avoid sudden stop
        //-----------------------------------------------------------------------
        if(relicElbowTimer.milliseconds() > 80){
            if(g2.right_stick_y <0){
                relicElbowPosition = Range.clip(relicElbowPosition + (g2.right_stick_y< -0.8?0.03:0.01), 0.10, 0.85);
                robot.setRelicElbowPosition(relicElbowPosition);
                relicElbowTimer.reset();
            }
            else if(g2.right_stick_y >0){
                relicElbowPosition = Range.clip(relicElbowPosition - (g2.right_stick_y> 0.8?0.03:0.01), 0.10, 0.85);
                robot.setRelicElbowPosition(relicElbowPosition);
                relicElbowTimer.reset();
            }
        }

        relicElbowPosition = robot.getRelicElbowPosition();

        // operator controller left+right bumper at once to release the RELIC claw
        // this is not reversible, use with caution
        // if accidentally release, need to move it up
        //-------------------------------------------------------------------------
        if (g2.rightBumper() && g2.leftBumper()) {
            robot.releaseClaw();
        }

        if(robot.isGlyphTouched()) {
            robot.turnOnBlueLed();
        } else if(robot.isBlueLedOn()){
            robot.turnOffBlueLed();
        }

        // driving the robot
        //------------------------------------------------------
        DriveHelper.drive(g1, robot, telemetry);

        telemetry.addData("relicElbowPosition: ", String.format("%.2f",relicElbowPosition));
        telemetry.addData("Relic Elbow Position: ", String.format("%.2f",robot.getRelicElbowPosition()));
        telemetry.addData("Glyph Lift Count: ", robot.getGlyphLiftPosition());
        telemetry.addData("Glyph Touched: ", robot.isGlyphTouched());
    }
}
