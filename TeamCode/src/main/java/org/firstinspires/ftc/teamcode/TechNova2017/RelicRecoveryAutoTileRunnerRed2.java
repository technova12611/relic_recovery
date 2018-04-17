package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBlue2.State.RESET_GLYPH_TRAY;

@Autonomous(name = "Auto RED #2 More Glyph (Strafe)", group = "Competition")
@Disabled
public class RelicRecoveryAutoTileRunnerRed2 extends RelicRecoveryAutoTileRunnerAbstract {
    public AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }

    // make new States for each autonomous strategy
    // like Auto 2, Auto 3 etc.
    //  the following is for Auto 2 (both BLUE and RED)
    //----------------------------------------------
    enum State implements AutoState {
        START,
        PUSH_JEWEL,
        BACKWARD_3_FEET,
        STRAFE_LEFT,
        BACKWARD_7_INCHES,
        PLACE_GLYPH_INTO_CRYPTO,
        PREPARE_FOR_MORE_GLYPHS,
        PICKUP_MORE_GLYPHS,
        PLACE_MORE_GLYPHS,
        RESET_GLYPH_TRAY,
        END;

        private static RelicRecoveryAutoTileRunnerRed2.State[] vals = values();
        public RelicRecoveryAutoTileRunnerRed2.State next()
        {
            return vals[(this.ordinal()+1) % vals.length];
        }
        public String toString() {
            return this.ordinal() + " [" + super.toString() +"]";
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // common for all alliance to handle the initializatio of our mecamnum robot
        //------------------------------------------------------------------------
        initOpMode();

        // waiting for operator to press start button
        while(!isStarted()) {
            telemetry.addData("Distance (x1, x2): ", "(%.1f, %.1f)", robot.getX1Distance(), robot.getColDistance());
            telemetry.addData("IMU : ",
                    String.format("(%.1f)", robot.getHeadingAngle()));
            telemetry.addData("Touch", robot.isColumnTouched());
            telemetry.addData("Column", robot.columnDetected());
            telemetry.update();

            sleep(100);
        }

        this.resetStartTime();

        //-----------------------------------------------------------
        //   Auto Starts Here
        //-----------------------------------------------------------

        // 1. set drive train and glyph motors in the right RUN_MODE
        //------------------------------------------------------------
        robot.onStart();

        robot.logInfo(telemetry, "Robot:", String.format("Robot starting %.1f", timer.milliseconds()));
        telemetry.update();

        // starts the state machine
        //---------------------------------
        v_state = RelicRecoveryAutoTileRunnerRed2.State.START;


        // 2. Run the state machine
        //  test, and more test
        //-------------------------------------------------------------------
        while (opModeIsActive() && v_state != RelicRecoveryAutoTileRunnerRed2.State.END) {

            boolean detectVuMark = false;
            double motorSpeed = 0.25;

            logStateInfo(v_state, "Start");

            RelicRecoveryAutoTileRunnerRed2.State step = (RelicRecoveryAutoTileRunnerRed2.State)v_state;

            switch (step) {
                case START:
                    detectVuMark = true;
                    gotoNextState();
                    break;

                case PUSH_JEWEL:

                    detectVuMark = true;

                    // even if the pusher fails to initialize
                    // we still want to perform the glyph
                    if(pusher == null) {
                        gotoNextState();
                        break;
                    }

                    pusher.performTasks();
                    gotoNextState();
                    break;

                case BACKWARD_3_FEET:
                    detectVuMark = true;
                    driveBackwardInches(27.5, motorSpeed, 5.0);
                    gotoNextState();
                    break;

                case STRAFE_LEFT:

                    switch (vuMark) {

                        // need to place glyph into RIGHT Crypto box
                        case RIGHT:
                            driveLeftInches(6.5, motorSpeed, 2.0);
                            break;

                        // need to place glyph into CENTER Crypto box
                        // -------------------------------------------------
                        case CENTER:
                            driveLeftInches(15.0, motorSpeed, 5.0);
                            break;

                        // need to place glyph into LEFT Crypto box
                        // -------------------------------------------------
                        case LEFT:
                            driveLeftInches(23.0, motorSpeed,5.0);
                            break;

                        // Default is CENTER position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            driveLeftInches(15.5, motorSpeed,5.0);
                            break;
                    }

                    turnToAngle(0.0, 0.10);

                    robot.extendDistanceSensorArmServo();

                    gotoNextState();
                    break;

                case BACKWARD_7_INCHES:

                    driveBackwardInchesToColumn(3.0, motorSpeed, 5.0);
                    gotoNextState();
                    break;

                case PLACE_GLYPH_INTO_CRYPTO:
                    placeGlyphIntoColumn(motorSpeed, false);
                    driveForwardInches(1.5, motorSpeed, 1.5);

                    gotoNextState();
                    break;

                case PREPARE_FOR_MORE_GLYPHS:
                    if(getRuntime() > 23.0 || !pickupMoreGlyphs()) {
                        gotoState(RESET_GLYPH_TRAY);
                        break;
                    }

                    switch (vuMark) {

                        // need to place glyph into RIGHT Crypto box
                        case RIGHT:
                            driveLeftInches(27.5, motorSpeed,5.0);
                            break;

                        // need to place glyph into CENTER Crypto box
                        // -------------------------------------------------
                        case CENTER:
                            driveLeftInches(19.0, motorSpeed, 4.0);
                            break;

                        // need to place glyph into LEFT Crypto box
                        // -------------------------------------------------
                        case LEFT:
                            driveLeftInches(9.0, motorSpeed, 3.0);
                            break;

                        // Default is CENTER position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            driveLeftInches(19.0, motorSpeed, 3.0);
                            break;
                    }

                    turn(-13.5);

                    robot.resetForTeleOps();

                    // turn on the intake wheels
                    robot.collectGlyph();

                    gotoNextState();
                    break;

                case PICKUP_MORE_GLYPHS:

                    // drive to glyph pit
                    driveForwardInches(33.5, 0.50, 5.0);
                    sleepInAuto(100);

                    //turn(0.0);
                    // push forward a bit to collect
                    driveForwardInches(7.5, 0.35, 2.0);
                    sleepInAuto(100);

                    //driveBackwardInches(7.0, 0.5, 2.0);

                    //turn(-15.0);

                    driveBackwardInches(34.5, 0.50, 5.0);

                    robot.pushGlyph();

                    //driveRightInches(7.0, motorSpeed,3.0);

                    boolean glyphStucked = isGlyphStucked();

                    if(glyphStucked) {
                        glyphStucked = isGlyphStucked();
                    }

                    robot.stopIntake();

                    if(!dumpMoreGlyphs() || glyphStucked) {
                        gotoNextState();
                        break;
                    }

                    turn(0.0);

                    gotoNextState();
                    break;

                case PLACE_MORE_GLYPHS:

                    robot.closeIntakeWheels();

                    if(vuMark == RelicRecoveryVuMark.LEFT) {
                        driveRightInches(17.0, 0.35, 2.0);
                    } else {
                        driveRightInches(7.5, 0.35, 2.0);
                    }

                    robot.extendDistanceSensorArmServo();

                    turn(1.0);

                    driveBackwardInchesToColumn(10.0, 0.25, 2.0);

                    if(getRuntime() < 27.0) {
                        // push glyph again
                        robot.pushGlyph();
                        sleepInAuto(300);

                        placeGlyphIntoColumn(0.35, false);
                    }

                    robot.holdPusher();

                    driveForwardInches(3.0, 0.35, 2.0);

                    turnToAngle(-15.0, 0.25);

                    gotoNextState();
                    break;

                case RESET_GLYPH_TRAY:
                    // move the glyph lift back to zero position
                    robot.resetForTeleOps();
                    gotoNextState();
                    break;

                case END:
                    Default:
                    break;
            }

            // if vuMark is not visible, keep trying
            if(detectVuMark && vuMark == RelicRecoveryVuMark.UNKNOWN) {
                vuMark = vuMarkVision.detect(telemetry);
            }

            if(vuMark != RelicRecoveryVuMark.UNKNOWN) {
                robot.turnOnBlueLed();
                vuMarkVision.setFlashTorchMode(false);
            }
        }

        robot.onStop();
    }

    protected boolean pickupMoreGlyphs() {
        return true;
    }

    protected boolean dumpMoreGlyphs() { return false; }

    //------------------------------------
}
