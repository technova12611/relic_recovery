package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBase.State.END;
import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBlue2.State.RESET_GLYPH_TRAY;

@Autonomous(name = "Auto BLUE #2 (Strafe)", group = "Competition")
public class RelicRecoveryAutoTileRunnerBlue2 extends RelicRecoveryAutoTileRunnerAbstract {
    public AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }

    // make new States for each autonomous strategy
    // like Auto 2, Auto 3 etc.
    //  the following is for Auto 2 (both BLUE and RED)
    //----------------------------------------------
    enum State implements AutoState {
        START,
        PUSH_JEWEL,
        FORWARD_3_FEET,
        TURN_TO_180_DEGREE,
        STRAFE_RIGHT,
        BACKWARD_3_INCHES,
        PLACE_GLYPH_INTO_CRYPTO,
        PICKUP_SECOND_GLYPH,
        RESET_GLYPH_TRAY,
        END;

        private static RelicRecoveryAutoTileRunnerBlue2.State[] vals = values();
        public RelicRecoveryAutoTileRunnerBlue2.State next()
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
        v_state = RelicRecoveryAutoTileRunnerBlue2.State.START;


        // 2. Run the state machine
        //  test, and more test
        //-------------------------------------------------------------------
        while (opModeIsActive() && v_state != RelicRecoveryAutoTileRunnerBlue2.State.END) {

            boolean detectVuMark = false;
            double motorSpeed = 0.25;

            logStateInfo(v_state, "Start");

            RelicRecoveryAutoTileRunnerBlue2.State step = (RelicRecoveryAutoTileRunnerBlue2.State)v_state;

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

                case FORWARD_3_FEET:
                    detectVuMark = true;
                    driveForwardInches(28.25, motorSpeed, 5.0);
                    gotoNextState();
                    break;

                case TURN_TO_180_DEGREE:
                    turn(175.0);
                    sleepInAuto(300);
                    turnToAngle(-176.0, 0.10);

                    gotoNextState();
                    break;

                case STRAFE_RIGHT:

                    // need more testing on each position
                    // may need to add range sensor to have better distance control
                    //-------------------------------------------------------------
                    switch (vuMark) {

                        // need to place glyph into RIGHT Crypto box
                        case RIGHT:
                            driveRightInches(23.5, motorSpeed,5.0);
                            break;

                        // need to place glyph into CENTER Crypto box
                        // -------------------------------------------------
                        case CENTER:
                            driveRightInches(13.5, motorSpeed, 4.0);
                            break;

                        // need to place glyph into LEFT Crypto box
                        // -------------------------------------------------
                        case LEFT:
                            driveRightInches(6.0, motorSpeed, 3.0);
                            break;

                        // Default is CENTER position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            driveRightInches(15.0, motorSpeed, 5.0);
                            break;
                    }

                    gotoNextState();
                    break;

                case BACKWARD_3_INCHES:
                    robot.extendDistanceSensorArmServo();
                    turnToAngle(-174.0, 0.10);
                    sleepInAuto(100);

                    driveBackwardInches(5.0, motorSpeed, 2.0);
                    gotoNextState();
                    break;

                case PLACE_GLYPH_INTO_CRYPTO:
                    placeGlyphIntoColumn(motorSpeed, false);
                    gotoNextState();

                    break;

                case PICKUP_SECOND_GLYPH:

                    if(getRuntime() > 23.0 || !pickupMoreGlyphs()) {
                        gotoState(RESET_GLYPH_TRAY);
                        break;
                    }

                    double fasterMotorSpeed = 0.40;
                    switch (vuMark) {

                        // need to place glyph into RIGHT Crypto box
                        case RIGHT:
                            driveRightInches(7.0,fasterMotorSpeed ,3.0);
                            break;

                        // need to place glyph into CENTER Crypto box
                        // -------------------------------------------------
                        case CENTER:
                            driveRightInches(14.0, fasterMotorSpeed, 4.0);
                            break;

                        // need to place glyph into LEFT Crypto box
                        // -------------------------------------------------
                        case LEFT:
                            driveRightInches(21.0, fasterMotorSpeed, 5.0);
                            break;

                        // Default is CENTER position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            driveRightInches(10.0, fasterMotorSpeed, 4.0);
                            break;
                    }

                    turn(-165.0);
                    //turnToAngle(-135.0, 0.10);

                    robot.resetForTeleOps();

                    // drive to glyph pit
                    driveForwardInches(33.0, 0.60, 5.0);
                    // turn on the intake wheels
                    robot.collectGlyph();
                    sleepInAuto(200);

                    // push forward a bit to collect
                    driveForwardInches(7.0, 0.25, 2.0);
                    sleepInAuto(300);

                    driveBackwardInches(37.5, 0.75, 5.0);

                    robot.pushGlyph();

                    int previousIntakeCount = robot.intakeRight.getCurrentPosition();
                    boolean glyphStucked = false;
                    sleepInAuto(300);
                    if(Math.abs(previousIntakeCount - robot.intakeRight.getCurrentPosition()) < 20) {
                        robot.reverseGlyph();
                        sleepInAuto(1500);
                        glyphStucked = true;
                    }

                    robot.holdPusher();

                    robot.stopIntake();

                    if(!dumpMoreGlyphs() || glyphStucked) {
                        gotoNextState();
                        break;
                    }

                    turn(-176.0);
                    robot.extendDistanceSensorArmServo();

                    if(vuMark == RelicRecoveryVuMark.RIGHT) {
                        driveLeftInches(10.0, motorSpeed, 3.0);
                    } else {
                        driveLeftInches(3.0, motorSpeed, 3.0);
                    }

                    driveBackwardInchesToColumn(8.0, 0.35, 3.0);

                    turn(-178.0);

                    if(getRuntime() < 26.0) {
                        // push glyph again
                        robot.pushGlyph();
                        sleepInAuto(300);

                        placeGlyphIntoColumn(0.35, false);
                    }

                    gotoNextState();
                    break;

                case RESET_GLYPH_TRAY:
                    // move the glyph lift back to zero position
                    robot.initServosForTeleOps();
                    gotoNextState();
                    break;

                case END:
                    Default:
                        gotoState(RelicRecoveryAutoTileRunnerBlue2.State.END);
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
    //-------------------------------------------------------
}
