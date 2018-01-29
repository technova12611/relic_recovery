package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBase.State.END;
import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBase.State.PICKUP_SECOND_GLYPH;
import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBase.State.START;

public class RelicRecoveryAutoTileRunnerBase extends RelicRecoveryAutoTileRunnerAbstract {

     // make new States for each autonomous strategy
     // like Auto 2, Auto 3 etc.
    //  the following is for Auto 1 (both BLUE and RED)
    //----------------------------------------------
     enum State implements AutoState {
         START,
         PUSH_JEWEL,
         GET_OFF_STONE,
         TURN_TO_90_DEGREE,
         STRAFE_3_FEET,
         FORWARD_1_FEET,
         PLACE_GLYPH_INTO_CRYPTO,
         RESET_GLYPH_TRAY,
         PICKUP_SECOND_GLYPH,
         READY_FOR_TELEOPS,
         END;

         private static State[] vals = values();
         public State next()
         {
             return vals[(this.ordinal()+1) % vals.length];
         }
         public String toString() {
             return this.ordinal() + " [" + super.toString() +"]";
         }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // common for all alliance to handle the initialization of our mecamnum robot
        //------------------------------------------------------------------------
        initOpMode();

        // waiting for operator to press start button
        while(!isStarted()) {
            telemetry.addData("Distance (x1, x2): ",
                    String.format("(%.1f, %.1f)", robot.getX1Distance(), robot.getX2Distance()));
            telemetry.update();
            sleep(100);
        }

        //waitForStart();

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
        v_state = START;

        // 2. Run the state machine
        //  test, and more test
        //-------------------------------------------------------------------
        while (opModeIsActive() && v_state != END) {

            boolean detectVuMark = false;
            double motorSpeed = 0.25;

            logStateInfo(v_state, "Start");

            State step = (State)v_state;

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

                case GET_OFF_STONE:

                    switch (vuMark) {

                        // need to place glyph into RIGHT Crypto box
                        case RIGHT:
                            if(getAllianceColor() ==  AllianceColor.RED) {

                                driveBackwardInches(41.0, motorSpeed, 3.0);

                            }
                            // if this is BLUE Alliance
                            else {
                                driveForwardInches(28.0, motorSpeed, 3.0);

                            }
                            break;
                        // need to place glyph into CENTER Crypto box
                        // -------------------------------------------------
                        case CENTER:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveBackwardInches(36.0, motorSpeed, 5.0);

                            }
                            // if this is BLUE Alliance
                            else {
                                driveForwardInches(36.0, motorSpeed, 5.0);

                            }
                            break;

                        // need to place glyph into LEFT Crypto box
                        // -------------------------------------------------
                        case LEFT:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveBackwardInches(41.0, motorSpeed, 5.0);

                            }
                            // if this is BLUE Alliance
                            else {
                                driveForwardInches(28.0, motorSpeed, 5.0);

                            }
                            break;

                        // Default is CENTER position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveBackwardInches(36.0, motorSpeed, 5.0);
                            }
                            // if this is BLUE Alliance
                            else {
                                driveForwardInches(36.0, motorSpeed, 5.0);
                            }
                            break;
                    }

                    gotoNextState();
                    break;

                case TURN_TO_90_DEGREE:
                    // turn right to 90 degree
                    // need to figure out the turn direction for
                    // red and blue alliance
                    turn(-82.0);
                    sleepInAuto(1000);
                    gotoNextState();
                    break;

                case FORWARD_1_FEET:
                    turnToAngle(-88.0, 0.08);
                    driveBackwardInches(7.0, motorSpeed, 2.0);
                    gotoNextState();
                    break;

                case PLACE_GLYPH_INTO_CRYPTO:
                    turnToAngle(-90.0, 0.08);
                    placeGlyphIntoColumn(motorSpeed);
                    gotoNextState();

                    break;

                case RESET_GLYPH_TRAY:

                    // move the glyph lift back to zero position
                    robot.resetForTeleOps();

                    if(getRuntime() > 20.0 || !pickupMoreGlyphs()) {
                        gotoState(END);
                    } else {
                        if(getAllianceColor() == AllianceColor.RED) {
                            if (vuMark == RelicRecoveryVuMark.LEFT) {
                                turn(-80.0);
                            } else {
                                turn(-100.0);
                            }
                        } else {
                            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                                turn(-80.0);
                            } else {
                                turn(-100.0);
                            }
                        }
                        gotoNextState();
                    }
                    break;

                case PICKUP_SECOND_GLYPH:
                    // drive to glyph pit
                    driveForwardInches(26.0, 0.50, 5.0);

                    // turn on the intake wheels
                    robot.collectGlyph();

                    // push forward a bit to collect
                    driveForwardInches(4.0, 0.30, 5.0);
                    sleepInAuto(1500);

                    // move back and push the glyph into
                    driveBackwardInches(6.0, 0.60, 2.0);
                    robot.pushGlyph();

                    driveBackwardInches(30.0, 0.60, 5.0);

                    if(getRuntime() < 27.0) {
                        placeGlyphIntoColumn(0.5);
                    }

                    gotoNextState();

                    break;

                case READY_FOR_TELEOPS:

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
            }
        }

        robot.onStop();
    }

    protected boolean pickupMoreGlyphs() {
        return false;
    }

    //------------------------------------
}