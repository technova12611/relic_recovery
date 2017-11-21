package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoStrategyBase.State.END;
import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoStrategyBase.State.FORWARD_3_FEET;
import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoStrategyBase.State.START;

public class RelicRecoveryAutoStrategyBase extends RelicRecoveryAutoAbstract {

     // make new States for each autonomous strategy
     // like Auto 2, Auto 3 etc.
    //  the following is for Auto 1 (both BLUE and RED)
    //----------------------------------------------
     enum State implements AutoState {
         START,
         PUSH_JEWEL,
         PICK_UP_GLYPH,
         FORWARD_3_FEET,
         TURN_TO_90_DEGREE,
         FORWARD_1_FEET,
         ALIGN_TO_CRYPTOBOX,
         PLACE_GLYPH_INTO_CRYPTO,
         RESET_GLYPH_LIFT,
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
            telemetry.addData("Distance (x1, x2): ", "(%.1f, %.1f)", robot.getX1Distance(), robot.getX2Distance());
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
            double motorSpeed = 0.4;

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

                case PICK_UP_GLYPH:

                    detectVuMark = true;

                    // close the grabber, and move up the lift by 1 or 2 inches
                    robot.pickupGlyphInAuto();

                    gotoState(FORWARD_3_FEET);
                    break;

                case FORWARD_3_FEET:

                    // need more testing on each position
                    // may need to add range sensor to have better distance control
                    //-------------------------------------------------------------
                    switch (vuMark) {

                        // need to place glyph into RIGHT Crypto box
                        case RIGHT:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveForwardInches(27.5, motorSpeed);
                            }
                            // if this is BLUE Alliance
                            else {
                                driveBackwardInches(43.0, motorSpeed);
                            }
                            break;
                        // need to place glyph into CENTER Crypto box
                        // -------------------------------------------------
                        case CENTER:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveForwardInches(36.0, motorSpeed);
                            }
                            // if this is BLUE Alliance
                            else {
                                driveBackwardInches(36.5,motorSpeed);
                              }
                            break;

                        // need to place glyph into LEFT Crypto box
                        // -------------------------------------------------
                        case LEFT:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveForwardInches(44.0, motorSpeed);
                            }
                            // if this is BLUE Alliance
                            else {
                                driveBackwardInches(27.5, motorSpeed);
                            }
                            break;

                        // Default is CENTER position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveForwardInches(36.0, motorSpeed);
                            }
                            // if this is BLUE Alliance
                            else {
                                driveBackwardInches(36.5, motorSpeed);
                            }
                            break;
                    }

                    gotoNextState();
                    break;


                case TURN_TO_90_DEGREE:
                    // turn right to 90 degree
                    // need to figure out the turn direction for
                    // red and blue alliance
                    turn(-90.0);
                    gotoNextState();
                    break;

                case FORWARD_1_FEET:
                    driveForwardInches(4.0, motorSpeed);
                    gotoNextState();
                    break;

                case ALIGN_TO_CRYPTOBOX:

                    // need more testing on each position
                    // use the ultra_sonic sensor to align robot to the right cryto column
                    //----------------------------------------------------------------------

                    // within 1 cm is fine
                    // need more testing
                    double targetDistance = 0.0;
                    switch (vuMark) {

                        // need to place glyph into RIGHT Crypto box
                        case RIGHT:
                            targetDistance = getRightColumnTargetDistanceInCM();
                            break;

                        // need to place glyph into CENTER Crypto box
                        // -------------------------------------------------
                        case CENTER:
                            targetDistance = getCenterColumnTargetDistanceInCM();
                            break;

                        // need to place glyph into LEFT Crypto box
                        // -------------------------------------------------
                        case LEFT:
                            targetDistance = geLeftColumnTargetDistanceInCM();
                            break;

                        // Default is CENTER position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            break;
                    }

                    double avgXDistance = measureXDistance(500);
                    logInfo("X range:",vuMark + " | " + String.format("%.2f cm", getXDistance()));

                    if(targetDistance > 0.0) {
                        alignToCryptoBox(targetDistance);
                    }

                    gotoNextState();
                    break;

                case PLACE_GLYPH_INTO_CRYPTO:
                    logInfo(" --- Open Grabber --- ");
                    robot.openGlyphGripperMidWide();

                    while(opModeIsActive() && timer.time() < 300)  {
                        sleep(100);
                    }

                    // move forward to push the glyph into the box
                    //-------------------------------------------------
                    logInfo(" --- Drive forward to push --- ");
                    ElapsedTime watcher = new ElapsedTime();
                    driveForwardInches(4.0, motorSpeed);

                    logInfo(" Place Glyph into column (ms): " +
                            watcher.time(TimeUnit.MILLISECONDS) + " | " + vuMark
                            + " | " + String.format("%.2f cm", getXDistance()));

                    // move backward to separate robot from glyph
                    //----------------------------------------------
                    logInfo(" --- Drive backward to finish --- ");
                    driveBackwardInches(6.0, motorSpeed);

                    // fast turn to face the glyph pit, get ready for teleops
                    //------------------------------------------------
                    turnToAngle(90.0, 0.5);

                    gotoNextState();

                    break;

                case RESET_GLYPH_LIFT:
                    // move the glyph lift back to zero position
                    robot.resetGlyphLift();
                    robot.initServosForTeleOps();
                    gotoNextState();
                    break;

                case END:
                Default:
                    gotoState(END);
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


    protected void alignToCryptoBox(double targetPosition) throws InterruptedException
    {
        double xDistance = getXDistance();

        // range sensor failed, abort!
        if(xDistance == 0.0) {
            return;
        }

        double distanceThrehold = 1.0;

        double error = Range.clip(xDistance -targetPosition, 0.0, 3.0);
        ElapsedTime timer = new ElapsedTime();

        // try for 3 seconds only, to avoid oscilliation
        while(opModeIsActive() && Math.abs(error) > distanceThrehold && timer.time(TimeUnit.SECONDS) < 3) {
            xDistance = getXDistance();
            // turn right
            if(getAllianceColor() == AllianceColor.RED) {
                if (error > 0) {
                    driveRightInches(Math.abs(error/ 2.54), 0.25);
                } else {
                    driveLeftInches(Math.abs(error/ 2.54), 0.25);
                }
            }
            else {
                if (error > 0) {
                    driveLeftInches(Math.abs(error/ 2.54), 0.25);
                } else {
                    driveRightInches(Math.abs(error/ 2.54), 0.25);
                }
            }
        }
    }
    //------------------------------------
}