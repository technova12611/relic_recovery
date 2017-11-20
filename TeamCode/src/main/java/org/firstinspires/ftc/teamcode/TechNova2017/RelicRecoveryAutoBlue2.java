package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Blue 2", group = "Competition")
public class RelicRecoveryAutoBlue2 extends RelicRecoveryAutoAbstract {
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
        PICK_UP_GLYPH,
        BACKWARD_3_FEET,
        TURN_TO_180_DEGREE,
        RIGHT_1_FEET,
        FORWARD_1_FEET,
        ALIGN_TO_CRYPTOBOX,
        PLACE_GLYPH_INTO_CRYPTO,
        RESET_GLYPH_LIFT,
        END;

        private static RelicRecoveryAutoBlue2.State[] vals = values();
        public RelicRecoveryAutoBlue2.State next()
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
            telemetry.addData("Distance (x1, x2): ", "(%.1f, %.1f)", robot.getX1Distance(), robot.getX2Distance());
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
        v_state = RelicRecoveryAutoBlue2.State.START;


        // 2. Run the state machine
        //  test, and more test
        //-------------------------------------------------------------------
        while (opModeIsActive() && v_state != RelicRecoveryAutoBlue2.State.END) {

            boolean detectVuMark = false;
            double motorSpeed = 0.4;

            logStateInfo(v_state, "Start");

            RelicRecoveryAutoBlue2.State step = (RelicRecoveryAutoBlue2.State)v_state;

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

                    gotoState(RelicRecoveryAutoBlue2.State.BACKWARD_3_FEET);
                    break;

                case BACKWARD_3_FEET:
                    driveBackwardInches(25.0, motorSpeed);
                    gotoNextState();
                    break;

                case TURN_TO_180_DEGREE:
                    turn(180.0);
                    gotoNextState();
                    break;

                case RIGHT_1_FEET:

                    // need more testing on each position
                    // may need to add range sensor to have better distance control
                    //-------------------------------------------------------------
                    switch (vuMark) {

                        // need to place glyph into RIGHT Crypto box
                        case RIGHT:
                            driveRightInches(22.5, motorSpeed);
                            break;

                        // need to place glyph into CENTER Crypto box
                        // -------------------------------------------------
                        case CENTER:
                            driveRightInches(16.0, motorSpeed);
                            break;

                        // need to place glyph into LEFT Crypto box
                        // -------------------------------------------------
                        case LEFT:
                            driveRightInches(6.0, motorSpeed);
                            break;

                        // Default is CENTER position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            driveRightInches(16.0, motorSpeed);
                            break;
                    }

                    gotoNextState();
                    break;

                case FORWARD_1_FEET:
                    driveForwardInches(2.0, motorSpeed);
                    gotoNextState();
                    break;

                case ALIGN_TO_CRYPTOBOX:
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

                    logInfo(" Place Glyph into column (ms): " + watcher.time(TimeUnit.MILLISECONDS));

                    // move backward to separate robot from glyph
                    //----------------------------------------------
                    logInfo(" --- Drive backward to finish --- ");
                    driveBackwardInches(4.0, motorSpeed);

                    gotoNextState();

                    break;

                case RESET_GLYPH_LIFT:
                    // move the glyph lift back to zero position
                    robot.openGlyphGripperMidWide();
                    robot.resetGlyphLift();
                    gotoNextState();
                    break;

                case END:
                    Default:
                    gotoState(RelicRecoveryAutoBlue2.State.END);
                    break;
            }

            // if vuMark is not visible, keep trying
            if(detectVuMark && vuMark == RelicRecoveryVuMark.UNKNOWN) {
                vuMark = vuMarkVision.detect(telemetry);
            }
        }

        robot.onStop();
    }

    //------------------------------------
}
