package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTest.State.END;
import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTest.State.START;

@Autonomous(name = "Auto Test", group = "Test")
@Disabled
public class RelicRecoveryAutoTest extends RelicRecoveryAutoAbstract {

    // make new States for each autonomous strategy
    // like Auto 2, Auto 3 etc.
    //  the following is for Auto 1 (both BLUE and RED)
    //----------------------------------------------
    enum State implements AutoState {
        START,
        PICK_UP_GLYPH,
        FORWARD_2_FEET,
        SIDEWAY_LEFT_1_FEET,
        SIDEWAY_RIGHT_1_FEET,
        TURN_TO_90_DEGREE,
        SIDEWAY_RIGHT_2_INCHES,
        SIDEWAY_LEFT_1_INCHES,
        BACKWARD_2_FEET,
        END;

        private static RelicRecoveryAutoTest.State[] vals = values();
        public RelicRecoveryAutoTest.State next()
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
        waitForStart();

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

            double motorSpeed = 0.25;

            logStateInfo(v_state, "Start");

            State step = (State)v_state;

            switch (step) {
                case START:
                    gotoNextState();
                    break;

                case PICK_UP_GLYPH:

                    // close the grabber, and move up the lift by 1 or 2 inches
                   // robot.pickupGlyphInAuto();

                    gotoNextState();
                    break;

                case FORWARD_2_FEET:
                    driveBackwardInches(6.0, motorSpeed);
                    gotoNextState();
                    break;

                case SIDEWAY_LEFT_1_FEET:
                    driveLeftInches(12.0, motorSpeed);
                    gotoNextState();
                    break;

                case SIDEWAY_RIGHT_1_FEET:
                    driveRightInches(12.0, motorSpeed);
                    gotoState(END);
                    //gotoNextState();
                    break;

                case TURN_TO_90_DEGREE:
                    // turn right to 90 degree
                    // need to figure out the turn direction for
                    // red and blue alliance
                    turn(-90.0);
                    gotoNextState();
                    break;

                case SIDEWAY_RIGHT_2_INCHES:
                    driveRightInches(2.0, 0.25);
                    gotoNextState();
                    break;

                case SIDEWAY_LEFT_1_INCHES:
                    driveLeftInches(1.0, 0.25);
                    gotoNextState();
                    break;

                case BACKWARD_2_FEET:
                    driveBackwardInches(24.0, motorSpeed);
                    gotoNextState();

                case END:
                    robot.onStop();
                    break;
            }
        }

        robot.onStop();
    }

}
