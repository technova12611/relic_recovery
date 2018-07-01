package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Auto RED #2 (Turn)", group = "Competition")
@Disabled
public class RelicRecoveryAutoTileRunnerRed5 extends RelicRecoveryAutoTileRunnerAbstract {
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
        TURN_LEFT_TO_90,
        BACKWARD_1_FEET,
        TURN_RIGHT_TO_0,
        BACKWARD_7_INCHES,
        PLACE_GLYPH_INTO_CRYPTO,
        RESET_GLYPH_TRAY,
        END;

        private static RelicRecoveryAutoTileRunnerRed5.State[] vals = values();
        public RelicRecoveryAutoTileRunnerRed5.State next()
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
        v_state = RelicRecoveryAutoTileRunnerRed5.State.START;


        // 2. Run the state machine
        //  test, and more test
        //-------------------------------------------------------------------
        while (opModeIsActive() && v_state != RelicRecoveryAutoTileRunnerRed5.State.END) {

            boolean detectVuMark = false;
            double motorSpeed = 0.25;

            logStateInfo(v_state, "Start");

            RelicRecoveryAutoTileRunnerRed5.State step = (RelicRecoveryAutoTileRunnerRed5.State)v_state;

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
                    driveBackwardInches(28.0, motorSpeed, 5.0);
                    gotoNextState();
                    break;

                case TURN_LEFT_TO_90:

                    turn(86.0);
                    sleepInAuto(500);
                    gotoNextState();
                    break;

                case BACKWARD_1_FEET:
                    // make sure it's at 90 degree
                    turnToAngle(90.0, 0.08);

                    // need more testing on each position
                    // may need to add range sensor to have better distance control
                    //-------------------------------------------------------------
                    switch (vuMark) {

                        // need to place glyph into RIGHT Crypto box
                        case RIGHT:
                            driveBackwardInches(6.0, motorSpeed, 5.0);
                            break;

                        // need to place glyph into CENTER Crypto box
                        // -------------------------------------------------
                        case CENTER:
                            driveBackwardInches(12.5, motorSpeed, 5.0);
                            break;

                        // need to place glyph into LEFT Crypto box
                        // -------------------------------------------------
                        case LEFT:
                            driveBackwardInches(19.5, motorSpeed, 5.0);
                            break;

                        // Default is RIGHT position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            driveBackwardInches(12.5, motorSpeed, 5.0);
                            break;
                    }

                    gotoNextState();
                    break;

                case TURN_RIGHT_TO_0:
                    turn(2.0);
                    sleepInAuto(500);
                    gotoNextState();
                    break;

                case BACKWARD_7_INCHES:
                    robot.extendDistanceSensorArmServo();
                    driveBackwardInches(2.5, motorSpeed, 5.0);
                    gotoNextState();
                    break;

                case PLACE_GLYPH_INTO_CRYPTO:
                    placeGlyphIntoColumn(motorSpeed);

                    if(vuMark == RelicRecoveryVuMark.CENTER) {
                        driveForwardInches(6.0, 0.35,2.0);
                    } else if (vuMark == RelicRecoveryVuMark.RIGHT){
                        driveForwardInches(12.0, 0.35, 2.0);
                    }
                    turn(-35.0);

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

    //------------------------------------
}
