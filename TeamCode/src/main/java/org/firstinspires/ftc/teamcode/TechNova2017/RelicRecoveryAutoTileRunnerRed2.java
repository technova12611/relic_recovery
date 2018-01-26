package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto RED #2", group = "Competition")
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
        FORWARD_3_FEET,
        LEFT_1_FEET,
        FORWARD_1_FEET,
        PLACE_GLYPH_INTO_CRYPTO,
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
            telemetry.addData("Distance (x1, x2): ", "(%.1f, %.1f)", robot.getX1Distance(), robot.getX2Distance());
            telemetry.update();
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
            double motorSpeed = 0.4;

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

                case FORWARD_3_FEET:
                    driveBackwardInches(25.0, motorSpeed);
                    gotoNextState();
                    break;

                case LEFT_1_FEET:

                    double distanceToWall = measureXDistance(500)/2.54;
                    if(distanceToWall > 28.0 || distanceToWall < 18.0) {
                        distanceToWall = 20.0;
                    }
                    double distanceToNearColumnInInches = 25.5 - distanceToWall;

                    // need more testing on each position
                    // may need to add range sensor to have better distance control
                    //-------------------------------------------------------------
                    switch (vuMark) {

                        // need to place glyph into RIGHT Crypto box
                        case RIGHT:
                            driveRightInches(distanceToNearColumnInInches, motorSpeed);
                            break;

                        // need to place glyph into CENTER Crypto box
                        // -------------------------------------------------
                        case CENTER:
                            driveRightInches(8.5+distanceToNearColumnInInches,motorSpeed);
                            break;

                        // need to place glyph into LEFT Crypto box
                        // -------------------------------------------------
                        case LEFT:
                            driveRightInches(17.5+distanceToNearColumnInInches, motorSpeed);
                            break;

                        // Default is CENTER position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            driveRightInches(8.5+distanceToNearColumnInInches,motorSpeed);
                            break;
                    }

                    gotoNextState();
                    break;

                case FORWARD_1_FEET:
                    driveBackwardInches(2.0, motorSpeed);
                    gotoNextState();
                    break;

                case PLACE_GLYPH_INTO_CRYPTO:

                    logInfo(" --- Open Grabber --- ");
                    robot.dumpGlyphsFromTray();

                    while(opModeIsActive() && timer.time() < 750)  {
                        sleep(100);
                    }

                    driveForwardInches(2.0, motorSpeed);

                    // move forward to push the glyph into the box
                    //-------------------------------------------------
                    logInfo(" --- Drive forward to push --- ");
                    ElapsedTime watcher = new ElapsedTime();
                    driveBackwardInches(6.0, motorSpeed);

                    logInfo(" Place Glyph into column (ms): " +
                            watcher.time(TimeUnit.MILLISECONDS) + " | " + vuMark
                            + " | " + String.format("%.2f cm", getXDistance()));

                    // move backward to separate robot from glyph
                    //----------------------------------------------
                    logInfo(" --- Drive backward to finish --- ");
                    driveForwardInches(7.0, motorSpeed);

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
            }
        }

        robot.onStop();
    }

    //------------------------------------
}
