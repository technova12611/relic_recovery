package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerTest.State.END;
import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerTest.State.START;

@Autonomous(name = "Auto Test TileRunner", group = "Competition")
public class RelicRecoveryAutoTileRunnerTest extends RelicRecoveryAutoTileRunnerAbstract {

     // make new States for each autonomous strategy
     // like Auto 2, Auto 3 etc.
    //  the following is for Auto 1 (both BLUE and RED)
    //----------------------------------------------
     enum State implements AutoState {
         START,
         GET_OFF_STONE,
         TURN_TO_90_DEGREE,
         STRAFE_3_FEET,
         FORWARD_1_FEET,
         PLACE_GLYPH_INTO_CRYPTO,
         RESET_GLYPH_TRAY,
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
            telemetry.addData("Distance (x1, x2): ", String.format("(%.1f, %.1f)", robot.getX1Distance(), robot.getX2Distance()));
            telemetry.update();
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

                case GET_OFF_STONE:
                    if(getAllianceColor() ==  AllianceColor.RED) {

                        driveBackwardInches(24.0, motorSpeed, 5.0);
                    }
                    // if this is BLUE Alliance
                    else {
                        driveForwardInches(24.0, motorSpeed, 5.0);
                    }

                    gotoNextState();
                    break;

                case TURN_TO_90_DEGREE:

                    // turn right to 90 degree
                    // need to figure out the turn direction for
                    // red and blue alliance
                    turnToAngle(-90.0, 0.15);

                    turnToAngle(-90.0, 0.10);
                    //gotoNextState();
                    gotoState(END);
                    break;

                case STRAFE_3_FEET:

                    // need more testing on each position
                    // may need to add range sensor to have better distance control
                    //-------------------------------------------------------------
                    double distanceToWall = measureXDistance(500)/2.54;
                    if(distanceToWall > 48.0 || distanceToWall < 36.0) {
                        distanceToWall = 39.0;
                    }
                    double distanceToNearColumnInInches = 42.5 - distanceToWall;

                    switch (vuMark) {

                        // need to place glyph into RIGHT Crypto box
                        case RIGHT:
                            if(getAllianceColor() ==  AllianceColor.RED) {

                                driveLeftInches(distanceToNearColumnInInches, motorSpeed, 3.0);
                            }
                            // if this is BLUE Alliance
                            else {
                                driveRightInches(18.0+distanceToNearColumnInInches, motorSpeed, 3.0);
                            }
                            break;
                        // need to place glyph into CENTER Crypto box
                        // -------------------------------------------------
                        case CENTER:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveLeftInches(8.5+distanceToNearColumnInInches, motorSpeed, 3.0);
                            }
                            // if this is BLUE Alliance
                            else {
                                driveRightInches(9.5+distanceToNearColumnInInches,motorSpeed, 3.0);
                            }
                            break;

                        // need to place glyph into LEFT Crypto box
                        // -------------------------------------------------
                        case LEFT:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveLeftInches(17.0+distanceToNearColumnInInches, motorSpeed, 3.0);
                            }
                            // if this is BLUE Alliance
                            else {
                                driveRightInches(1.0+distanceToNearColumnInInches, motorSpeed, 3.0);
                            }
                            break;

                        // Default is CENTER position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveLeftInches(8.5+distanceToNearColumnInInches, motorSpeed, 3.0);
                            }
                            // if this is BLUE Alliance
                            else {
                                driveRightInches(9.5+distanceToNearColumnInInches, motorSpeed, 3.0);
                            }
                            break;
                    }

                    gotoNextState();
                    break;

                case FORWARD_1_FEET:
                    driveForwardInches(4.0, motorSpeed, 2.0);
                    gotoNextState();
                    break;

                case PLACE_GLYPH_INTO_CRYPTO:

                    logInfo(" --- Dump Glyph into column and push --- ");
                    robot.dumpGlyphsFromTray();

                    while(opModeIsActive() && timer.time() < 1000)  {
                        sleep(100);
                    }

                    driveBackwardInches(2.0, 0.5, 2.0);

                    // move forward to push the glyph into the box
                    //-------------------------------------------------
                    logInfo(" --- Drive forward to push --- ");
                    ElapsedTime watcher = new ElapsedTime();
                    driveForwardInches(7.0, motorSpeed, 2.0);

                    logInfo(" Place Glyph into column (ms): " +
                            watcher.time(TimeUnit.MILLISECONDS) + " | " + vuMark
                            + " | " + String.format("%.2f cm", getXDistance()));

                    // move backward to separate robot from glyph
                    //----------------------------------------------
                    logInfo(" --- Drive backward to finish --- ");
                    driveBackwardInches(6.0, motorSpeed, 2.0);

                    gotoNextState();

                    break;

                case RESET_GLYPH_TRAY:
                    // move the glyph lift back to zero position
                    robot.resetForTeleOps();

                    // make sure robot is very close to 90 degree
                    turnToAngle(90.0, 0.2);

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

    public AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
    //------------------------------------
}