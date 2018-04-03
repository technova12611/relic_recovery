package org.firstinspires.ftc.teamcode.TechNova2017;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBase.State.END;
import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBase.State.READY_FOR_TELEOPS;
import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBase.State.START;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.DISTANCE_SENSOR_UPRIGHT_POSITION;

public class RelicRecoveryAutoTileRunnerBase extends RelicRecoveryAutoTileRunnerAbstract {

     // make new States for each autonomous strategy
     // like Auto 2, Auto 3 etc.
    //  the following is for Auto 1 (both BLUE and RED)
    //----------------------------------------------
     enum State implements AutoState {
         START,
         PUSH_JEWEL,
         GET_OFF_STONE,
         ALIGN_TO_COLUMN,
         TURN_TO_90_DEGREE,
         PICKUP_ANOTHER_GLYPH,
         FORWARD_TO_CRYPTOBOX,
         PLACE_GLYPH_INTO_CRYPTO,
         PREPARE_FOR_MORE_GLYPHS,
         PICKUP_MORE_GLYPH,
         DRIVE_BACK_TO_CRYPTO,
         PLACE_MORE_GLYPHS_INTO_CRYPTO,
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
                    String.format("(%.1f, %.1f)", robot.getX1Distance(), robot.getColDistance()));
            telemetry.addData("Glyph Color : ",robot.getGlyphColorRGB() +"");
            telemetry.addData("IMU : ",
                    String.format("(%.1f)", robot.getHeadingAngle()));
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
            double motorSpeed = 0.275;
            double fasterMotorSpeed = 0.355;

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
                    detectVuMark = true;

                    if(getAllianceColor() ==  AllianceColor.RED) {
                        driveBackwardInches(24.5, motorSpeed, 5.0);
                    }
                    // if this is BLUE Alliance
                    else {
                        driveForwardInches(24.5, motorSpeed, 5.0);
                    }

                    gotoNextState();
                    break;

                case ALIGN_TO_COLUMN:
                    switch (vuMark) {

                        // need to place glyph into RIGHT Crypto box
                        case RIGHT:
                            if(getAllianceColor() ==  AllianceColor.RED) {

                                driveBackwardInches(3.0, motorSpeed, 2.0);

                            }
                            // if this is BLUE Alliance
                            else {
                                driveForwardInches(16.25, motorSpeed, 5.0);
                            }
                            break;
                        // need to place glyph into CENTER Crypto box
                        // -------------------------------------------------
                        case CENTER:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveBackwardInches(11.0, motorSpeed, 5.0);
                            }
                            // if this is BLUE Alliance
                            else {
                                driveForwardInches(8.0, motorSpeed, 5.0);
                            }
                            break;

                        // need to place glyph into LEFT Crypto box
                        // -------------------------------------------------
                        case LEFT:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveBackwardInches(18.5, motorSpeed, 5.0);
                            }
                            // if this is BLUE Alliance
                            else {
                                driveForwardInches(2.0, motorSpeed, 2.0);
                            }
                            break;

                        // Default is CENTER position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveBackwardInches(11.0, motorSpeed, 5.0);
                            }
                            // if this is BLUE Alliance
                            else {
                                driveForwardInches(8.0, motorSpeed, 5.0);
                            }
                            break;
                    }

                    gotoNextState();
                    break;

                case TURN_TO_90_DEGREE:

                    // turn right to 90 degree
                    // need to figure out the turn direction for
                    // red and blue alliance
                    if(getAllianceColor() ==AllianceColor.RED) {
                        turn(-85.5);
                    } else {
                        turn(-86.5);
                    }

                    //sleepInAuto(250);
                    //turn(-88.5);

                    gotoNextState();
                    break;

                case PICKUP_ANOTHER_GLYPH:

                    robot.resetForTeleOps();

                    robot.setServoPosition(robot.distSensorServo, DISTANCE_SENSOR_UPRIGHT_POSITION);

                    if(do4Glyphs()) {

                        robot.resetGlyphTray();
                        robot.collectGlyph();

                        driveForwardInches(12.0, 0.75, 2.0);
                        sleepInAuto(200);

                        driveForwardInches(6.0, 0.35, 2.0);
                        sleepInAuto(300);

                        robot.pushGlyph();

                        driveBackwardInches(18.0,0.5,3.0);

                        if(isGlyphStucked()) {
                            robot.pushGlyph();
                            isGlyphStucked();
                        }

                        turn(-88.0);
                        robot.stopIntake();
                    }

                    robot.extendDistanceSensorArmServo();

                    gotoNextState();
                    break;

                case FORWARD_TO_CRYPTOBOX:

                    if(do4Glyphs()) {
                        driveBackwardInchesToColumn(7.50, motorSpeed, 3.0);
                    } else {
                        driveBackwardInches(6.25, motorSpeed, 3.0);
                    }

                    gotoNextState();
                    break;

                case PLACE_GLYPH_INTO_CRYPTO:

                    placeGlyphIntoColumn(fasterMotorSpeed, false);
                    gotoNextState();

                    break;

                case PREPARE_FOR_MORE_GLYPHS:

                    if(getRuntime() > 22.0 || !pickupMoreGlyphs()) {
                        gotoState(READY_FOR_TELEOPS);
                        break;
                    } else {
                        //turn(-92.0);
                        double oneColumnDistance = 8.75;
                        if(getAllianceColor() == AllianceColor.RED) {
                            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                                driveLeftInches(oneColumnDistance+4.5, fasterMotorSpeed, 2.0);
                            } else {
                                driveRightInches(oneColumnDistance, fasterMotorSpeed, 2.0);
                            }
                        } else {
                            if (vuMark == RelicRecoveryVuMark.LEFT) {
                                driveRightInches(oneColumnDistance + 4.5, fasterMotorSpeed, 2.0);
                            } else {
                                driveLeftInches(oneColumnDistance, fasterMotorSpeed, 2.0);
                            }
                        }

                        turn(-89.0);
                        gotoNextState();
                    }
                    break;

                case PICKUP_MORE_GLYPH:

                    // turn on the intake wheels
                    robot.collectGlyph();

                    double collectSpeed = 0.35;
                    // drive to glyph pit
                    driveForwardInches(24.0, 0.80, 5.0);
                    sleepInAuto(350);

                    //driveForwardInches(4.0, collectSpeed, 2.0);
                    //sleepInAuto(200);
                    //driveBackwardInches(2.0, collectSpeed, 2.0);

                    turn(-103.5);

                    driveForwardInches(3.0, collectSpeed, 2.0);
                    sleepInAuto(250);

                    driveForwardInches(4.0, collectSpeed, 2.0);
                    sleepInAuto(250);

                    // push forward a bit to collect
                    //driveForwardInches(4.0, collectSpeed, 2.0);
                    //sleepInAuto(300);

                    driveBackwardInches(7.5, 0.5, 2.0);
                    turn(-89.0);
                    //robot.pushGlyph();

                    gotoNextState();
                    break;

                case DRIVE_BACK_TO_CRYPTO:

                    driveBackwardInches(20.0, 0.80, 5.0);

                    robot.extendDistanceSensorArmServo();
                    robot.pushGlyph();
                    sleepInAuto(300);

                    turn(-89.0);

                    // push #1
                    robot.holdPusher();
                    sleepInAuto(400);

                    // push #2
                    robot.pushGlyph();

                    logInfo( getRuntime() + " Col Dist: " + String.format("%.2f", robot.getColDistance()) + ", " + robot.columnDetected());

                    // detect if glyph is stuck
                    boolean glyphStucked = isGlyphStucked();

                    if(glyphStucked) {
                        // push again if glyph is stuck
                        robot.pushGlyph();
                        glyphStucked = isGlyphStucked();
                    }

                    robot.stopIntake();

                    if(!dumpMoreGlyphs() || glyphStucked || getRuntime() > 27.0) {
                        driveBackwardInches(3.50, 0.25, 2.0);
                        gotoState(READY_FOR_TELEOPS);
                        break;
                    }

                    //robot.pushGlyph();
                    //sleepInAuto(200);
                    //robot.moveUpPusher();

                    gotoNextState();
                    break;

                case PLACE_MORE_GLYPHS_INTO_CRYPTO:

                    collectSpeed = 0.35;

                    double columnDist = robot.getColDistance();

                    if( (robot.columnDetected() != null && !robot.columnDetected()) || (columnDist > 7.0 && columnDist < 100)) {
                        logInfo( getRuntime() + " Move to column: " + String.format("%.2f", columnDist));
                        driveBackwardInchesToColumn(7.25, collectSpeed, 2.0);
                    }

                    //robot.holdPusher();
                    sleepInAuto(200);
                    robot.pushGlyph();

                    // make sure 90 degree
                    turn(-89.0);

                    placeGlyphIntoColumn(fasterMotorSpeed, false);

                    // move pusher to hold position
                    robot.holdPusher();

                    // if have time, move back from the cryptobox
                    driveForwardInches(2.0, 0.5, 1.0);

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
                vuMarkVision.setFlashTorchMode(false);
            }
        }

        robot.onStop();
    }

    protected boolean do4Glyphs() {
        return false;
    }

    //------------------------------------
}