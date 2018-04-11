package org.firstinspires.ftc.teamcode.TechNova2017;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBase.State.END;
import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBase.State.PUSH_JEWEL;
import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBase.State.READY_FOR_TELEOPS;
import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBase.State.START;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.DISTANCE_SENSOR_UPRIGHT_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.DISTANCE_SENSOR_UPRIGHT_POSITION_2;

public class RelicRecoveryAutoTileRunnerBase extends RelicRecoveryAutoTileRunnerAbstract {

     // make new States for each autonomous strategy
     // like Auto 2, Auto 3 etc.
    //  the following is for Auto 1 (both BLUE and RED)
    //----------------------------------------------
     enum State implements AutoState {
         START,
         PUSH_JEWEL,
         GET_OFF_STONE,
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

        if(do4Glyphs()) {
            robot.resetGlyphTray();
        }

        // waiting for operator to press start button
        while(!isStarted()) {
            telemetry.addData("Distance (x1, x2): ",
                    String.format("(%.1f, %.1f)", robot.getX1Distance(), robot.getColDistance()) + " | " + robot.columnDetected());
            telemetry.addData("Glyph Color : ",robot.getGlyphColorRGB() +"");
            telemetry.addData("IMU", String.format("(%.1f)", robot.getHeadingAngle()));
            telemetry.addData("Touch", robot.isColumnTouched());
            telemetry.addData("Column", robot.columnDetected());
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
        v_state = PUSH_JEWEL;

        boolean detectVuMark = true;
        // 2. Run the state machine
        //  test, and more test
        //-------------------------------------------------------------------
        while (opModeIsActive() && v_state != END) {

            double motorSpeed = 0.28;
            double fasterMotorSpeed = 0.35;
            double straightAngle = -85.0;

            if(detectVuMark && vuMark == RelicRecoveryVuMark.UNKNOWN) {
                vuMark = vuMarkVision.detect(telemetry);
            }

            if(vuMark != RelicRecoveryVuMark.UNKNOWN) {
                robot.turnOnBlueLed();
                vuMarkVision.setFlashTorchMode(false);
            }

            logStateInfo(v_state, "Start");

            State step = (State)v_state;

            switch (step) {
                case START:
                    gotoNextState();
                    break;

                case PUSH_JEWEL:

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
                    detectVuMark = false;

                    if(getAllianceColor() ==  AllianceColor.RED) {
                        driveBackwardInches(24.5, motorSpeed, 4.0);
                    }
                    // if this is BLUE Alliance
                    else {
                        driveForwardInches(24.5, motorSpeed, 4.0);
                    }

                    switch (vuMark) {

                        // need to place glyph into RIGHT Crypto box
                        case RIGHT:
                            if(do4Glyphs()) {
                                if(getAllianceColor() ==  AllianceColor.RED) {
                                    driveBackwardInches(2.5, motorSpeed, 1.5);
                                }
                                // if this is BLUE Alliance
                                else {
                                    driveForwardInches(17.5, motorSpeed, 3.0);
                                }
                            } else {
                                if(getAllianceColor() ==  AllianceColor.RED) {
                                    driveBackwardInches(2.5, motorSpeed, 1.5);
                                }
                                // if this is BLUE Alliance
                                else {
                                    driveForwardInches(17.5, motorSpeed, 3.0);
                                }
                            }
                            break;
                        // need to place glyph into CENTER Crypto box
                        // -------------------------------------------------
                        case CENTER:
                            if(do4Glyphs()) {
                                if(getAllianceColor() ==  AllianceColor.RED) {
                                    driveBackwardInches(9.5, motorSpeed, 3.0);
                                }
                                // if this is BLUE Alliance
                                else {
                                    driveForwardInches(10.5, motorSpeed, 2.0);
                                }
                            } else {
                                if(getAllianceColor() ==  AllianceColor.RED) {
                                    driveBackwardInches(11.0, motorSpeed, 3.0);
                                }
                                // if this is BLUE Alliance
                                else {
                                    driveForwardInches(10.5, motorSpeed, 2.0);
                                }
                            }

                            break;

                        // need to place glyph into LEFT Crypto box
                        // -------------------------------------------------
                        case LEFT:
                            if(do4Glyphs()) {
                                if (getAllianceColor() == AllianceColor.RED) {
                                    //driveBackwardInches(18.5, motorSpeed, 3.0);
                                    driveBackwardInches(15.0, motorSpeed, 3.0);
                                }
                                // if this is BLUE Alliance
                                else {
                                    driveForwardInches(2.0, motorSpeed, 2.0);
                                }
                            }
                            else {
                                if (getAllianceColor() == AllianceColor.RED) {
                                    driveBackwardInches(18.5, motorSpeed, 3.0);
                                }
                                // if this is BLUE Alliance
                                else {
                                    driveForwardInches(2.0, motorSpeed, 2.0);
                                }
                            }
                            break;

                        // Default is CENTER position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            if(do4Glyphs()) {
                                if(getAllianceColor() ==  AllianceColor.RED) {
                                    driveBackwardInches(9.5, motorSpeed, 3.0);
                                }
                                // if this is BLUE Alliance
                                else {
                                    driveForwardInches(10.5, motorSpeed, 2.0);
                                }
                            } else {
                                if(getAllianceColor() ==  AllianceColor.RED) {
                                    driveBackwardInches(11.0, motorSpeed, 3.0);
                                }
                                // if this is BLUE Alliance
                                else {
                                    driveForwardInches(10.5, motorSpeed, 2.0);
                                }
                            }
                            break;
                    }

                    // turn right to 90 degree
                    // need to figure out the turn direction for
                    // red and blue alliance
                    if(getAllianceColor() ==AllianceColor.RED) {
                        turn(-82.5);
                    } else {
                        turn(-83.5);
                    }

                    gotoNextState();
                    break;

                case PICKUP_ANOTHER_GLYPH:

                    robot.resetForTeleOps();

                    if(do4Glyphs()) {

                        robot.resetGlyphTray();

                        driveForwardInches(16.0, 0.50, 3.0);
                        turn(straightAngle);
                        robot.collectGlyph();
                        sleepInAuto(100);

                        driveForwardInches(9.0, 0.35, 1.5);
                        sleepInAuto(100);

                        driveBackwardInches(8.5, 0.35, 1.5);

                        turn(straightAngle);

                        robot.pushGlyph();
                        robot.extendDistanceSensorArmServo();

                        driveBackwardInches(12.5,0.50,3.0);
                        robot.holdPusher();

                        if(isGlyphStucked()) {
                            robot.pushGlyph();
                            isGlyphStucked();
                        }

                        turn(straightAngle);
                        robot.stopIntake();
                    }

                    gotoNextState();
                    break;

                case FORWARD_TO_CRYPTOBOX:

                    robot.extendDistanceSensorArmServo();

                    if(do4Glyphs()) {
                        // push #2
                        robot.pushGlyph();
                        boolean columnTouched = driveBackwardInchesToColumn(8.50, 0.15, 3.0);
                        robot.holdPusher();

                        if(columnTouched) {
                             if(!robot.columnDetected()) {
                                 driveRightInches(3.0, 0.5, 1.0);
                                 driveBackwardInchesToColumn(3.0, 0.15, 1.0);
                             }
                             else {
                                 driveForwardInches(2.0, 0.25, 1.0);
                             }
                        }

                        robot.raiseGlyphTrayup2();
                    } else {
                        driveBackwardInches(6.05, 0.2, 2.0);
                    }

                    gotoNextState();
                    break;

                case PLACE_GLYPH_INTO_CRYPTO:

                    placeGlyphIntoColumn(fasterMotorSpeed, false);
                    driveForwardInches(2.0, 0.5, 2.0);
                    gotoNextState();

                    break;

                case PREPARE_FOR_MORE_GLYPHS:

                    if(getRuntime() > 22.0 || !pickupMoreGlyphs()) {
                        gotoState(READY_FOR_TELEOPS);
                        break;
                    } else {
                        robot.moveDistanceSensorArmServo(DISTANCE_SENSOR_UPRIGHT_POSITION);

                        //turn(-92.0);
                        double oneColumnDistance = 9.50;
                        if(getAllianceColor() == AllianceColor.RED) {
                            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                                driveLeftInches(oneColumnDistance+ 5.5, fasterMotorSpeed, 3.0);
                            }
                            else if (vuMark == RelicRecoveryVuMark.LEFT) {
                                driveLeftInches(oneColumnDistance+ 1.5, fasterMotorSpeed, 3.0);
                            }
                            else {
                                driveRightInches(oneColumnDistance+ 1.5, fasterMotorSpeed, 3.0);
                            }
                        } else {
                            if (vuMark == RelicRecoveryVuMark.LEFT) {
                                driveRightInches(oneColumnDistance+ 3.5, fasterMotorSpeed, 3.0);
                            }
                            else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                                driveLeftInches(oneColumnDistance + 0.5, fasterMotorSpeed, 3.0);
                            }
                            else {
                                driveLeftInches(oneColumnDistance + 0.5, fasterMotorSpeed, 3.0);
                            }
                        }

                        turn(straightAngle);
                        gotoNextState();
                    }
                    break;

                case PICKUP_MORE_GLYPH:

                    // turn on the intake wheels
                    robot.collectGlyph();

                    double collectSpeed = 0.35;
                    // drive to glyph pit
                    driveForwardInches(22.0, 0.70, 5.0);
                    sleepInAuto(50);
                    //driveBackwardInches(2.0, collectSpeed, 2.0);

                    if(getAllianceColor() == AllianceColor.RED) {
                        turnToAngle(-74.5, 0.25);
                    } else {
                        turnToAngle(-103.5, 0.25);
                    }

                    driveForwardInches(5.0, collectSpeed, 1.25);
                    sleepInAuto(50);

                    driveForwardInches(7.0, collectSpeed, 1.25);
                    sleepInAuto(50);

                    // push forward a bit to collect
                    //driveForwardInches(4.0, collectSpeed, 2.0);
                    //sleepInAuto(300);

                    driveBackwardInches(12.5, 0.5, 2.0);
                    turn(straightAngle);
                    robot.pushGlyph();

                    gotoNextState();
                    break;

                case DRIVE_BACK_TO_CRYPTO:

                    driveBackwardInches(15.5, 0.45, 3.0);
                    robot.extendDistanceSensorArmServo();

                    robot.pushGlyph();
                    turn(straightAngle);

                    logInfo( " Col Dist: " + String.format("%.2f", robot.getColDistance()) + ", " + robot.columnDetected());

                    boolean glyphStucked = false;
                    if(!do4Glyphs() && getRuntime() < 24.0) {
                        // detect if glyph is stuck
                        glyphStucked = isGlyphStucked();

                        if (glyphStucked) {
                            // push again if glyph is stuck
                            robot.pushGlyph();
                            glyphStucked = isGlyphStucked();
                        }
                    }

                    robot.stopIntake();

                    if(!dumpMoreGlyphs() || glyphStucked || getRuntime() > 27.0) {
                        driveBackwardInches(2.50, 0.25, 2.0);
                        gotoState(READY_FOR_TELEOPS);
                        break;
                    }

                    //robot.pushGlyph();
                    //sleepInAuto(200);
                    //robot.moveUpPusher();

                    gotoNextState();
                    break;

                case PLACE_MORE_GLYPHS_INTO_CRYPTO:

                    turn(straightAngle);

                    double columnDist = robot.getColDistance();

                    if( (robot.columnDetected() != null && !robot.columnDetected())
                            || (columnDist > 2.5 && columnDist < 100)) {
                        logInfo( " Move to column: " + String.format("%.2f", columnDist) + " | " + robot.columnDetected());
                        boolean columnTouched = driveBackwardInchesToColumn(7.25, 0.18, 2.0);

                        if(columnTouched) {
                            if (!robot.columnDetected()) {
                                driveRightInches(4.0, 0.35, 3.0);
                                if(!robot.columnDetected()) {
                                    driveBackwardInchesToColumn(2.5, 0.18, 1.0);
                                }
                            } else {
                                driveForwardInches(1.5, 0.35, 1.0);
                            }
                        }
                    } else {
                        logInfo( " Col Detected: " + String.format("%.2f", columnDist) + " | " + robot.columnDetected());
                        sleepInAuto(100);
                    }

                    robot.pushGlyph();

                    // make sure 90 degree
                    //turn(-87.5);

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
        }

        robot.onStop();
    }

    protected boolean do4Glyphs() {
        return false;
    }

    //------------------------------------
}