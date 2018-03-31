package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.teamcode.TechNova2017.RelicRecoveryAutoTileRunnerBase.State.END;
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
                                driveForwardInches(9.5, motorSpeed, 5.0);
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
                                driveForwardInches(2.5, motorSpeed, 2.0);
                            }
                            break;

                        // Default is CENTER position, in case Vumark is not visible
                        // -------------------------------------------------
                        default:
                            if(getAllianceColor() ==  AllianceColor.RED) {
                                driveBackwardInches(12.0, motorSpeed, 5.0);
                            }
                            // if this is BLUE Alliance
                            else {
                                driveForwardInches(11.0, motorSpeed, 5.0);
                            }
                            break;
                    }

                    gotoNextState();
                    break;

                case TURN_TO_90_DEGREE:

                    robot.setServoPosition(robot.distSensorServo, DISTANCE_SENSOR_UPRIGHT_POSITION);

                    // turn right to 90 degree
                    // need to figure out the turn direction for
                    // red and blue alliance
                    if(getAllianceColor() ==AllianceColor.RED) {
                        turn(-83.5);
                    } else {
                        turn(-84.5);
                    }

                    //sleepInAuto(250);

                    //turn(-88.5);

                    gotoNextState();
                    break;

                case FORWARD_1_FEET:
                    robot.extendDistanceSensorArmServo();
                    driveBackwardInches(6.75, motorSpeed, 2.0);
                    gotoNextState();
                    break;

                case PLACE_GLYPH_INTO_CRYPTO:
                    placeGlyphIntoColumn(fasterMotorSpeed);
                    gotoNextState();

                    break;

                case RESET_GLYPH_TRAY:

                    // move the glyph lift back to zero position
                    robot.resetForTeleOps();

                    if(getRuntime() > 23.0 || !pickupMoreGlyphs()) {
                        gotoState(END);
                    } else {
                        //turn(-92.0);
                        double oneColumnDistance = 5.75;
                        if(getAllianceColor() == AllianceColor.RED) {
                            if (vuMark == RelicRecoveryVuMark.LEFT) {
                                driveRightInches(oneColumnDistance+0.5, fasterMotorSpeed, 2.0);
                            } else {
                                driveLeftInches(oneColumnDistance, fasterMotorSpeed, 2.0);
                            }
                        } else {
                            if (vuMark == RelicRecoveryVuMark.LEFT) {
                                driveRightInches(oneColumnDistance + 2.5, fasterMotorSpeed, 2.0);
                            } else {
                                driveLeftInches(oneColumnDistance, fasterMotorSpeed, 2.0);
                            }
                        }

                        turn(-89.0);
                        gotoNextState();
                    }
                    break;

                case PICKUP_SECOND_GLYPH:

                    // turn on the intake wheels
                    robot.collectGlyph();

                    double collectSpeed = 0.35;
                    // drive to glyph pit
                    driveForwardInches(22.0, 0.80, 5.0);
                    sleepInAuto(200);

                    driveForwardInches(6.0, 0.25, 2.0);
                    sleepInAuto(200);

                    driveBackwardInches(2.0, collectSpeed, 2.0);

                    turn(-115.0);

                    driveForwardInches(4.0, collectSpeed, 2.0);
                    sleepInAuto(300);

                    // push forward a bit to collect
                    driveForwardInches(4.0, collectSpeed, 2.0);
                    sleepInAuto(500);

                    driveBackwardInches(8.0, collectSpeed, 2.0);
                    turn(-89.0);
                    //robot.pushGlyph();

                    driveBackwardInches(27.0, 0.75, 5.0);

                    robot.extendDistanceSensorArmServo();
                    robot.pushGlyph();

                    turn(-89.0);

                    robot.holdPusher();
                    sleepInAuto(300);
                    robot.pushGlyph();

                    int previousIntakeCount = robot.intakeRight.getCurrentPosition();
                    sleepInAuto(200);
                    robot.holdPusher();
                    sleepInAuto(200);

                    boolean glyphStucked = false;
                    if(Math.abs(previousIntakeCount - robot.intakeRight.getCurrentPosition()) < 20) {
                        robot.reverseGlyph();
                        sleepInAuto(1500);
                        robot.collectGlyph();
                        glyphStucked = true;
                    }

                    if(!dumpMoreGlyphs() || glyphStucked) {
                        //driveBackwardInches(2.50, 0.25, 2.0);
                        robot.stopIntake();
                        gotoNextState();
                        break;
                    }

                    robot.stopIntake();
                    robot.pushGlyph();
                    sleepInAuto(200);
                    robot.moveUpPusher();

                    if(getRuntime() < 27.0) {

                        logInfo(" Move forward to dump second glyph...");
                        driveBackwardInchesToColumn(5.25, collectSpeed, 2.0);

                        ElapsedTime timer2 = new ElapsedTime();

                        while(opModeIsActive() && timer2.seconds() < 2.0 && getRuntime() < 28.5) {
                            double columnDist = robot.getColDistance();
                            logInfo(" Second glyph distance:" + String.format("%.1f", columnDist));

                            if (columnDist > 10.0) {
                                driveBackwardInches(2.0, 0.25, 2.0);
                            }  else {
                                break;
                            }
                        }

                        robot.holdPusher();
                        sleepInAuto(200);
                        robot.pushGlyph();

                        placeGlyphIntoColumn(fasterMotorSpeed);
                    }

                    driveBackwardInches(3.0, 0.5, 1.0);

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

    //------------------------------------
}