package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FTC 9915
 */
@Autonomous(name="Auto TW Test", group="Test")
@Disabled
public class VelocityVortexAuto_TW extends ETC2016AutoBaseOpMode {

    //**********************************************************************************************
    //      State Machine
    //
    //**********************************************************************************************

    //      Run Forward, Turn, and Reach Tape
    protected final static int MOVE_12_INCHES = 1;
    protected final static int SHOOT_1_BALL = 2;
    protected final static int SHOOT_2_BALL = 3;

    protected final static int TURN_20_DEGREE = 4;
    protected final static int TURN_TO_0_DEGREE = 5;
    protected final static int PUSH_BEACON_BUTTON = 6;
    protected final static int FORWARD_ONE_FOOT = 7;
    protected final static int BLUE_TURN_40_DEGREE = 8;
    protected final static int BLUE_TURN_TO_0_DEGREE = 9;
    //
    //**********************************************************************************************
    //      End
    private final static int END = 100;

    //
    //**********************************************************************************************
    //
    //  class variables
    //
    private boolean buttonPressed = false;
    private Boolean beaconDetected = null;
    private long startTime = 0;

    private int counter = 0;

    public VelocityVortexAuto_TW() {
    }

    @Override
    public void init_loop() {
        super.init_loop();
        this.robot.ballShooter.init_loop(true, true);

        if(elapsedTime.time() < 1000) {
            robot.collectorMotor.setPower(1.0);
        } else {
            robot.collectorMotor.setPower(0.0);
        }

        if(elapsedTime.time() > 7000) {
            logSensorInfo(true);
            telemetry.update();
        }
    }

    @Override
    public void start() {
        super.start();

        robot.collectorMotor.setPower(0.0);

        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Log.i("ETCTeleOpBase", " === Autonomous Started ===");

        timer.reset();
        elapsedTime.reset();

        this.robot.setColorSensorParticleEnabled(false);
    }

    //**********************************************************************************************
    //  loop
    //

    @Override
    public void loop() {
        if (prev_state != v_state) {
            logSensorInfo(true);
            telemetry.addData("10.", "Enter state: " + v_state);
            prev_state = v_state;
        }

        logSensorInfoOnChange();

        switch (v_state) {
            case START:
//                if(robot.buttonPusher.pushButton()) {
//                    logInfo("Button Pushed ... ");
//                    goToNextState();
//                }
                goToNextState();
                break;

            case MOVE_12_INCHES:
                if(this.robot.mediumTurn(120.0)) {
                    goToNextState();
                }

//                if(this.robot.setPowerRunToPosition(0.5,0.5,(int)(COUNTS_PER_INCH*12.0))) {
//                    goToNextState();
//                }
                break;

            case SHOOT_1_BALL:
                this.robot.setColorSensorFrontEnabled(false);
                this.robot.setColorSensorBackEnabled(false);
                this.robot.setColorSensorBeaconEnabled(false);

                if(this.robot.mediumTurn(0.0)) {
                    goToState(END);
                }

//                if (this.robot.ballShooter.isReadyToFire()) {
//                    if (counter ==0) {
//                        this.robot.ballShooter.fire();
//                        logInfo("Fire Called: " + this.robot.ballShooter.getShooterState());
//                        counter++;
//                    } else {
//                        goToNextState();
//                    }
//                }
//
//                this.robot.ballShooter.fire_loop();
//                this.robot.ballShooter.moveStopper(0.0);
//
//                if(timer.time() > 3000) {
//                    goToNextState();
//                }

                break;

            case SHOOT_2_BALL:
                if (timer.time() > 500 &&
                        this.robot.ballShooter.isReadyToFire()) {
                    if (counter == 1) {
                        this.robot.ballShooter.fire();
                        counter++;
                    }
                    else {
                        goToNextState();
                    }
                }
                this.robot.ballShooter.fire_loop();
                this.robot.ballShooter.moveStopper(0.0);

                if(timer.time() > 1500 && this.robot.ballShooter.isReloadingCompleted()) {
                    goToNextState();
                }

                if(timer.time() > 3000) {
                    goToNextState();
                }

                break;

            case TURN_20_DEGREE:

                if(this.robot.mediumTurn(20.0)) {
                    goToNextState();
                }

                break;


            case TURN_TO_0_DEGREE:

                if(this.robot.slowTurn(0.0)) {
                    goToNextState();
                }

                break;

            case PUSH_BEACON_BUTTON:

                this.robot.ballShooter.fire_loop();
                this.robot.ballShooter.moveStopper(0.0);
                if(robot.buttonPusher.pushButton() || timer.time() > 3000) {
                    this.robot.ballShooter.force_stop();
                    buttonPressed = true;
                    beaconDetected = null;
                    logInfo("Button Pushed ... ");
                    goToState(END);
                }

                break;

            case FORWARD_ONE_FOOT:

                if(this.robot.setPowerRunToPosition(-0.5,-0.5,(int)(-COUNTS_PER_INCH*12.0))) {
                    goToNextState();
                }

                break;

            case BLUE_TURN_40_DEGREE:

                if(this.robot.mediumTurn(-40.0)) {
                    goToNextState();
                }

                break;

            case BLUE_TURN_TO_0_DEGREE:

                if(this.robot.slowTurn(-5.0)) {
                    goToNextState();
                }

                break;


            default:
                robot.buttonPusher.reset();
                v_state = END;
                break;
        }
    }

    @Override
    public AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }

}

//**************************************************************************************************