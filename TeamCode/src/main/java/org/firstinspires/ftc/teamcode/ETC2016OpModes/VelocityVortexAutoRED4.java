package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ETCHardware.ETCConstants;

/**
 * Created by FTC 9915
 */
@Autonomous(name="Auto Red 2 Alliance", group="9915 Red")
@Disabled
public class VelocityVortexAutoRED4 extends ETC2016AutoBaseOpMode {

    //**********************************************************************************************
    //      State Machine
    protected final static int START = 0;
    //
    //**********************************************************************************************
    //      Run Forward, Turn, and Reach Tape

    protected final static int SHOOT_1_BALL         = 1;
    protected final static int SHOOT_2_BALL         = 2;
    protected final static int MOVE_FORWARD_HALF_FOOT = 3;
    protected final static int TURN_90_DEGREES      = 4;
    protected final static int WAIT_SECONDS         = 5;
    protected final static int MOVE_TO_VORTEX       = 6;
    //
    //**********************************************************************************************
    //      End
    private final static int END = 100;

    //
    //**********************************************************************************************
    //
    //  class variables
    //
    private Boolean beaconDetected = null;
    private int counter = 0;
    private Boolean whiteLineDetected = null;
    private boolean firstButtonPushed = false;

    public VelocityVortexAutoRED4() {
    }

    @Override
    public void init_loop() {
        super.init_loop();
        robot.ballShooter.initialTension = ETCConstants.ALLIANCE_AUTONOMOUS_TENSION;
        boolean shooterInitSuccess = this.robot.ballShooter.init_loop(true, true);

        if(elapsedTime.time() > 5000) {
            telemetry.addLine("Sensor Data: ").addData(" ", getSensorInfo(true, false));
            telemetry.addLine(" ");
            telemetry.addLine("Tension Position: " + this.robot.ballShooter.getTensionPosition());
            telemetry.addLine("Shooter Status: " + shooterInitSuccess);
            telemetry.addLine("Shooter Switch: " + this.robot.ballShooter.isShooterSwitchPressed());
            telemetry.addLine("Tension switch: " + this.robot.ballShooter.isTensionSwitchPressed());
            telemetry.update();
        }
    }

    @Override
    public void start() {
        super.start();

        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.robot.setColorSensorParticleEnabled(false);

        logInfo(" === Velocity Vortex RED Autonomous Started ===");
        elapsedTime.reset();
        timer.reset();

        resetStartTime();
        robot.ballShooter.resetGameTimer();

        enableColorSensorRegister = false;
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

        //logSensorInfoOnChange();

        switch (v_state) {
            case START:
                if (elapsedTime.time() >=20) {
                    goToNextState();
                }

                break;

//------------------------Change wait time as necessary------------------------------------
            case SHOOT_1_BALL:

                if (this.robot.ballShooter.isReadyToFire()) {
                    if (counter == 0) {
                        this.robot.ballShooter.fire();
                        logInfo("Auto Fire 1: " + this.robot.ballShooter.getShooterState());
                        counter++;
                    } else {
                        goToNextState();
                    }
                }

                this.robot.ballShooter.fire_loop();
                this.robot.ballShooter.moveStopper(0.0);

                if(timer.time() > 3000) {
                    goToNextState();
                }

                break;

            case SHOOT_2_BALL:
                if (this.robot.ballShooter.isReadyToFire()) {
                    if (counter == 1) {
                        this.robot.ballShooter.fire();
                        logInfo("Auto Fire 2: " +
                                     this.robot.ballShooter.getShooterState());
                        counter++;
                    }
                    else {
                        goToNextState();
                    }
                }
                this.robot.ballShooter.fire_loop();
                this.robot.ballShooter.moveStopper(0.0);

                if(timer.time() > 500 && this.robot.ballShooter.isReloadingCompleted()) {
                    goToNextState();
                }

                if(timer.time() > 3000) {
                    goToNextState();
                }

                break;

            case MOVE_FORWARD_HALF_FOOT:

                this.robot.ballShooter.force_stop();

                if(parkAtCenter()) {
                    if (this.robot.setPowerRunToPosition(-0.5, -0.5, (int) (-COUNTS_PER_INCH * 18.0))) {
                        goToNextState();
                    }
                }
                else {
                    goToState(END);
                }

                break;

            case TURN_90_DEGREES:

                if(this.robot.mediumTurn(-45.0))
                {
                    goToNextState();
                }
                break;


            case WAIT_SECONDS:
                if(elapsedTime.time() > ETCConstants.POSITION_2_DELAY_PARKING_TIME) {
                    goToNextState();
                }
                break;

            case MOVE_TO_VORTEX:

                this.robot.collectorMotor.setPower(-1.0);
                if (this.robot.setPowerRunToPosition(0.75, 0.75, (int)(COUNTS_PER_INCH*45.0))) {
                    goToNextState();
                }

                break;

            default:
                robot.buttonPusher.reset();
                this.robot.collectorMotor.setPower(0.0);

                if(robot.ballShooter.moveTensionToLowPosition()) {
                    robot.ballShooter.stopTensionMotor();
                }
                v_state = END;
                break;
        }

        isStateStart = false;
    }

    @Override
    public AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
}


//**************************************************************************************************