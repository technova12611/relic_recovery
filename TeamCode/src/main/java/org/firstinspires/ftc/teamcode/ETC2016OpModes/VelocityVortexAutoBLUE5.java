package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ETCHardware.ETCConstants;

import static org.firstinspires.ftc.teamcode.ETCHardware.ETCConstants.SPEED_GEAR_RATIO_FACTOR;

/**
 * Created by FTC 9915
 */
@Autonomous(name="Auto Blue 2 Alliance Defense", group="9915 Blue")
@Disabled
public class VelocityVortexAutoBLUE5 extends ETC2016AutoBaseOpMode {

    //**********************************************************************************************
    //      State Machine
    private final static int START = 0;
    //
    //**********************************************************************************************
    //      Run Forward, Turn, and Reach Tape
    protected final static int WAIT_SECONDS         = 1;
    protected final static int SHOOT_1_BALL         = 2;
    protected final static int SHOOT_2_BALL         = 3;
    protected final static int MOVE_FORWARD_HALF_FOOT = 4;
    protected final static int TURN_90_DEGREES      = 5;
    protected final static int MOVE_TO_VORTEX       = 6;
    protected final static int TURN_TO_120_DEGREES = 7;
    protected final static int MOVE_TO_BETWEEN_BEACONS = 8;
    protected final static int ADJUST_TO_90_DEGREE = 9;
    protected final static int WAIT_TO_BLOCK_OPPONENT = 10;
    protected final static int PARK_AT_VOTEX          = 11;
    //
    //**********************************************************************************************
    //      End
    private final static int END = 100;

    //
    //**********************************************************************************************
    //
    //  class variables
    //

    private Boolean whiteLineDetected = null;
    private Boolean beaconDetected = null;
    private int counter = 0;
    private boolean firstButtonPushed = false;

    public VelocityVortexAutoBLUE5() {
    }

    @Override
    public void init_loop() {
        super.init_loop();
        robot.ballShooter.initialTension = ETCConstants.ALLIANCE_AUTONOMOUS_TENSION;
        boolean shooterInitSuccess = this.robot.ballShooter.init_loop(true, true);

        if(elapsedTime.time() > 7000) {
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

            case WAIT_SECONDS:
                if(timer.time() > 0) {
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

                if (this.robot.setPowerRunToPosition(0.5, 0.5, (int)(COUNTS_PER_INCH*8.0))) {
                    goToNextState();
                }

                break;

            case TURN_90_DEGREES:

                if(this.robot.fastTurn(-135.0))
                {
                    goToNextState();
                }
                break;

            case MOVE_TO_VORTEX:

                this.robot.collectorMotor.setPower(-1.0);

                driveMotorPower = timer.time() < 300?0.20:0.75;
                if (this.robot.setPowerRunToPosition(driveMotorPower, driveMotorPower, (int)(COUNTS_PER_INCH*70.0))) {
                    goToNextState();
                }

                break;

            case TURN_TO_120_DEGREES:

                this.robot.collectorMotor.setPower(0.0);
                if (this.robot.fastTurn(-69.0) || timer.time() > 3000) {
                    goToNextState();
                }

                break;

            case MOVE_TO_BETWEEN_BEACONS:

                driveMotorPower = timer.time() < 300?0.20:0.75;
                if (elapsedTime.time() > 10500 &&
                        this.robot.setPowerRunToPosition(driveMotorPower, driveMotorPower, (int)(COUNTS_PER_INCH*60.0))) {
                    goToNextState();
                }
                break;

            case ADJUST_TO_90_DEGREE:

                this.robot.setPowerUsingEncoders(0,-0.25*SPEED_GEAR_RATIO_FACTOR);

                if(this.robot.hasTargetAngleReached(-90.0,-1) || timer.time() > 2000) {
                    goToNextState();
                }

                break;

            case WAIT_TO_BLOCK_OPPONENT:

                this.robot.collectorMotor.setPower(0.0);

                if(robot.ballShooter.moveTensionToLowPosition()) {
                    robot.ballShooter.stopTensionMotor();
                }

                if(timer.time() > 5000) {
                    robot.ballShooter.stopTensionMotor();
                }

                if(elapsedTime.time() > 25000) {
                    if (this.robot.fastTurn(-110.0)) {
                        goToNextState();
                    }
                }
                break;

            case PARK_AT_VOTEX:
                if(parkAtCenter()) {
                    if (this.robot.setPowerRunToPosition(-0.6*SPEED_GEAR_RATIO_FACTOR,
                            -0.6*SPEED_GEAR_RATIO_FACTOR, (int) (-COUNTS_PER_INCH * 36.0))) {
                        goToNextState();
                    }
                } else {
                    goToState(END);
                }
                break;
            default:
                robot.buttonPusher.reset();
                this.robot.collectorMotor.setPower(0.0);
                v_state = END;
                break;
        }
    }


    protected int getAlliance() {
        return BLUE_ALLIANCE;
    }

    @Override
    public AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }
}

//**************************************************************************************************