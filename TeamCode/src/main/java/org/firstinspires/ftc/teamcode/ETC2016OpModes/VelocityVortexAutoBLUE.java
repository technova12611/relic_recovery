package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ETCHardware.ETC2016Robot;

import static org.firstinspires.ftc.teamcode.ETCHardware.ETCConstants.SPEED_GEAR_RATIO_FACTOR;

/**
 * Created by FTC 9915
 */
@Autonomous(name="Auto Blue 1", group="9915 Blue")
@Disabled
public class VelocityVortexAutoBLUE extends ETC2016AutoBaseOpMode {

    //**********************************************************************************************
    //      State Machine
    private final static int START = 0;
    //
    //**********************************************************************************************
    //      Run Forward, Turn, and Reach Tape
    protected final static int TURN_40_DEGREES         = 1;
    protected final static int MOVE_3_FEET             = 2;
    protected final static int SHOOT_1_BALL            = 3;
    protected final static int SHOOT_2_BALL            = 4;
    protected final static int ADJUST_ANGLE            = 5;
    protected final static int MOVE_1_FOOT             = 6;
    protected final static int TURN_TO_10_DEGREES      = 7;
    protected final static int MOVE_TO_TAPE            = 8;
    protected final static int TURN_PARALLEL_TO_BEACON = 9;
    protected final static int MOVE_BACKWARD_TO_BEACON = 10;
    protected final static int PUSH_BEACON_BUTTON      = 11;
    protected final static int BEACON_BUTTON_PUSHED    = 12;
    protected final static int MOVE_TO_SECOND_BUTTON   = 13;
    protected final static int PUSH_OTHER_BUTTON       = 14;
    protected final static int OTHER_BUTTON_PRESSED    = 15;
    protected final static int MOVE_TO_0_DEGREES       = 16;
    protected final static int MOVE_BETWEEN_BEACONS    = 17;
    protected final static int TURN_DEGREES            = 18;
    protected final static int Turn_TO_0_DEGREES       = 19;
    protected final static int MOVE_TO_TAPE_2          = 20;
    protected final static int Turn_TO_0_DEGREES_AGAIN = 21;
    protected final static int PUSH_BEACON_BUTTON_2    = 22;
    protected final static int MOVE_TO_SECOND_BUTTON_2 = 23;
    protected final static int PUSH_OTHER_BUTTON_2     = 24;
    protected final static int TURN_45_DEGREES_TO_CAPBALL = 25;
    protected final static int MOVE_BACK_TO_HIT_CAPBALL = 26;
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
    private Boolean defenseTargetReached = null;

    public VelocityVortexAutoBLUE() {
    }

    @Override
    public void init_loop() {
        super.init_loop();
        boolean shooterInitSuccess = this.robot.ballShooter.init_loop(true, false);

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

        logInfo(" === Velocity Vortex Autonomous BLUE Started ===");
        elapsedTime.reset();
        timer.reset();

        resetStartTime();
        robot.ballShooter.resetGameTimer();

        this.robot.setColorSensorParticleEnabled(false);
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

        switch (v_state) {
            case START:
                if (elapsedTime.time() > 20) {
                    if(enableColorSensorRegister) {
                        this.robot.setColorSensorFrontEnabled(false);
                        this.robot.setColorSensorBackEnabled(false);
                        this.robot.setColorSensorBeaconEnabled(false);
                    }

                    goToState(MOVE_3_FEET);
                }

                break;

            case TURN_40_DEGREES:
                goToNextState();
                break;

            case MOVE_3_FEET:
                driveMotorPower = (timer.time() < 300?-0.20:-0.60) * SPEED_GEAR_RATIO_FACTOR;
                if (this.robot.setPowerRunToPosition(driveMotorPower,driveMotorPower, (int)(-COUNTS_PER_INCH*27.0))) {
                    goToNextState();
                }
                break;

            case SHOOT_1_BALL:

                if (this.robot.ballShooter.isReadyToFire()) {
                    if (counter == 0) {
                        this.robot.ballShooter.fire();
                        logInfo("Auto Blue Fire 1: " + this.robot.ballShooter.getShooterState());
                        counter++;
                    } else {
                        goToNextState();
                    }
                }

                this.robot.ballShooter.fire_loop();
                this.robot.ballShooter.moveStopper(0.0);

                if(getNumOfParticles() == 1 &&
                        (timer.time() > 1000 || this.robot.ballShooter.isReloadingCompleted())) {
                    goToState(ADJUST_ANGLE);
                    break;
                }

                if(timer.time() > 3000) {
                    goToNextState();
                }

                break;

            case SHOOT_2_BALL:
                if (this.robot.ballShooter.isReadyToFire()) {
                    if (counter == 1) {
                        this.robot.ballShooter.fire();
                        logInfo("Auto Blue Fire 2: " + this.robot.ballShooter.getShooterState());
                        counter++;
                    }
                    else {
                        goToNextState();
                    }
                }
                this.robot.ballShooter.fire_loop();
                this.robot.ballShooter.moveStopper(0.0);

                if(timer.time() > 500 && this.robot.ballShooter.isReloadingCompleted()) {
                    this.robot.ballShooter.force_stop();
                    goToNextState();
                }

                if(timer.time() > 3000) {
                    goToNextState();
                }

                break;

            case ADJUST_ANGLE:
                this.robot.ballShooter.fire_loop();
                if(this.robot.slowTurn(-41.0) || timer.time() > 1500)
                {
                    goToNextState();
                }
                break;

            case MOVE_1_FOOT:
                if(enableColorSensorRegister) {
                    this.robot.setColorSensorFrontEnabled(false);
                    this.robot.setColorSensorBackEnabled(true);
                    this.robot.setColorSensorBeaconEnabled(false);
                }

                this.robot.ballShooter.fire_loop();
                //this.robot.ballShooter.moveStopper(0.0);

                driveMotorPower = (timer.time() < 200?-0.20:-0.40) * SPEED_GEAR_RATIO_FACTOR;

                if(this.robot.setPowerRunToPosition(driveMotorPower, driveMotorPower, (int)(-COUNTS_PER_INCH*32.5))) {
                    goToNextState();
                }

                if (this.robot.detectWhiteTapeAny())
                {
                    whiteLineDetected = true;
                }
                break;

            case TURN_TO_10_DEGREES:
                this.robot.ballShooter.force_stop();
                //this.robot.ballShooter.moveStopper(0.0);

                if(timer.time() < 100) {
                    break;
                }

                double turnAngle =  Range.clip(-26.0 + (-40.0 - robot.stateBeginHeadingAngle), -30.0, -20.0);

                if (this.robot.mediumTurn(turnAngle)) {
                    //this.robot.ballShooter.force_stop();
                    goToNextState();
                }

                if (this.robot.detectWhiteTapeAny())
                {
                    whiteLineDetected = true;
                }
                break;

            case MOVE_TO_TAPE:
                //move till white tape
                if (this.robot.detectWhiteTapeAny())
                {
                    whiteLineDetected = true;
                }

                this.robot.setPowerUsingEncoders(-MOVE_TO_WHITE_TAPE_POWER, -MOVE_TO_WHITE_TAPE_POWER);
                if (this.robot.getRangeSensorDistance() < 5.95) {
                   goToNextState();
                }

                break;

            case TURN_PARALLEL_TO_BEACON:
                this.robot.setPowerUsingEncoders(-0.1*SPEED_GEAR_RATIO_FACTOR, 0.1*SPEED_GEAR_RATIO_FACTOR);
                if(this.robot.hasTargetAngleReached(-1.5, 1))
                {
                    goToNextState();
                }
                break;

            case MOVE_BACKWARD_TO_BEACON:
                if(enableColorSensorRegister) {
                    this.robot.setColorSensorFrontEnabled(true);
                    this.robot.setColorSensorBackEnabled(true);
                    this.robot.setColorSensorBeaconEnabled(true);
                }

                if (whiteLineDetected != null) {
                    this.robot.setPowerUsingEncoders(MOVE_TO_WHITE_TAPE_POWER, MOVE_TO_WHITE_TAPE_POWER);
                    if (this.robot.getBeaconColorSensor() != ETC2016Robot.BeaconColor.UNKNOWN) {
                        beaconColor = this.robot.getBeaconColorSensor();
                    }
                    if ((this.robot.detectWhiteTape() &&
                            beaconColor == ETC2016Robot.BeaconColor.BLUE)) {
                        firstButtonPushed = true;
                        goToNextState();
                    } else if(this.robot.detectWhiteTape2()) {
                        beaconColor = this.robot.getBeaconColorSensor();
                        goToNextState();
                    }
                }
                else {
                    this.robot.setPowerUsingEncoders(-MOVE_TO_WHITE_TAPE_POWER, -MOVE_TO_WHITE_TAPE_POWER);
                    if (this.robot.getBeaconColorSensor() != ETC2016Robot.BeaconColor.UNKNOWN) {
                        beaconColor = this.robot.getBeaconColorSensor();
                    }
                    if( (this.robot.detectWhiteTape2() &&
                            beaconColor == ETC2016Robot.BeaconColor.BLUE)) {
                        goToNextState();
                    } else if(this.robot.detectWhiteTape()) {
                        beaconColor = this.robot.getBeaconColorSensor();
                        firstButtonPushed = true;
                        goToNextState();
                    } else if(timer.time() > 1000) {
                        whiteLineDetected = true;
                    }
                }

                break;

            case PUSH_BEACON_BUTTON:
                //pushButton the correct button on the beacon box
                if( ((firstButtonPushed && beaconColor == ETC2016Robot.BeaconColor.BLUE) ||
                        this.robot.getBeaconColorSensor() == ETC2016Robot.BeaconColor.BLUE) &&
                        this.robot.getBeaconColorSensor() != ETC2016Robot.BeaconColor.RED)
                {
                    if(beaconDetected == null)
                    {
                        timer.reset();
                        beaconDetected = true;
                    }

                    this.robot.slowTurn(0.0);
                    if(robot.buttonPusher.pushButton() || timer.time() > 5000) {
                        beaconDetected = null;
                        if(firstButtonPushed) {
                            logInfo("First Blue Beacon Pushed");
                        } else {
                            logInfo("Second Blue Beacon Pushed");
                        }
                        goToNextState();
                    }
                }
                else if (this.robot.getBeaconColorSensor() == ETC2016Robot.BeaconColor.RED)
                {
                    logInfo("Blue Auto: Red Beacon Detected");
                    beaconDetected = null;

                    goToState(MOVE_TO_SECOND_BUTTON);
                }
                else
                {
                    logInfo("No beacon detected");

                    this.robot.setPowerUsingEncoders(0.1, 0.1);

                    if(timer.time() > 3000) {
                        goToState(MOVE_TO_0_DEGREES);
                    }
                }
                break;

            case BEACON_BUTTON_PUSHED:
                robot.buttonPusher.reset();
                goToState(MOVE_TO_0_DEGREES);
                break;

            case MOVE_TO_SECOND_BUTTON:
                if(this.robot.setPowerRunToPosition(-MOVE_TO_WHITE_TAPE_POWER, -MOVE_TO_WHITE_TAPE_POWER, -1*(int)(COUNTS_PER_INCH*5.5)) ||
                        this.robot.detectWhiteTape()){
                    goToNextState();
                }

                if(timer.time() > 3000) {
                    goToState(MOVE_TO_0_DEGREES);
                }
                break;

            case PUSH_OTHER_BUTTON:
                if(beaconDetected == null)
                {
                    beaconDetected = true;
                    timer.reset();
                    logInfo("First Blue Beacon Detected");
                }

                if(this.robot.getBeaconColorSensor() == ETC2016Robot.BeaconColor.RED) {
                    whiteLineDetected = true;
                    goToState(MOVE_BACKWARD_TO_BEACON);
                    break;
                }

                this.robot.slowTurn(0.0);
                if(this.robot.buttonPusher.pushButton() || timer.time() > 6000)
                {
                    beaconDetected = null;
                    firstButtonPushed = true;
                    logInfo("First Blue Beacon Pushed");
                    goToNextState();
                }
                break;

            case OTHER_BUTTON_PRESSED:
                robot.buttonPusher.reset();
                goToNextState();
                break;

            case MOVE_TO_0_DEGREES:

                if(enableColorSensorRegister) {
                    this.robot.setColorSensorFrontEnabled(false);
                    this.robot.setColorSensorBackEnabled(false);
                    this.robot.setColorSensorBeaconEnabled(false);
                }

                if(this.robot.slowTurn(0.0)){
                    goToNextState();
                    betweenBeaconStartEncoderCount = this.robot.leftDriveMotor.getCurrentPosition();
                }

                break;

            case MOVE_BETWEEN_BEACONS:
                driveMotorPower = (timer.time() < 300?-0.20:-0.55) * SPEED_GEAR_RATIO_FACTOR;

                int distance = (int) (-COUNTS_PER_INCH * 37.0);
                distanceBetweenBeacons = distance;

                if (this.robot.setPowerRunToPosition(driveMotorPower, driveMotorPower, distance)) {
                    goToNextState();
                }
                else if ((this.timer.time() > 1050 &&
                        (this.robot.getRangeSensorDistance() <= 3.5 || this.robot.getRangeSensorDistance() > 6.1))
                        ||
                        (timer.time() > 500 &&
                        (this.robot.getRangeSensorDistance() <= 2.0 || this.robot.getRangeSensorDistance() > 7.1)))
                {
                    goToNextState();
                    logInfo("Too close or far from wall...");
                }
                beaconColor = null;

                break;

            case TURN_DEGREES:

                if (this.robot.getRangeSensorDistance() <= 3.5)
                {
                    if(this.robot.slowTurn(5.5)) {
                        madeAdjustment = true;
                        goToState(MOVE_TO_TAPE_2);
                        logInfo("Too close to wall...");
                    }
                }
                else if(this.robot.getRangeSensorDistance() > 6.1) {
                    if(this.robot.slowTurn(-5.5)) {
                        madeAdjustment = true;
                        goToState(MOVE_TO_TAPE_2);
                        logInfo("Too far from wall...");
                    }
                }
                else if(this.robot.getRangeSensorDistance() > 5.75) {
                    if(this.robot.slowTurn(-2.5)) {
                        madeAdjustment = true;
                        goToState(MOVE_TO_TAPE_2);
                        logInfo("Too far 2 from wall...");
                    }
                }
                else
                {
                    goToState(MOVE_TO_TAPE_2);
                }

                if(timer.time() > 3000) {
                    goToNextState();
                }

                beaconColor = null;
                break;

            case Turn_TO_0_DEGREES:
                if(enableColorSensorRegister) {
                    this.robot.setColorSensorFrontEnabled(false);
                    this.robot.setColorSensorBackEnabled(true);
                    this.robot.setColorSensorBeaconEnabled(false);
                }

                if(timer.time() > 100) {
                    if (this.robot.slowTurn(0.0)) {
                        goToNextState();
                    }
                }

                if(timer.time() > 2000) {
                    goToNextState();
                }

                beaconColor = null;
                break;

            case MOVE_TO_TAPE_2:

                double motorPower = -MOVE_TO_WHITE_TAPE_POWER;
//                if(this.robot.leftDriveMotor.getCurrentPosition() < betweenBeaconStartEncoderCount - distanceBetweenBeacons) {
//                    motorPower = -0.30;
//                }
                //move till the second white tape
                this.robot.setPowerUsingEncoders(motorPower, motorPower);

                if (madeAdjustment &&
                        this.robot.getRangeSensorDistance() > 3.5 &&
                        this.robot.getRangeSensorDistance() < 4.5) {
                    madeAdjustment = false;
                    goToState(Turn_TO_0_DEGREES);
                    break;
                }

                if(this.robot.getBeaconColorSensor() != ETC2016Robot.BeaconColor.UNKNOWN) {
                    beaconColor = this.robot.getBeaconColorSensor();
                }

                if ( this.robot.detectWhiteTape2()) {
                    beaconDetected = null;
                    goToNextState();
                }
                break;

            case Turn_TO_0_DEGREES_AGAIN:
                if(this.robot.slowTurn(0.0))
                {
                    goToNextState();
                }

                if(this.robot.getBeaconColorSensor() != ETC2016Robot.BeaconColor.UNKNOWN) {
                    beaconColor = this.robot.getBeaconColorSensor();
                }
                break;

            case PUSH_BEACON_BUTTON_2:
                if(enableColorSensorRegister) {
                    this.robot.setColorSensorFrontEnabled(false);
                    this.robot.setColorSensorBackEnabled(true);
                    this.robot.setColorSensorBeaconEnabled(true);
                }

                //pushButton the correct button on the beacon box
                if(beaconColor == ETC2016Robot.BeaconColor.BLUE ||
                        this.robot.getBeaconColorSensor() == ETC2016Robot.BeaconColor.BLUE)
                {
                    this.robot.slowTurn(-1.0);
                    if (robot.buttonPusher.pushButton() || timer.time() > 6000) {
                        beaconDetected = null;
                        firstButtonPushed = true;
                        logInfo("First Blue Beacon Pushed");
                        goToState(MOVE_TO_SECOND_BUTTON_2);
                    }
                }
                else if(beaconColor == ETC2016Robot.BeaconColor.RED ||
                        this.robot.getBeaconColorSensor() == ETC2016Robot.BeaconColor.RED)
                {
                    logInfo("First Beacon is Red, move to second Beacon");
                    beaconDetected = null;
                    firstButtonPushed = false;

                    goToNextState();
                }
                else {
                    if(elapsedTime.time() > 24000) {
                        isDefensiveAllowed = false;
                    }
                    goToState(TURN_45_DEGREES_TO_CAPBALL);
                }
                break;

            case MOVE_TO_SECOND_BUTTON_2:
                this.robot.setPowerUsingEncoders(-MOVE_TO_WHITE_TAPE_POWER, -MOVE_TO_WHITE_TAPE_POWER);
                if(this.robot.detectWhiteTape()){
                //if(this.robot.setPowerRunToPosition(-0.2,-0.2, (int)(-COUNTS_PER_INCH*5.0)) ||
                //        this.robot.detectWhiteTape()) {
                    if(firstButtonPushed) {
                        if(elapsedTime.time() > 24000) {
                            isDefensiveAllowed = false;
                        }
                        goToState(TURN_45_DEGREES_TO_CAPBALL);
                    } else {
                        goToNextState();
                    }
                }

                if(timer.time() > 5000)
                {
                    logInfo("Couldn't reach second Blue beacon button.");
                    if(elapsedTime.time() > 24000) {
                        isDefensiveAllowed = false;
                    }
                    goToState(TURN_45_DEGREES_TO_CAPBALL);
                }
                break;

            case PUSH_OTHER_BUTTON_2:
                if(beaconDetected == null)
                {
                    beaconDetected = true;
                    timer.reset();
                }

                if(this.robot.slowTurn(-0.5) || timer.time() > 500) {
                    if (!this.robot.detectWhiteTape2() &&
                        this.robot.getBeaconColorSensor() == ETC2016Robot.BeaconColor.BLUE) {
                        if (robot.buttonPusher.pushButton() || timer.time() > 6000) {
                            logInfo("Second blue Beacon pushed");
                            goToNextState();
                        }
                    } else {
                        this.robot.setPowerUsingEncoders(MOVE_TO_WHITE_TAPE_POWER, MOVE_TO_WHITE_TAPE_POWER);
                        beaconColor = this.robot.getBeaconColorSensor();
                        if (this.robot.detectWhiteTape2()) {
                            goToState(PUSH_BEACON_BUTTON_2);
                        }
                    }
                }

                break;
            case TURN_45_DEGREES_TO_CAPBALL:
                robot.buttonPusher.reset();

                if(timer.time() > 200) {
                    if(!isDefensivePlay() || !this.isDefensiveAllowed) {
                        if(!parkAtCenter()) {
                            goToState(END);
                            break;
                        }
                        robot.setPower(0.5*SPEED_GEAR_RATIO_FACTOR, 0.0);
                        if (robot.hasTargetAngleReached(-41.5, -1)) {
                            goToNextState();
                        }
                    } else {
                        if(elapsedTime.time() < 26000 && !Boolean.FALSE.equals(defenseTargetReached)) {
                            if(defenseTargetReached == null) {
                                if (robot.hasTargetAngleReached(-82.0, -1)) {
                                    resetState();
                                    defenseTargetReached = true;
                                } else {
                                    robot.setPower(0.75, 0.0);
                                }
                            } else if(defenseTargetReached){
                                if(robot.setPowerRunToPosition(0.5, 0.5, (int) (COUNTS_PER_INCH * 18.0))) {
                                    resetState();
                                    defenseTargetReached = false;
                                }
                            }
                        }
                        else if(elapsedTime.time() > 26000){
                            if(!parkAtCenter()) {
                                goToState(END);
                                break;
                            }

                            if (robot.mediumTurn(-22.5) || robot.hasTargetAngleReached(-25.0, 1)) {
                                goToNextState();
                            }
                        }
                    }
                } else if(elapsedTime.time() > 24000){
                    this.isDefensiveAllowed = false;
                }

                break;

            case MOVE_BACK_TO_HIT_CAPBALL:
                this.robot.collectorMotor.setPower(-1.0);
                double runDistance = isDefensivePlay()?38.0:49.0;
                if (this.robot.setPowerRunToPosition(0.75,
                        0.75, (int)(COUNTS_PER_INCH*runDistance))) {
                    goToNextState();
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