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
@Autonomous(name="Auto Red 1", group="9915 Red")
@Disabled
public class VelocityVortexAutoRED extends ETC2016AutoBaseOpMode {

    //**********************************************************************************************
    //      State Machine
    protected final static int START = 0;
    //
    //**********************************************************************************************
    //      Run Forward, Turn, and Reach Tape
    protected final static int TURN_40_DEGREES         = 1;
    protected final static int MOVE_3_FEET             = 2;
    protected final static int SHOOT_1_BALL            = 3;
    protected final static int SHOOT_2_BALL            = 4;
    protected final static int ADJUST_ANGLE            = 5;
    protected final static int MOVE_1_FOOT             = 6;
    protected final static int TURN_TO_20_DEGREES      = 7;
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
    private Boolean beaconDetected = null;
    private int counter = 0;
    private Boolean whiteLineDetected = null;
    private boolean firstButtonPushed = false;

    private Boolean defenseTargetReached = null;

    public VelocityVortexAutoRED() {
    }

    @Override
    public void init_loop() {
        super.init_loop();
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

                driveMotorPower = (timer.time() < 300?0.20:0.70) * SPEED_GEAR_RATIO_FACTOR;
                if (this.robot.setPowerRunToPosition(driveMotorPower, driveMotorPower, (int)(COUNTS_PER_INCH*36.0))) {
                    goToNextState();
                }
                break;

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
                    if (counter == 1 && getNumOfParticles() == 2) {
                        this.robot.ballShooter.fire();
                        logInfo("Auto Fire 2: " +
                                     this.robot.ballShooter.getShooterState());
                        counter++;
                    }
                    else {
                        goToNextState();
                        break;
                    }
                }
                this.robot.ballShooter.fire_loop();

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
                if(this.robot.slowTurn(40.5) || timer.time() > 1500)
                {
                    goToNextState();
                }

                break;

            case MOVE_1_FOOT:

                if(enableColorSensorRegister) {
                    this.robot.setColorSensorFrontEnabled(true);
                    this.robot.setColorSensorBackEnabled(false);
                    this.robot.setColorSensorBeaconEnabled(false);
                }
                this.robot.ballShooter.fire_loop();

                if (this.robot.detectWhiteTapeAny())
                {
                    whiteLineDetected = true;
                }

                driveMotorPower = (timer.time() < 200?0.20:0.40) * SPEED_GEAR_RATIO_FACTOR;
                if(this.robot.setPowerRunToPosition(driveMotorPower, driveMotorPower, (int)(COUNTS_PER_INCH*26.5))) {
                    goToNextState();
                }
                break;

            case TURN_TO_20_DEGREES:
                //this.robot.ballShooter.force_stop();
                if(timer.time() < 100) {
                    break;
                }

                if (this.robot.detectWhiteTapeAny())
                {
                    whiteLineDetected = true;
                }

                double turnAngle =  Range.clip(27.0 + (40.0- robot.stateBeginHeadingAngle),20.0, 30.0);

                if (this.robot.mediumTurn(turnAngle)) {
                    //this.robot.ballShooter.force_stop();
                    goToNextState();
                }
                break;

            case MOVE_TO_TAPE:
                //move till white tape
                if (this.robot.detectWhiteTapeAny())
                {
                    whiteLineDetected = true;
                }
                this.robot.setPowerUsingEncoders(MOVE_TO_WHITE_TAPE_POWER, MOVE_TO_WHITE_TAPE_POWER);
                if (this.robot.getRangeSensorDistance() < 5.7) {
                   goToNextState();
                }

                this.robot.collectorMotor.setPower(-1.0);
                break;

            case TURN_PARALLEL_TO_BEACON:
                this.robot.setPowerUsingEncoders(0.10*SPEED_GEAR_RATIO_FACTOR, -0.10*SPEED_GEAR_RATIO_FACTOR);
                if(this.robot.hasTargetAngleReached(1.5, -1)){
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
                    this.robot.setPowerUsingEncoders(-MOVE_TO_WHITE_TAPE_POWER, -MOVE_TO_WHITE_TAPE_POWER);
                    if (this.robot.getBeaconColorSensor() != ETC2016Robot.BeaconColor.UNKNOWN) {
                        beaconColor = this.robot.getBeaconColorSensor();
                    }
                    if ((this.robot.detectWhiteTape2() &&
                            beaconColor == ETC2016Robot.BeaconColor.RED)) {
                        firstButtonPushed = true;
                        goToNextState();
                    }
                    else if (this.robot.detectWhiteTape()) {
                        firstButtonPushed = false;
                        goToNextState();
                    }
                }
                else {
                    this.robot.setPowerUsingEncoders(MOVE_TO_WHITE_TAPE_POWER, MOVE_TO_WHITE_TAPE_POWER);
                    if (this.robot.getBeaconColorSensor() != ETC2016Robot.BeaconColor.UNKNOWN) {
                        beaconColor = this.robot.getBeaconColorSensor();
                    }
                    if( (this.robot.detectWhiteTape() &&
                            beaconColor == ETC2016Robot.BeaconColor.RED)) {
                        firstButtonPushed = false;
                        goToNextState();
                    } else if(this.robot.detectWhiteTape2()){
                        beaconColor = this.robot.getBeaconColorSensor();
                        firstButtonPushed = true;
                        goToNextState();
                    } else if(timer.time() > 1000) {
                        whiteLineDetected = true;
                    }
                }

                this.robot.collectorMotor.setPower(0.0);
                break;

            case PUSH_BEACON_BUTTON:
                //pushButton the correct button on the beacon box
                if( ((firstButtonPushed && beaconColor == ETC2016Robot.BeaconColor.RED) ||
                        this.robot.getBeaconColorSensor() == ETC2016Robot.BeaconColor.RED) &&
                       this.robot.getBeaconColorSensor() != ETC2016Robot.BeaconColor.BLUE)
                {
                    if(beaconDetected == null)
                    {
                        timer.reset();
                        beaconDetected = true;
                    }

                    this.robot.slowTurn(1.0);
                    if(robot.buttonPusher.pushButton() || timer.time() > 3000) {
                        beaconDetected = null;
                        if(firstButtonPushed) {
                            logInfo("First Red Button Pushed ... ");
                        } else {
                            logInfo("Second Red Button Pushed ... ");
                        }
                        goToNextState();
                    }
                }
                else if (this.robot.getBeaconColorSensor() == ETC2016Robot.BeaconColor.BLUE)
                {
                    logInfo("Red Auto: Blue Detected");
					if(this.robot.detectWhiteTape()) {
					    firstButtonPushed = true;
					} 
                    beaconDetected = null;
                    goToState(MOVE_TO_SECOND_BUTTON);
                }
                else
                {
                    logInfo("No beacon detected");

                    this.robot.setPowerUsingEncoders(-0.1, -0.1);
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
                if(this.robot.setPowerRunToPosition(
                        MOVE_TO_WHITE_TAPE_POWER,MOVE_TO_WHITE_TAPE_POWER, (int)(COUNTS_PER_INCH*5.5)) ||
                        this.robot.detectWhiteTape2()){
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
                }

                if(this.robot.getBeaconColorSensor() == ETC2016Robot.BeaconColor.BLUE) {
                    whiteLineDetected = true;
                    goToState(MOVE_BACKWARD_TO_BEACON);
                    break;
                }

                this.robot.slowTurn(1.0);
                if(this.robot.buttonPusher.pushButton() || timer.time() > 5000)
                {
                    beaconDetected = null;
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

                if(timer.time() >100 && this.robot.slowTurn(1.0)) {
                    goToNextState();
                    betweenBeaconStartEncoderCount = this.robot.leftDriveMotor.getCurrentPosition();
                }
                break;

            case MOVE_BETWEEN_BEACONS:
                this.robot.collectorMotor.setPower(-1.0);
                int distance = firstButtonPushed?(int) (COUNTS_PER_INCH * 36.0):
                                                    (int) (COUNTS_PER_INCH * 39.0);

                distanceBetweenBeacons = distance;

                driveMotorPower = (timer.time() < 300?0.20:0.55) * SPEED_GEAR_RATIO_FACTOR;

                if (this.robot.setPowerRunToPosition(driveMotorPower, driveMotorPower, distance)) {
                    goToNextState();
                }
                else if ( (timer.time() > 1000 &&
                        (this.robot.getRangeSensorDistance() <= 3.5 || this.robot.getRangeSensorDistance() > 6.1))
                        ||
                        (timer.time() > 500 &&
                        (this.robot.getRangeSensorDistance() <= 2.0 || this.robot.getRangeSensorDistance() > 7.1))
                        ) {
                    goToNextState();
                    logInfo("Too close or far from wall...");
                }
                beaconColor = null;

                break;

            case TURN_DEGREES:

                if (this.robot.getRangeSensorDistance() <= 3.5)
                {
                    if(this.robot.slowTurn(-5.5)) {
                        madeAdjustment = true;
                        goToState(MOVE_TO_TAPE_2);
                        logInfo("Too close to wall...");
                    }
                }
                else if(this.robot.getRangeSensorDistance() > 6.1) {
                    if(this.robot.slowTurn(5.5)) {
                        madeAdjustment = true;
                        goToState(MOVE_TO_TAPE_2);
                        logInfo("Too far from wall...");
                    }
                }
                else if(this.robot.getRangeSensorDistance() > 5.75) {
                    if(this.robot.slowTurn(2.5)) {
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
                break;

            case Turn_TO_0_DEGREES:

                if(enableColorSensorRegister) {
                    this.robot.setColorSensorFrontEnabled(true);
                    this.robot.setColorSensorBackEnabled(false);
                    this.robot.setColorSensorBeaconEnabled(true);
                }
                if(timer.time() > 100) {
                    if (this.robot.slowTurn(0.5)) {
                        goToNextState();
                    }
                }
                break;

            case MOVE_TO_TAPE_2:

                double motorPower = MOVE_TO_WHITE_TAPE_POWER;
//                if(this.robot.getLeftMotorPosition() < betweenBeaconStartEncoderCount + distanceBetweenBeacons) {
//                    motorPower = 0.30;
//                }

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

                if (this.robot.detectWhiteTape()) {
                    beaconDetected = null;
                    firstButtonPushed = false;
                    goToNextState();
                }
                break;

            case Turn_TO_0_DEGREES_AGAIN:
                if(this.robot.slowTurn(1.5))
                {
                    goToNextState();
                }

                if(this.robot.getBeaconColorSensor() != ETC2016Robot.BeaconColor.UNKNOWN) {
                    beaconColor = this.robot.getBeaconColorSensor();
                }

                break;

            case PUSH_BEACON_BUTTON_2:
                //pushButton the correct button on the beacon box
                if(beaconColor == ETC2016Robot.BeaconColor.RED ||
                    this.robot.getBeaconColorSensor() == ETC2016Robot.BeaconColor.RED)
                {
                    if(beaconDetected == null)
                    {
                        timer.reset();
                        beaconDetected = true;
                    }

                    this.robot.slowTurn(1.0);
                    if (robot.buttonPusher.pushButton() || timer.time() > 5000) {
                        beaconDetected = null;
                        logInfo("First Red Beacon pushed");
                        firstButtonPushed = true;
                        goToState(MOVE_TO_SECOND_BUTTON_2);
                    }
                }
                else if(beaconColor == ETC2016Robot.BeaconColor.BLUE ||
                        this.robot.getBeaconColorSensor() == ETC2016Robot.BeaconColor.BLUE)
                {
                    logInfo("First Beacon is Blue, move to second Beacon");
                    beaconDetected = null;
                    firstButtonPushed = false;
                    goToNextState();
                }
                else {
                    if(elapsedTime.time() > 24000) {
                        isDefensiveAllowed = false;
                    }
                    goToState(TURN_45_DEGREES_TO_CAPBALL);;
                }
                break;

            case MOVE_TO_SECOND_BUTTON_2:
                this.robot.setPowerUsingEncoders(MOVE_TO_WHITE_TAPE_POWER, MOVE_TO_WHITE_TAPE_POWER);
                if(this.robot.detectWhiteTape2()){
                        //this.robot.setPowerRunToPosition(0.2,0.2, (int)(COUNTS_PER_INCH*5.0))) {
                    if(firstButtonPushed) {
                        if(elapsedTime.time() > 24000) {
                            isDefensiveAllowed = false;
                        }
                        goToState(TURN_45_DEGREES_TO_CAPBALL);;
                    } else {
                        goToNextState();
                    }
                }

                if(timer.time() > 5000)
                {
                    logInfo("Couldn't reach second Red beacon button.");
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

                if(this.robot.slowTurn(0.0) || timer.time() > 500 ) {
                    if (!this.robot.detectWhiteTape() &&
                        this.robot.getBeaconColorSensor() == ETC2016Robot.BeaconColor.RED) {
                        if (robot.buttonPusher.pushButton() || timer.time() > 6000) {
                            logInfo("Second Red Beacon pushed");
                            goToNextState();
                        }
                    } else {
                        this.robot.setPowerUsingEncoders(-MOVE_TO_WHITE_TAPE_POWER, -MOVE_TO_WHITE_TAPE_POWER);
                        if (this.robot.detectWhiteTape()) {
                            goToState(PUSH_BEACON_BUTTON_2);
                        }
                    }
                }
                break;

            case TURN_45_DEGREES_TO_CAPBALL:
                robot.buttonPusher.reset();
                this.robot.collectorMotor.setPower(0.0);

                if(timer.time() > 200) {
                    if(!isDefensivePlay() || !this.isDefensiveAllowed) {
                        if(!parkAtCenter()) {
                            goToState(END);
                            break;
                        }

                        robot.setPower(-0.5*SPEED_GEAR_RATIO_FACTOR, 0.0);
                        if (robot.hasTargetAngleReached(40.5, 1)) {
                            goToNextState();
                        }
                    } else {
                        if(elapsedTime.time() < 26000 && !Boolean.FALSE.equals(defenseTargetReached)) {
                            if(defenseTargetReached == null) {
                                if (robot.hasTargetAngleReached(95.0, 1)) {
                                    resetState();
                                    defenseTargetReached = true;
                                } else {
                                    robot.setPower(-0.75, 0.0);
                                }
                            } else if(defenseTargetReached){
                                if(robot.setPowerRunToPosition(-0.5, -0.5, (int) (-COUNTS_PER_INCH * 18.0))) {
                                    logInfo("Move back on defense ...");
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

                            if (robot.fastTurn(27.0) || robot.hasTargetAngleReached(30.0, -1)) {
                                goToNextState();
                                break;
                            }
                        }
                    }
                } else if(elapsedTime.time() > 24000){
                    this.isDefensiveAllowed = false;
                }
                break;

            case MOVE_BACK_TO_HIT_CAPBALL:

                double runDistance = isDefensivePlay()?-38.0:-49.0;
                if (this.robot.setPowerRunToPosition(-0.75,
                        -0.75,
                        (int)(COUNTS_PER_INCH*runDistance))) {
                    goToNextState();
                }
                break;
            default:
                robot.buttonPusher.reset();
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