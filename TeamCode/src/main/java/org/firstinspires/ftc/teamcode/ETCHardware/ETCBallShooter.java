package org.firstinspires.ftc.teamcode.ETCHardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.ETCHardware.ETCConstants.SHOOTER_CPR;

/**
 * Created by FTC 9915 on 10/18/2016.
 */

public class ETCBallShooter {

    private DcMotor shooterMotor;
    private DcMotor tensionMotor;

    private TouchSensor shooterSwitch;
    private TouchSensor tensionSwitch;

    private Servo stopperServo;

    private static final int SHOOTER_STATE_FIRE = 0;
    private static final int SHOOTER_STATE_STOP = 1;
    private static final int SHOOTER_STATE_WAIT = 2;
    private static final int SHOOTER_STATE_RELOAD = 3;
    private static final int SHOOTER_STATE_MANUAL = 4;
    private static final int SHOOTER_STATE_MANUAL_RELOAD = 5;

    int shooterState = SHOOTER_STATE_STOP;
    Telemetry telemetry         = null;

    ElapsedTime shooterTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime stopperTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime reloadTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime tensionTimer = null;

    ElapsedTime gameTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private int lastShooterPosition = 0;
    private int initialShoorterPosition = 0;
    private int initialTensionPosition = 0;
    private int lastTensionPosition = 0;
    private int lastShooterError = 0;

    private boolean tensionInitialized = false;

    private boolean autoOpen = false;

    private boolean autonomous = false;

    private int numOfShots = 0;

    public int initialTension = ETCConstants.AUTONOMOUS_TENSION_POSITION;

    double tensionTime = 0;

    boolean justInitilaized = false;

    double prevShooterTime = 0.0;
    double cycleTime = 0.0;
    double prevCycleTime = 0.0;

    boolean manualStopper = false;
    int numOfCycles = 0;

    boolean touchPressed = false;
    boolean overShoot = false;
    int shooterAdjustment = 0;

    public ETCBallShooter(DcMotor shooterM,
                          DcMotor tensionM,
                          TouchSensor shooterS,
                          TouchSensor tensionS,
                          Servo stopperS,
                          Telemetry teleM) {
        this.shooterMotor = shooterM;
        this.tensionMotor = tensionM;
        this.shooterSwitch = shooterS;
        this.tensionSwitch = tensionS;
        this.stopperServo = stopperS;
        this.telemetry = teleM;
    }

    public void init() {
        this.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.tensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterState = SHOOTER_STATE_RELOAD;
        this.stopperServo.setPosition(ETCConstants.STOPPER_CLOSE_POSITION);
        shooterTimer.reset();
        gameTimer.reset();
    }

    public void start() {
        shooterTimer.reset();
        gameTimer.reset();
    }

    public boolean init_loop(boolean isAutonomous, boolean isRed) {

        autonomous = isAutonomous;

        if(shooterTimer.time() < 20) {
            logInfo("Shooter starts initialization ... ");
        }

        if(shooterTimer.time() > 100 && shooterState == SHOOTER_STATE_RELOAD) {
            int currentPosition = this.shooterMotor.getCurrentPosition();
            if (!this.shooterSwitch.isPressed()) {
                int error = (int)(SHOOTER_CPR*0.8) - Math.abs(currentPosition);
                this.shooterMotor.setPower(-1*Range.clip(0.0006*error, 0.08, 0.20));
            } else {
                shooterState = SHOOTER_STATE_STOP;
                this.shooterMotor.setPower(0.0);

                touchPressed = true;
                justInitilaized = true;
                initialShoorterPosition = currentPosition;
                lastShooterPosition = currentPosition;
                logInfo("Shooter stopped at:" + currentPosition );
            }
        }

        if(shooterTimer.time() > 500 && this.tensionMotor != null && !tensionInitialized) {
            if(shooterTimer.time() > 10000 ) {
                stopTensionMotor();
                tensionInitialized = false;
            }
            else {
                initialTensionPosition = this.tensionMotor.getCurrentPosition();
                if (this.shooterSwitch.isPressed() && !this.tensionSwitch.isPressed()) {
                    this.tensionMotor.setPower(0.40);
                }

                if (this.tensionSwitch.isPressed()) {
                    this.tensionMotor.setPower(0.0);
                    lastTensionPosition = getTensionPosition();
                    tensionInitialized = true;
                    tensionTime = shooterTimer.time();
                    //logInfo("Tension Initialized: " + lastTensionPosition);
                }
            }
        }

        if(this.tensionMotor == null) {
            logInfo("Tension Motor is null ");
        }

        if(tensionInitialized) {
            if(isAutonomous) {
                if(isRed) {
                    if(this.moveTensionToAutonomousPosition()) {
                        stopTensionMotor();
                        //logInfo("Autonomous tension: " + getTensionPosition());
                    }
                } else {
                    if(this.moveTensionToBlueAutonomousPosition()) {
                        stopTensionMotor();
                        //logInfo("Autonomous tension: " + getTensionPosition());
                    }
                }
            } else {
                if(this.moveTensionToMediumPosition()) {
                    stopTensionMotor();
                    //logInfo("TeleOps tension: " + getTensionPosition());
                }
            }

            if((shooterTimer.time() - tensionTime) > 3000) {
                stopTensionMotor();
            }
        }

        if(!isAutonomous) {
            this.stopperServo.setPosition(ETCConstants.STOPPER_OPEN_POSITION);
        }

        if(shooterState == SHOOTER_STATE_STOP) {
            lastShooterPosition = this.shooterMotor.getCurrentPosition();
        }

        lastShooterError = 0;
        return this.shooterSwitch.isPressed() && tensionInitialized;
    }

    public void fire_loop() {
        if(this.shooterMotor != null) {
            // when gamepad2.a button is pressed
            // move the choo choo to shoot
            // if the touch sensor is not pressed, it means choo choo is firing
            // so don't move motor any more
            if (shooterState == SHOOTER_STATE_FIRE) {
                //this.shooterMotor.setPower(-0.95);
                this.stopperServo.setPosition(ETCConstants.STOPPER_CLOSE_POSITION);
                shooterState = SHOOTER_STATE_RELOAD;

                if(!this.shooterSwitch.isPressed()) {
                    logInfo("Sensor not touched !!!");
                    shooterAdjustment = 15;
                }
            }

            if (shooterState == SHOOTER_STATE_RELOAD) {
                numOfCycles++;
                cycleTime = (shooterTimer.time()/numOfCycles);

                this.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                int targetPosition = ETCConstants.SHOOTER_CPR + lastShooterError + shooterAdjustment;
                if(justInitilaized) {
                    targetPosition = ETCConstants.SHOOTER_CPR - 20;
                    justInitilaized = false;
                }

                int error = targetPosition -
                        Math.abs(this.shooterMotor.getCurrentPosition()-lastShooterPosition);

                boolean slowMode = (shooterTimer.time()/(numOfCycles+1)) > 50.0;
                double pcoef = error > 400?0.0020: (slowMode?0.0005:0.0018);
                double power = -1*Range.clip(pcoef*error, slowMode?0.12:0.20, 0.95);

                this.shooterMotor.setPower(power);

                int threshold = 5;
                if(slowMode) {
                    threshold = 15;
                }

                if(!autonomous && false) {
                    logInfo("Shooter Auto: | " + String.format("%.1f", shooterTimer.time())
                            + " | " + String.format("%.1f", cycleTime)
                            + " | " + String.format("%.1f", (shooterTimer.time() - prevCycleTime))
                            + " | " + targetPosition
                            + " | " + Math.abs((this.shooterMotor.getCurrentPosition() - lastShooterPosition))
                            + " | " + error
                            + " | " + lastShooterError
                            + " | " + String.format("%.2f", power)
                            + " | " + this.shooterSwitch.isPressed()
                    );
                }

                prevCycleTime = shooterTimer.time();
                if( error < threshold ||
                        (shooterTimer.time() > 700 && this.shooterSwitch.isPressed())) {
                    shooterState = SHOOTER_STATE_STOP;
                    reloadTimer.reset();
                    stopperTimer.reset();
                    touchPressed = this.shooterSwitch.isPressed();

                    if(touchPressed) {
                        lastShooterError = (error < 0 && error > -100)? (int)(error/1.2): 0;
                        if(error > 5 && error < 50) {
                            lastShooterError = -(int)(error/1.2);
                        }
                        lastShooterError = Range.clip(lastShooterError, -50, -20);
                    } else {
                        lastShooterError = 0;
                    }

                    logInfo("Reload Complete: | " + String.format("%.1f",shooterTimer.time())
                            + " | " + String.format("%.1f",cycleTime)
                            + " | " + numOfCycles
                            + " | " + this.stopperServo.getPosition()
                            + " | " + targetPosition
                            + " | " + Math.abs((this.shooterMotor.getCurrentPosition()-lastShooterPosition))
                            + " | " + error
                            + " | " + lastShooterError
                            + " | " + shooterAdjustment
                            + " | " + this.shooterSwitch.isPressed()
                    );

                    shooterTimer.reset();
                }

                if(shooterTimer.time()> 500) {
                    this.stopperServo.setPosition(ETCConstants.STOPPER_OPEN_POSITION);
                }
            }
            // move the choo choo to stop position.
            // stop when touch is pressed
            else if (shooterState == SHOOTER_STATE_MANUAL_RELOAD) {

                numOfCycles++;
                cycleTime = (shooterTimer.time()/numOfCycles);

                this.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if(shooterTimer.time() < 500) {
                    if(overShoot) {
                        this.shooterMotor.setPower(-0.50);
                    }
                    else {
                        this.shooterMotor.setPower(-0.30);
                    }
                }
                else {
                    this.shooterMotor.setPower(-0.15);
                }

                if ( this.shooterSwitch.isPressed() ) {
                    reloadTimer.reset();
                    stopperTimer.reset();

                    overShoot = false;
                    shooterState = SHOOTER_STATE_STOP;

                    manualStopper = false;
                    touchPressed = true;

                    justInitilaized =true;
                    lastShooterError = 0;

                    logInfo("Manual Reload Complete: | " + String.format("%.1f",shooterTimer.time()) + " | " +
                            this.stopperServo.getPosition() + " | " + this.shooterMotor.getCurrentPosition()
                             + " | " + String.format("%.1f",cycleTime)
                             + " | " + this.shooterSwitch.isPressed() );

                    shooterTimer.reset();
                }

                if(shooterTimer.time()> 500) {
                    this.stopperServo.setPosition(ETCConstants.STOPPER_OPEN_POSITION);
                }
            }

            // stop the motor
            if(shooterState == SHOOTER_STATE_STOP) {
                this.shooterMotor.setPower(0.0);
                lastShooterPosition = this.shooterMotor.getCurrentPosition();

                if(!this.shooterSwitch.isPressed() && touchPressed ) {
                    if(shooterTimer.time() < 500) {
                        overShoot = true;
                        shooterState = SHOOTER_STATE_MANUAL_RELOAD;
                        logInfo("Overshoot: " + String.format("%.1f", shooterTimer.time()));
                    }
                    logInfo("Detect Overshoot: " + String.format("%.1f", shooterTimer.time()));
                }

                touchPressed = this.shooterSwitch.isPressed();
                shooterAdjustment = 0;

                if(isReadyToFire() && !manualStopper && reloadTimer.time() < 200) {
                    this.stopperServo.setPosition(ETCConstants.STOPPER_CLOSE_POSITION);
                }

                if(isReadyToFire() && !manualStopper && reloadTimer.time() > 5000) {
                    this.stopperServo.setPosition(ETCConstants.STOPPER_OPEN_POSITION);
                }
            }
        }
    }

    public void unstuck() {
        this.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooterMotor.setPower(0.05);
        shooterTimer.reset();
    }

    public void fire() {
        if (isReadyToFire()) {
            shooterState = SHOOTER_STATE_FIRE;
            numOfShots++;

            logInfo("Fire Started: " + Math.abs((this.shooterMotor.getCurrentPosition()-lastShooterPosition))
                    + " | " + String.format("%.1f",shooterTimer.time())
                    + " | tension:" + getTensionPosition() + " | Shots: " + getNumOfShots() +
                      " | shooter switch:" + this.isShooterSwitchPressed() +
                    " | " + String.format("%.1f",(gameTimer.time() - prevShooterTime)));

            shooterTimer.reset();
            numOfCycles = 0;
            overShoot = false;
            prevShooterTime = gameTimer.time();
            prevCycleTime = 0.0;
            lastShooterPosition = this.shooterMotor.getCurrentPosition();
        }
    }

    public int getNumOfShots() { return numOfShots;}

    public int getShooterState() {
        return shooterState;
    }

    public boolean isReloadingCompleted() {
        return shooterState == SHOOTER_STATE_RELOAD
                && shooterTimer.time()> 600
                && !shooterSwitch.isPressed();
    }

    public boolean isReadyToFire() {
//        logInfo("shooter_state:" + shooterState + " | pressed:" + this.shooterSwitch.isPressed() +
//           " |" +  stopperServo.getPosition() + " | timer: " + stopperTimer.time());

        return !manualStopper && shooterState == SHOOTER_STATE_STOP  &&
                stopperTimer.time() > (autonomous?350:100);
    }

    public void stop() {
        shooterState = SHOOTER_STATE_STOP;
        this.shooterMotor.setPower(0.0);
    }
    public void force_stop() {
        shooterState = SHOOTER_STATE_STOP;
        this.shooterMotor.setPower(0.0);
    }

    public void manualReload() {
        if(shooterState != SHOOTER_STATE_MANUAL_RELOAD) {
            shooterState = SHOOTER_STATE_MANUAL_RELOAD;
            prevShooterTime = 0.0;
            shooterTimer.reset();
            numOfCycles = 0;
            overShoot = false;
            logInfo("Manual reloading starts ...");
        }
    }

    public void adjustTension(double power) {
        if(this.tensionMotor != null) {
            this.tensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (this.tensionSwitch == null ||
                    (this.tensionSwitch != null && !this.tensionSwitch.isPressed())) {

                if(Math.abs(getTensionPosition()) <8000 || power>0) {
                    this.tensionMotor.setPower(power);
                }
            }
            else if( (this.tensionSwitch != null && this.tensionSwitch.isPressed() && power < 0)) {
                this.tensionMotor.setPower(power);
            }
            else {
                this.tensionMotor.setPower(0.0);
            }

            lastTensionPosition = getTensionPosition();
        }

        //logInfo("Adjust Tension:" + tensionMotor.getCurrentPosition() + " | Power: " + tensionMotor.getPower());
    }

    public void moveStopper(double power) {
        if(this.stopperServo != null) {
            if(power> 0.5) {
                this.stopperServo.setPosition(ETCConstants.STOPPER_CLOSE_POSITION);
                this.manualStopper = true;
                reloadTimer.reset();
                //logInfo("Open stopper !!!");
            } else if(power < -0.5) {
                this.stopperServo.setPosition(ETCConstants.STOPPER_OPEN_POSITION);
                this.manualStopper = true;
                //logInfo("Close stopper !!! ");
            } else {
                this.manualStopper = false;
            }
//            else if(isReadyToFire()) {
//                this.stopperServo.setPosition(ETCConstants.STOPPER_CLOSE_POSITION);
//                if(autoOpen) {
//                    autoOpen = false;
//                    logInfo("Close stopper: " + String.format("%.1f",stopperTimer.time()));
//                }
//            }
        }
    }

    public boolean moveTensionToAutonomousPosition() {
        return adjustTensionPosition(initialTension);
    }

    public boolean moveTensionToBlueAutonomousPosition() {
        return adjustTensionPosition(ETCConstants.BLUE_AUTONOMOUS_TENSION_POSITION);
    }

    public boolean moveTensionToLowPosition() {
        return adjustTensionPosition(ETCConstants.LOW_TENSION_POSITION);
    }

    public boolean moveTensionToMediumPosition() {
        return adjustTensionPosition(ETCConstants.MEDIUM_TENSION_POSITION);
    }

    public boolean moveTensionToHighPosition() {
        return adjustTensionPosition(ETCConstants.HIGH_TENSION_POSITION);
    }

    public boolean moveTensionToVeryHighPosition() {
        return adjustTensionPosition(ETCConstants.VERY_HIGH_TENSION_POSITION);
    }

    public boolean moveTensionToInitialPosition() {
        return adjustTensionPosition(initialTensionPosition);
    }

    public boolean moveTensionToZeroPosition() {
        return adjustTensionPosition(ETCConstants.ZERO_TENSION_POSITION);
    }

    public boolean adjustTensionPosition(int position) {
        if(tensionTimer == null) {
            tensionTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }
        this.tensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.tensionMotor.setTargetPosition(initialTensionPosition + position);
        this.tensionMotor.setPower(-0.75);

        if((!this.tensionMotor.isBusy() &&
                        getTensionPosition() != lastTensionPosition) ||
                tensionTimer.time() > 5000) {
            lastTensionPosition = getTensionPosition();
            //logInfo("tension motor stopped.");
            tensionTimer = null;
            return true;
        }

        return false;
    }

    public void stopTensionMotor() {
        this.tensionMotor.setPower(0.0);
        lastTensionPosition = getTensionPosition();
        if(this.tensionMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            this.tensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public int getTensionPosition() {
        return this.tensionMotor != null?
                (this.tensionMotor.getCurrentPosition() - initialTensionPosition):0;
    }

    public boolean isTensionSwitchPressed() {
        return this.tensionSwitch.isPressed();
    }

    public boolean isShooterSwitchPressed() {
        return this.shooterSwitch.isPressed();
    }

    protected void logInfo(String message) {
        Log.i(this.getClass().getSimpleName(), String.format("%.1f", gameTimer.time()) + ": " + message);
    }

    public void resetGameTimer() {
        gameTimer.reset();
    }

    public double getCycleTime() {
        return this.cycleTime;
    }

    public boolean isOverShoot() {
        return this.overShoot;
    }
}
