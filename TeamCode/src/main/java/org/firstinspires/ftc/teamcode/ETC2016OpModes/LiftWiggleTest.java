package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FTC 9915 on 1/16/2017.
 */

@Autonomous (name="Lift Wiggle Test", group="9915 Test")
@Disabled

public class LiftWiggleTest extends ETC2016AutoBaseOpMode {
    protected final static int Turn_40_degrees_slow      = 1;  //|wiggle|
    protected final static int Turn_other_way_slow       = 2;  //|      |
    protected final static int Drive_forward_slow        = 3;  //|forward backward test|
    protected final static int Drive_back_slow           = 4;  //|                     |
    protected final static int Turn_40_degrees_medium    = 5;  //|wiggle|
    protected final static int Turn_other_way_medium     = 6;  //|      |
    protected final static int Drive_forward_medium      = 7;  //|forward backward test|
    protected final static int Drive_back_medium         = 8;  //|                     |
    protected final static int Turn_40_degrees_fast      = 9;  //|wiggle|
    protected final static int Turn_other_way_fast       = 10; //|      |
    protected final static int Drive_forward_fast        = 11; //|forward backward test|
    protected final static int Drive_back_fast           = 12; //|                     |

    private final static int END = 100;


    public LiftWiggleTest() {
    }

    @Override
    public void init_loop() {
        initGyro();
    }

    @Override
    public void start() {
        super.start();

        robot.collectorMotor.setPower(0.0);

        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Log.i("LiftWiggleTest", " === Lift Test Started ===");

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

                goToNextState();
                break;

            case Turn_40_degrees_slow:
                if(this.robot.slowTurn(40.0)) {
                    goToNextState();
                }
                break;

            case Turn_other_way_slow:
                if(this.robot.slowTurn(-40.0)) {
                    goToNextState();
                }
                break;

            case Drive_forward_slow:
                if(this.robot.setPowerRunToPosition(0.25, 0.25, (int)(COUNTS_PER_INCH*36.0))) {
                    goToNextState();
                }
                break;

            case Drive_back_slow:
                if(this.robot.setPowerRunToPosition(-0.25, -0.25, (int)(-COUNTS_PER_INCH*36.0))) {
                    goToNextState();
                }
                break;
            case Turn_40_degrees_medium:
                if(this.robot.mediumTurn(40.0)) {
                    goToNextState();
                }
                break;

            case Turn_other_way_medium:
                if(this.robot.mediumTurn(-40.0)) {
                    goToNextState();
                }
                break;

            case Drive_forward_medium:
                if(this.robot.setPowerRunToPosition(0.5, 0.5, (int)(COUNTS_PER_INCH*36.0))) {
                    goToNextState();
                }
                break;

            case Drive_back_medium:
                if(this.robot.setPowerRunToPosition(-0.5, -0.5, (int)(-COUNTS_PER_INCH*36.0))) {
                    goToNextState();
                }
                break;

            case Turn_40_degrees_fast:
                if(this.robot.fastTurn(40.0)) {
                    goToNextState();
                }
                break;

            case Turn_other_way_fast:
                if(this.robot.fastTurn(-40.0)) {
                    goToNextState();
                }
                break;

            case Drive_forward_fast:
                if(this.robot.setPowerRunToPosition(0.8, 0.8, (int)(COUNTS_PER_INCH*60.0))) {
                    goToNextState();
                }
                break;

            case Drive_back_fast:
                if(this.robot.setPowerRunToPosition(-0.8, -0.8, (int)(-COUNTS_PER_INCH*60.0))) {
                    goToNextState();
                }
                break;

            default:
                v_state = END;
                break;
        }
    }

    @Override
    public AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
}