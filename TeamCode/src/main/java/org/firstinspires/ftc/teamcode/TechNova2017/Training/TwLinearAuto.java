package org.firstinspires.ftc.teamcode.TechNova2017.Training;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Terry Auto", group="Training Auto")
@Disabled
public class TwLinearAuto extends LinearOpMode {

    //      State Machine
    protected final static int START = 0;

    protected final static int FORWARD_3_FEET          = 1;
    protected final static int TURN_TO_40_DEGREE       = 2;
    protected final static int FORWARD_1_FEET          = 3;
    protected final static int TURN_TO_0_DEGREE        = 4;

    protected final static int END = 100;

    int v_state = START;

    Robot robot = null;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            switch (v_state) {
                case START:
                    break;

                case FORWARD_3_FEET:
                    if(robot.encoderDrive(0.75,36.0)) {
                        gotoNextState();
                    }
                    break;

                case TURN_TO_40_DEGREE:
                    if(robot.mediumTurn(40.0) || timer.time() > 3000) {
                        gotoNextState();
                    }
                    break;

                case FORWARD_1_FEET:
                    if(robot.encoderDrive(0.25,12.0)) {
                        gotoNextState();
                    }
                    break;

                case TURN_TO_0_DEGREE:
                    if(robot.slowTurn(0.0)) {
                        gotoNextState();
                    }
                    break;

                default:
                    robot.stop();
                    break;
            }
        }

        robot.stop();
    }

    protected void gotoNextState() {
        v_state++;
        timer.reset();
        robot.stop();
    }

    protected void gotoState(int state) {
        v_state = state;
        robot.stop();
    }
}
