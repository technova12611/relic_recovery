package org.firstinspires.ftc.teamcode.ETCHardware;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

/**
 * Created by terryw on 12/19/2016.
 */

public class ETCDcMotor extends DcMotorImpl {

    protected double prevPower = 0.0;
    /**
     * Constructor
     *
     * @param controller DC motor controller this motor is attached to
     * @param portNumber portNumber position on the controller
     */
    public ETCDcMotor(DcMotorController controller, int portNumber) {
        this(controller, portNumber, Direction.FORWARD);
    }

    /**
     * Constructor
     *
     * @param controller DC motor controller this motor is attached to
     * @param portNumber portNumber port number on the controller
     * @param direction direction this motor should spin
     */
    public ETCDcMotor(DcMotorController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }

    @Override
    synchronized public void setPower(double power) {
        if(power != prevPower) {
            super.setPower(power);
            prevPower = power;
        }
    }
}
