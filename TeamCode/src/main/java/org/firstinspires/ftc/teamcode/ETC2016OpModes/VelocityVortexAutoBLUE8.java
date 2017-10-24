package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by FTC 9915
 */
@Autonomous(name="Auto Blue 1 Defense No Park", group="9915 Blue")
@Disabled
public class VelocityVortexAutoBLUE8 extends VelocityVortexAutoBLUE {

    public VelocityVortexAutoBLUE8() {
    }

    @Override
    protected boolean isDefensivePlay() {
        return true;
    }

    @Override
    protected boolean parkAtCenter() {return false;}
}

//**************************************************************************************************