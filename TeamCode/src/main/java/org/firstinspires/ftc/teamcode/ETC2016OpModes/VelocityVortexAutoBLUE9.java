package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by FTC 9915
 */
@Autonomous(name="Auto Blue 2 Alliance No Park", group="9915 Blue")
@Disabled
public class VelocityVortexAutoBLUE9 extends VelocityVortexAutoBLUE4 {

    public VelocityVortexAutoBLUE9() {
    }

    @Override
    protected boolean parkAtCenter() {return false;}
}

//**************************************************************************************************