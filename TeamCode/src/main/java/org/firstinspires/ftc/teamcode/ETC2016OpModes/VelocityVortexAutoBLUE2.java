package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by FTC 9915
 */
@Autonomous(name="Auto Blue 1 Defense", group="9915 Blue")
@Disabled
public class VelocityVortexAutoBLUE2 extends VelocityVortexAutoBLUE {

    public VelocityVortexAutoBLUE2() {
    }

    @Override
    protected boolean isDefensivePlay() {
        return true;
    }
}

//**************************************************************************************************