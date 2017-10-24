package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by FTC 9915
 */
@Autonomous(name="Auto Blue 1 One Particle Defense", group="9915 Blue")
@Disabled
public class VelocityVortexAutoBLUE7 extends VelocityVortexAutoBLUE {

    public VelocityVortexAutoBLUE7() {
    }

    @Override
    protected int getNumOfParticles() {
        return 1;
    }

    @Override
    protected boolean isDefensivePlay() {return true;}
}

//**************************************************************************************************