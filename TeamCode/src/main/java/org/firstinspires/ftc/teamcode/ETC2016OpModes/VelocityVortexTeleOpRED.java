package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Robotics 6 on 10/23/2016.
 */

@TeleOp(name="Velocity Vortex RED TeleOp", group="9915 TeleOp")
@Disabled
public class VelocityVortexTeleOpRED extends VelocityVortexTeleOp {
    @Override
    public AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
}

//**************************************************************************************************