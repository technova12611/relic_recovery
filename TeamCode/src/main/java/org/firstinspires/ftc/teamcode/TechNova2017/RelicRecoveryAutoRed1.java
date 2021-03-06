package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto Red 1", group = "Competition")
@Disabled
public class RelicRecoveryAutoRed1 extends RelicRecoveryAutoBase2{

    public AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }

    protected double getRightColumnTargetDistanceInCM() {
        return 0.0;
    }

    protected double getCenterColumnTargetDistanceInCM() {
        return 0.0;
    }

    protected double geLeftColumnTargetDistanceInCM() {
        return 0.0;
    }

}
