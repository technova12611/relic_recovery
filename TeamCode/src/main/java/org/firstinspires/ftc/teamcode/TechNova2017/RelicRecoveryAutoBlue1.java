package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Blue 1", group = "Competition")
public class RelicRecoveryAutoBlue1 extends RelicRecoveryAutoStrategyBase {

    public AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }

    protected double getRightColumnTargetDistanceInCM() {
        return 0.0;
    }

    protected double getCenterColumnTargetDistanceInCM() {
        return 0.0;
    }

    protected double geLeftColumnTargetDistanceInCM() {return 0.0;}

}
