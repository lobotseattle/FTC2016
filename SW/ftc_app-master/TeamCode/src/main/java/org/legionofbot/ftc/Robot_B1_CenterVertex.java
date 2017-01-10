package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Robot_B1_CenterVertex", group="Linear Opmode")
public class
Robot_B1_CenterVertex extends Omnibot_OpMode_Auto {

    @Override
    public void registerActions() {
        lstActions.add(new GoStraightRobotAction("South", motorSpeed, 50));
        lstActions.add(new GoStraightRobotAction("West", motorSpeed, 10));
    }
}
