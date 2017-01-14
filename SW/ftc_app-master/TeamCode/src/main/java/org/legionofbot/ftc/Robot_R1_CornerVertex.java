package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Robot_R1_CornerVertex", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class Robot_R1_CornerVertex extends Omnibot_OpMode_Auto {

    @Override
    public void registerActions() {

        double distance = 12;

        lstActions.add(new GoStraightRobotAction("West", motorSpeed, 50));
        lstActions.add(new GoStraightRobotAction("South", motorSpeed, 19));
        lstActions.add(new GoStraightRobotAction("East", motorSpeed, 30));
        lstActions.add(new ShootAction(motorSpeed, 4));
        lstActions.add(new GoStraightRobotAction("East", motorSpeed, 50));
    }
}
