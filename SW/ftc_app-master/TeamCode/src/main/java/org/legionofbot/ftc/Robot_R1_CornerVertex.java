package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Robot_R1_CornerVertex", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class Robot_R1_CornerVertex extends Omnibot_OpMode_Auto {

    @Override
    public void registerActions() {

        double distance = 12;

        lstActions.add(new GoStraightRobotAction("West", motorSpeed, 50));
        lstActions.add(new GoStraightRobotAction("South", motorSpeed, 9));
        lstActions.add(new ShootAction(motorSpeed, 12));
        lstActions.add(new SleepAction(2));
        lstActions.add(new GoStraightRobotAction("South", motorSpeed, 3));
        lstActions.add(new GoStraightRobotAction("East", motorSpeed, 80));
    }
}
