package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Robot_B1_CornerVertex", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class Robot_B1_CornerVertex extends Omnibot_OpMode_Auto {

    @Override
    public void registerActions() {

        double distance = 12;

        lstActions.add(new GoStraightRobotAction("South", motorSpeed, 50));
        lstActions.add(new GoStraightRobotAction("West", motorSpeed, 9));
        lstActions.add(new ShootAction(motorSpeed, 12));
        lstActions.add(new SleepAction(2));
        lstActions.add(new GoStraightRobotAction("West", motorSpeed, 3));
        lstActions.add(new TurnRobotAction(turnSpeed, 10));
        lstActions.add(new GoStraightRobotAction("North", motorSpeed, 80));
     }
}

