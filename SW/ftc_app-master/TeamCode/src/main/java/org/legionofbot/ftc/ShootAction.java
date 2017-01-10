package org.legionofbot.ftc;

/**
 * Created by uma on 12/26/2016.
 */

public class ShootAction implements IRobotAction {
    RobotActionModes robotActionMode = RobotActionModes.Shoot;
    public String Direction;
    public double Speed ;
    double milliSeconds;

    public ShootAction(String direction, double motorSpeed, double milliSeconds)
    {
        this.robotActionMode = RobotActionModes.Shoot;
        this.Speed = motorSpeed;
        this.milliSeconds = milliSeconds;
    }
}
