package org.legionofbot.ftc;

/**
 * Created by uma on 12/29/2016.
 */

public class SleepAction implements IRobotAction {
    RobotActionModes robotActionMode = RobotActionModes.Sleep;
    double milliSeconds;

    public SleepAction(double milliSeconds)
    {
        this.milliSeconds = milliSeconds;
    }
}
