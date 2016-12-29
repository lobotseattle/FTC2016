package org.firstinspires.ftc.teamcode.teamcode;

/**
 * Created by uma on 12/26/2016.
 */

public class TurnRobotAction implements IRobotAction {
    RobotActionModes robotActionMode = RobotActionModes.Turn;
    public double MotorSpeed ;
    public int Degrees;

    public TurnRobotAction(double turnSpeed, int degrees)
    {
        this.robotActionMode = RobotActionModes.Turn;
        this.MotorSpeed = turnSpeed;
        this.Degrees = degrees;
    }
}
