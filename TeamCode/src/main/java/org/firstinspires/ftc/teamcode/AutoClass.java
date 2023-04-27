package org.firstinspires.ftc.teamcode;

public abstract class AutoClass extends Robot {
    //VARAIBLE ZONE!!!
    public double RobotAngle = 0;
    public double RobotX, RobotY;
    public double InitX, InitY;
    public double DForward, DSideways;

    public void UpdateOdometry(){
        InitX = RobotX;
        InitY = RobotY;
        RobotX = (InitX + DForward * Math.cos(RobotAngle))+ DSideways * Math.cos(RobotAngle + 90);
        RobotY = (InitY + DForward * Math.sin(RobotAngle))+ DSideways * Math.sin(RobotAngle + 90);
    }
}
