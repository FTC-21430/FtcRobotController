package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class AutoClass extends Teleop_class {
    //VARAIBLE ZONE!!!
    public double RobotAngle = 0;
    public double RobotX, RobotY;
    public double InitX, InitY;
    public double DForward, DSideways;
    public double rotV = 537.7;
    public double MMV = 96;
    public double InchV = 23.4;
    public double FrontLeft = 0;
    public double FrontRight=0;
    public double BackLeft=0;
    public double BackRight=0;
    public double FrontLeftOld,FrontRightOld,BackLeftOld,BackRightOld;
public float startOfsetDegrees = 0;
    public void RobotAngles(){

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        RobotAngle = orientation.getYaw(AngleUnit.DEGREES);
        RobotAngle += startOfsetDegrees;
    }
    public void UpdateOdometry(){
        InitX = RobotX;
        InitY = RobotY;
        DForward = (FrontRight + FrontLeft + BackRight + BackLeft)/4;
        DSideways = (FrontRight + FrontLeft + BackRight - BackLeft)/4;
        RobotX = (InitX + DForward * Math.cos(RobotAngle))+ DSideways * Math.cos(RobotAngle + 90);
        RobotY = (InitY + DForward * Math.sin(RobotAngle))+ DSideways * Math.sin(RobotAngle + 90);
    }
    public void UpdateEncoders(){
        FrontLeft = leftFrontMotor.getCurrentPosition();
        FrontRight = rightFrontMotor.getCurrentPosition();
        BackLeft =   leftBackMotor.getCurrentPosition();
        BackRight =   rightBackMotor.getCurrentPosition();

        FrontLeft -= FrontLeftOld;
        BackLeft -= BackLeftOld;
        FrontRight -= FrontRightOld;
        BackRight -= BackRightOld;

        FrontLeftOld = FrontLeft;
        FrontRightOld = FrontRight;
        BackLeftOld =  BackLeft;
        BackRightOld = BackRight;

        FrontLeft = FrontLeft/rotV;
        FrontLeft = FrontLeft/MMV;
        FrontLeft = FrontLeft/InchV;

        FrontRight = FrontRight/rotV;
        FrontRight = FrontRight/MMV;
        FrontRight = FrontRight/InchV;

        BackRight = BackRight/rotV;
        BackRight = BackRight/MMV;
        BackRight = FrontRight/InchV;

        BackLeft = BackLeft/rotV;
        BackLeft = BackLeft/MMV;
        BackLeft = BackLeft/InchV;
    }
}
