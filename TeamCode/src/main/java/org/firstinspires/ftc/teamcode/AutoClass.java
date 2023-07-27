package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class AutoClass extends Teleop_class {

    //VARAIBLE ZONE!!!


    public double InitX, InitY;
    public double DForward, DSideways;
    public double correctionFactor = 1.016723060905778;
    public double ticksPerRevolution = 537.7;
    public double MMPerRevolution = 96*Math.PI;
    public double MMPerInch = 25.4;
    public double FrontLeft = 0;
    public double FrontRight=0;
    public double BackLeft=0;
    public double BackRight=0;
    public double FrontLeftOld,FrontRightOld,BackLeftOld,BackRightOld;
    public float TESTfLeft, TESTfRight, TESTbLeft, TESTbRight;


public double startOfsetRadians = 0;
    public void RobotAngles(){

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
    RobotAngle = orientation.getYaw(AngleUnit.RADIANS);
        RobotAngle += startOfsetRadians;
    }
    public void UpdateOdometry(){

        DForward = (FrontRight + FrontLeft + BackRight + BackLeft)/4;
        DSideways = (-FrontRight + FrontLeft + BackRight - BackLeft)/4/1.2;
        RobotX = (InitX - DForward * Math.sin(RobotAngle)+ DSideways * Math.cos(RobotAngle));
        RobotY = (InitY + DForward * Math.cos(RobotAngle)+ DSideways * Math.sin(RobotAngle));
        InitX = RobotX;
        InitY = RobotY;
    }
    public void keepAtPoint(double Tx, double Ty) {

        distanceX = RobotX - Tx;
        distanceY = RobotY - Ty;

        PowerX = -distanceX * .55;
        PowerY = -distanceY * .4;

        PowerS = PowerX * Math.cos(RobotAngle) - PowerY * Math.sin(RobotAngle);
        PowerF = PowerX * Math.sin(RobotAngle) + PowerY * Math.cos(RobotAngle);

        if (PowerF >= 1 ) PowerF = 1;
        if (PowerF <= -1) PowerF = -1;
        if (PowerS >= 1 ) PowerS = 1;
        if (PowerS <= -1) PowerS = -1;

        drive = PowerF;
        slide = PowerS;
        straferAlgorithm();


    }
    public void RunToPoint(double TargetX, double TargetY){
        while()
        {
            IMUstuffs();
            keepAtPoint(TargetX, TargetY);
            ProportionalFeedbackControl();
            UpdateEncoders();
            UpdateOdometry();
            straferAlgorithm();
            setMotorPower();


         }

    }

    public void UpdateEncoders(){
        FrontLeft = TESTfLeft;
        FrontRight = TESTfRight;
        BackLeft = TESTbLeft;
        BackRight = TESTbRight;

        FrontLeft -= FrontLeftOld;
        BackLeft -= BackLeftOld;
        FrontRight -= FrontRightOld;
        BackRight -= BackRightOld;

        FrontLeftOld = TESTfLeft;
        FrontRightOld = TESTfRight;
        BackLeftOld =  TESTbLeft;
        BackRightOld = TESTbRight;

        FrontLeft = FrontLeft/ticksPerRevolution;
        FrontLeft = MMPerRevolution*FrontLeft;
        FrontLeft = FrontLeft/MMPerInch;
        FrontLeft *= correctionFactor;

        FrontRight = FrontRight/ticksPerRevolution;
        FrontRight = MMPerRevolution*FrontRight;
        FrontRight = FrontRight/MMPerInch;
        FrontRight *= correctionFactor;

        BackRight = BackRight/ticksPerRevolution;
        BackRight = MMPerRevolution*BackRight;
        BackRight = BackRight/MMPerInch;
        BackRight *= correctionFactor;

        BackLeft = BackLeft/ticksPerRevolution;
        BackLeft = MMPerRevolution*BackLeft;
        BackLeft = BackLeft/MMPerInch;
        BackLeft *= correctionFactor;





        telemetry.addData("left front inches: ", FrontLeft);
        telemetry.addData("right front inches: ", FrontRight);
        telemetry.addData("left back inches: ", BackLeft);
        telemetry.addData("right back inches: ", BackRight);

    }
}
