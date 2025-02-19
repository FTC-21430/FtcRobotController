package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Odometry_Test", group="summerRocks")
public class Odometry_Test extends AutoClass {

    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        waitForStart();
        runtime.reset();
leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IMUReset();
        RobotX = 0;
        RobotY = 0;
        while (opModeIsActive()) {
//            TESTfLeft = leftFrontMotor.getCurrentPosition();
//            TESTfRight = rightFrontMotor.getCurrentPosition();
//            TESTbLeft = leftBackMotor.getCurrentPosition();
//            TESTbRight = rightBackMotor.getCurrentPosition();
//            UpdateControls();
//            LiftControl();
//            IMUstuffs();

//
              RunToPoint(24,24);
           // RunToPoint(0,0);
//            GridRunner();
//            ProportionalFeedbackControl();
//            straferAlgorithm();
//            UpdateEncoders();
            RobotAngles();
//            UpdateOdometry();
//            speedControl();
//            setMotorPower();
//            IMUstuffs();
//
            telemetry.addData("Y", RobotY);
            telemetry.addData("X", RobotX);
            telemetry.addData("Angle", RobotAngle);
            telemetry.addData( "a motor", FrontLeft);
            telemetry.addData("leftfrontmotorencoder", leftFrontMotor.getCurrentPosition());
            telemetry.addData("leftbackmotorencoder", leftBackMotor.getCurrentPosition());
            telemetry.addData("rightfrontmotorencoder", rightFrontMotor.getCurrentPosition());
            telemetry.addData("rightbackmoterencoder", rightBackMotor.getCurrentPosition());
            telemetry.addData("stick data",(leftFrontPower));
            telemetry.addData("stick Data Y", drive);
            telemetry.addData("dpad",gamepad1.dpad_up);
            telemetry.update();


        }
    }
}
