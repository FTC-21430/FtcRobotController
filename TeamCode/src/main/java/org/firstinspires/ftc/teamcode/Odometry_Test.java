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
        while (opModeIsActive()) {
            TESTfLeft = leftFrontMotor.getCurrentPosition();
            TESTfRight = rightFrontMotor.getCurrentPosition();
            TESTbLeft = leftBackMotor.getCurrentPosition();
            TESTbRight = rightBackMotor.getCurrentPosition();
            UpdateControls();
            GridRunner();
            straferAlgorithm();
            UpdateEncoders();
            RobotAngles();
            UpdateOdometry();
            setMotorPower();
            telemetry.addData("Y", RobotY);
            telemetry.addData("X", RobotX);
            telemetry.addData("Angle", RobotAngle);
            telemetry.addData( "a motor", FrontLeft);
            telemetry.addData("leftfrontmotorencoder", leftFrontMotor.getCurrentPosition());
            telemetry.addData("leftbackmotorencoder", leftBackMotor.getCurrentPosition());
            telemetry.addData("rightfrontmotorencoder", rightFrontMotor.getCurrentPosition());
            telemetry.addData("rightbackmoterencoder", rightBackMotor.getCurrentPosition());

            telemetry.update();


        }
    }
}
