package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Odometry_Test", group="summerRocks")
public class Odometry_Test extends AutoClass {

    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            UpdateControls();
            GridRunner();
            straferAlgorithm();
            UpdateOdometry();
            UpdateEncoders();
            RobotAngles();
            setMotorPower();
            telemetry.addData("Y", RobotY);
            telemetry.addData("X", RobotX);
            telemetry.addData("Angle", RobotAngle);
            telemetry.update();


        }
    }
}
