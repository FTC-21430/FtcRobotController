

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Autonomous_Route_Monkey_5", group="Monkey")
public class Autonomous_Route_Monkey_5 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_Front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_Back");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_Front");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_Back");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double slide = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean slowMode = gamepad1.left_bumper;
        //double turn  =  gamepad1.right_stick_x;

        //Power Source
        leftFrontPower = Range.clip(drive + slide + turn, -1.0, 1.0);
        leftBackPower = Range.clip(drive - slide + turn, -1.0, 1.0);
        rightFrontPower = Range.clip(drive - slide - turn, -1.0, 1.0);
        rightBackPower = Range.clip(drive + slide - turn, -1.0, 1.0);

        if (slowMode) {
            leftFrontPower = leftFrontPower / 2;
            leftBackPower = leftBackPower / 2;
            rightFrontPower = rightFrontPower / 2;
            rightBackPower = rightBackPower / 2;
        }

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        //leftPower  = -gamepad1.left_stick_y ;
        //rightPower = -gamepad1.right_stick_y ;
        if (slowMode) {
            leftFrontPower = leftFrontPower / 2;
            leftBackPower = leftBackPower / 2;
            rightFrontPower = rightFrontPower / 2;
            rightBackPower = rightBackPower / 2;


            // Send calculated power to wheels
            leftFrontMotor.setPower(leftFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightBackMotor.setPower(rightBackPower);

        }
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.update();


            //Sideways to the Right
        rightBackMotor.setPower(1);
        leftFrontMotor.setPower(1);
        leftBackMotor.setPower(-1);
        rightFrontMotor.setPower(-1);

        sleep(351);

        //Forwards
        leftBackMotor.setPower(0.25);
        rightBackMotor.setPower(0.25);
        rightFrontMotor.setPower(0.25);
        leftFrontMotor.setPower(0.25);

        sleep(345);

        //Backwards
        leftBackMotor.setPower(-0.25);
        rightBackMotor.setPower(-0.25);
        rightFrontMotor.setPower(-0.25);
        leftFrontMotor.setPower(-0.25);

        sleep(347);

        //sideways to the Left

        leftBackMotor.setPower(1);
        rightBackMotor.setPower(-1);
        rightFrontMotor.setPower(1);
        leftFrontMotor.setPower(-1);

        sleep(645);

        stop();
    }
}