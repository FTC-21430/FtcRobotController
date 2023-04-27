

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
@Autonomous(name="Autonomous_Route_Seal_Right_Medium_19", group="Monkey")
public class Autonomous_Route_Seal_Right_Medium_19 extends Robot {
    OpenCvWebcam webcam;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Init();
        CameraInit();
        IntakeClose();
        GetSignalZone();
//closed
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(1);
        waitForStart();
        runtime.reset();
        //L=0.85 R=0.15 is open and L=0 R=1 is closed
        sleep(2000);
       IntakeClose();
        sleep(500);
        liftMotor.setTargetPosition(200);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Zone", Zone);
        telemetry.update();
encoderDrive(0.2, 0.1,0.1,0.1,0.1,5);
        encoderDrive(0.5, -2., -2., 2., 2.,5);
        encoderDrive(0.5,2.06,2.06,2.06,2.06,5);
        //First Forwards
        encoderDrive(0.5, 1.04, 1.04, -1.04, -1.4, 5);
        //Second slide to the Left
        liftMotor.setTargetPosition(MediumJunction);
        sleep(1500);
        encoderDrive(0.3, 0.37,0.37,0.37,0.37,5 );

        sleep(1000);
        stop();
       IntakeOpen();
        sleep(1000);
        encoderDrive(0.3, -0.4,-0.4,-0.4,-0.4,5 );
        sleep(500);
        IntakeClose();
        sleep(200);

        GroundLift();
        stop();
        if (Zone==2){
            encoderDrive(0.5, 1.15, 1.15, -1.15, -1.15, 5);

        }
        if(Zone==3){
            encoderDrive(0.5, 3.45, 3.45, -3.45, -3.45,5);
        }
        if (Zone==1 ){
            encoderDrive(0.5, -1.15,-1.15 , 1.15, 1.15,5);
        }
        if (Zone==0){
            encoderDrive(0.5, 3.25, 3.25, -3.25, -3.25,5);
        }
        sleep( 3000);
        stop();
    }
}