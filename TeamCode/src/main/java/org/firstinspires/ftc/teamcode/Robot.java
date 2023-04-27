package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
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
public abstract class Robot extends LinearOpMode {
        IMU imu;
        // Declare OpMode members.
        public ElapsedTime runtime = new ElapsedTime();
        public DcMotor leftFrontMotor = null;
        public DcMotor leftBackMotor = null;
        public DcMotor rightFrontMotor = null;
        public DcMotor rightBackMotor = null;
        public  DcMotor liftMotor = null;
        public boolean rightPressed = false;
        public boolean upStack_old = false;
        public DigitalChannel digitalTouch;  // Hardware Device Object
        public Servo servoL;
        public Servo servoR;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor liftMotor = null;
        public int MediumJunction = 1302;
    Robot.SamplePipeline pipeline =  new Robot.SamplePipeline();
    OpenCvWebcam webcam;
        public void CameraInit(){

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

            webcam.setPipeline(pipeline);
            webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {

                    webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {

                }
            });

        }
        public void Init() {

            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            // get a reference to our digitalTouch object.
            digitalTouch = hardwareMap.get(DigitalChannel.class, "Limit_Switch");
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            // set the digital channel to input.
            digitalTouch.setMode(DigitalChannel.Mode.INPUT);
            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            leftFrontMotor = hardwareMap.get(DcMotor.class, "left_Front");
            leftBackMotor = hardwareMap.get(DcMotor.class, "left_Back");
            rightFrontMotor = hardwareMap.get(DcMotor.class, "right_Front");
            rightBackMotor = hardwareMap.get(DcMotor.class, "right_Back");
            liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
            rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
            liftMotor.setTargetPosition(0);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servoL = hardwareMap.get(Servo.class, "left_hand");
            servoR = hardwareMap.get(Servo.class, "right_hand");
            servoL.setPosition(0.72);
            servoR.setPosition(0.33);



        }
        public void IntakeOpen(){
          servoL.setPosition(0.85);
          servoR.setPosition(0.285);
        }
        public void IntakeClose(){
            servoL.setPosition(0.72);
            servoR.setPosition(0.39);
        }
        public void GroundLift(){
            liftMotor.setTargetPosition(100);
        }
        public void LowLift(){
            liftMotor.setTargetPosition(774);
        }
        public void MidLift(){
            liftMotor.setTargetPosition(1302);
        }
        public void HighLift(){
            liftMotor.setTargetPosition(1820);
        }
    static final double     COUNTS_PER_FOOT  =  546.4480;
    public void encoderDrive(double speed,
                             double leftFrontMotorFoot, double rightBackMotorFoot, double leftBackMotorFoot, double rightFrontMotorFoot,
                             double timeoutS) {
        int newleftBackMotorTarget;
        int newletfFrontMotorTarget;
        int newrightFrontMotorTarget;
        int newrightBackMotorTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newleftBackMotorTarget = leftBackMotor.getCurrentPosition() + (int)(leftBackMotorFoot * COUNTS_PER_FOOT);
            newletfFrontMotorTarget= leftFrontMotor.getCurrentPosition() + (int)(leftFrontMotorFoot * COUNTS_PER_FOOT);
            newrightFrontMotorTarget = rightFrontMotor.getCurrentPosition() + (int)(rightFrontMotorFoot* COUNTS_PER_FOOT);
            newrightBackMotorTarget = rightBackMotor.getCurrentPosition() + (int)(rightBackMotorFoot * COUNTS_PER_FOOT);


            leftBackMotor.setTargetPosition(newleftBackMotorTarget);
            leftFrontMotor.setTargetPosition(newletfFrontMotorTarget);
            rightBackMotor.setTargetPosition(newrightBackMotorTarget);
            rightFrontMotor.setTargetPosition(newrightFrontMotorTarget);

            // Turn On RUN_TO_POSITION
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            leftBackMotor.setPower(Math.abs(speed));
            leftFrontMotor.setPower(Math.abs(speed));
            rightBackMotor.setPower(Math.abs(speed));
            rightFrontMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontMotor.isBusy() && leftBackMotor.isBusy() && rightFrontMotor.isBusy() && rightBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newletfFrontMotorTarget,  newleftBackMotorTarget, newrightBackMotorTarget, newrightFrontMotorTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftBackMotor.getCurrentPosition(), leftFrontMotor.getCurrentPosition(), rightBackMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        Scalar REDsum = new Scalar(0);
        Scalar REDsecondsum = new Scalar(0);
        Scalar BLUEsum = new Scalar(0);
        Scalar GREENsum = new Scalar(0);
        double[] midPixel = new double[3];


        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.cvtColor(input,input,Imgproc.COLOR_RGBA2RGB); /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */


            Mat cropped = new Mat(input,new Rect(530, 228,100 , 150));

            Imgproc.cvtColor(cropped,cropped,Imgproc.COLOR_RGB2HSV);
            midPixel = cropped.get(50,75);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */



            Scalar low = new Scalar(0, 127, 51);
            Scalar high = new Scalar(40,255,255);
            Mat out = new Mat();
            Core.inRange(cropped, low, high, out);
            REDsum = Core.sumElems(out);

            low = new Scalar(140, 100, 51);
            high = new Scalar(180,255,255);
            out = new Mat();
            Core.inRange(cropped, low, high, out);
            REDsecondsum =Core.sumElems(out);

            low = new Scalar(97, 120, 51);
            high = new Scalar(130,255,255);
            out = new Mat();
            Core.inRange(cropped, low, high, out);
            BLUEsum = Core.sumElems(out);

            low = new Scalar(40, 127, 51);
            high = new Scalar(97,255,255);
            out = new Mat();
            Core.inRange(cropped, low, high, out);
            GREENsum = Core.sumElems(out);

            Imgproc.cvtColor(cropped,cropped,Imgproc.COLOR_HSV2RGB);
            return cropped;
        }
        public double getREDsum(){return (REDsum.val[0] / 255)+(REDsecondsum.val[0]/255) ;}
        public double getBLUEsum(){return BLUEsum.val[0] / 255;}
        public double getGREENsum(){return GREENsum.val[0] / 255;}
        public double getMidH(){return midPixel[0];}
        public double getMidS(){return midPixel[1];}
        public double getMidV(){return midPixel[2];}
        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }

        }
    }
    int Zone = 0;
    public void GetSignalZone(){

        if (pipeline.getREDsum() >= pipeline.getBLUEsum() && pipeline.getREDsum() >= pipeline.getGREENsum()){
            Zone = 1;
        }
        if (pipeline.getBLUEsum() >= pipeline.getREDsum() && pipeline.getBLUEsum() >= pipeline.getGREENsum()){
            Zone = 2;
        }
        if (pipeline.getGREENsum() >= pipeline.getBLUEsum() && pipeline.getGREENsum() >= pipeline.getREDsum()){
            Zone = 3;
        }
        telemetry.addData("Zone", Zone);
        telemetry.update();
    }
}

