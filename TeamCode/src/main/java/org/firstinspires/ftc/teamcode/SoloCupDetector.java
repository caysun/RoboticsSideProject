package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "SoloCupDetector", group="Robot")
//@Disabled
public class SoloCupDetector extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    //Drive Code
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;

    //Create webCam and its parameters
    OpenCvWebcam webcam_1 = null;
    public static final int CAMERA_WIDTH = 1280;
    public static final int CAMERA_HEIGHT = 720;
    //public static final double DISTANCE_STRAFE_RATIO =
    public double soloCupDistance = 0.0;
    public double strafeTarget = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //Set up camera stream and pipeline
        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam_1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);
        CustomizedPipeline cp = new CustomizedPipeline();
        webcam_1.setPipeline(cp);

        webcam_1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam_1.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {

            }
        });
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDrive(DRIVE_SPEED, strafeTarget, -strafeTarget, -strafeTarget, strafeTarget, 15.0);

        sleep(5000);
        //encoderDrive(DRIVE_SPEED, strafeTarget, -strafeTarget, -strafeTarget, strafeTarget, 15.0);

        encoderDrive(DRIVE_SPEED, 1.0-soloCupDistance/2.54, 1.0-soloCupDistance/2.54, 1.0-soloCupDistance/2.54, 1.0-soloCupDistance/2.54, 15.0);

    }
    class CustomizedPipeline extends OpenCvPipeline {
        //Create first mat
        Mat originalMat = new Mat();
        //Create Matrix for threshold values
        Mat thresholdMat = new Mat();
        //Create threshold scalars for red
        Scalar lowerThreshold = new Scalar(0, 50, 50);
        Scalar upperThreshold = new Scalar(10, 255, 255);
        //Distance in centimeters and Width in pixels and centimeters from reference image
        static final double PIXEL_WIDTH = 550.0;
        static final double REAL_WORLD_DISTANCE = 19.5;
        static final double REAL_WORLD_WIDTH = 9.5;
        static final double FOCAL_LENGTH = PIXEL_WIDTH*REAL_WORLD_DISTANCE / REAL_WORLD_WIDTH;
        static final double STRAFE_OFFSET_RATIO = 33.533;
        @Override
        public Mat processFrame(Mat input){
            //Convert image to HSV
            Imgproc.cvtColor(input, originalMat, Imgproc.COLOR_RGB2HSV);
            if(input.empty()) return input;
            //Process image to threshold frame where white pixels are in the red threshold range and black pixels are not
            Core.inRange(originalMat, lowerThreshold, upperThreshold, thresholdMat);
            //Put all contours in list
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(thresholdMat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            //Find largest blob of red
            double largest = 0.0;
            MatOfPoint largestContour = null;
            Rect largestContourBoundingRectangle = null;
            for(MatOfPoint contour : contours){
                Rect currentBoundingRect = Imgproc.boundingRect(contour);
                if(currentBoundingRect.area()>largest){
                    largest = currentBoundingRect.area();
                    largestContour = contour;
                    largestContourBoundingRectangle = currentBoundingRect;
                }
            }
            //Draw contour around red solo cup and display to the camera stream
            //Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 0, 255));
            //Draw Bounding rectangle for rectangle
            Imgproc.rectangle(input, largestContourBoundingRectangle, new Scalar(0.0, 0.0, 255.0), 2);
            //Find pixel width of solo cup
            double pixelWidth = largestContourBoundingRectangle.width;
            double distance = getDistance(pixelWidth);

            double soloCupPosition = largestContourBoundingRectangle.x;
            double target = centerStrafe(soloCupPosition);
            telemetry.addData("Solo Cup Location", soloCupPosition);
            telemetry.addData("Contour Area", largest);
            telemetry.addData("Solo Cup width", pixelWidth);
            telemetry.addData("Distance from Solo Cup", distance);
            telemetry.addData("Strafe Target", target);
            telemetry.update();
            return input;
        }
        //Get distance to solo cup
        // distance = sD*R/err;
        //sD = distance*err/R
        private double getDistance(double pixelWidth){
            double distance = REAL_WORLD_WIDTH * FOCAL_LENGTH / pixelWidth;
            soloCupDistance = distance;
            return distance;
        }
        private double centerStrafe(double soloCupPosition){
            double err = soloCupPosition - CAMERA_WIDTH/2;
            double target = soloCupDistance*err/STRAFE_OFFSET_RATIO/2.54;
            strafeTarget = target;
            return target;
        }
    }
    //4.75cm strafe offset, 331 pixels, 43.78 centimeters from cup
    //4.75cm for 309 pixels at 43.78cm with a width of 245 pixels
    public void encoderDrive(double speed, double leftInchesFront, double rightInchesFront,
                             double leftInchesBack, double rightInchesBack,
                             double timeoutS) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = robot.leftFront.getCurrentPosition() + (int)(leftInchesFront * COUNTS_PER_INCH);
            newRFTarget = robot.rightFront.getCurrentPosition() + (int)(rightInchesFront * COUNTS_PER_INCH);
            newLBTarget = robot.leftBack.getCurrentPosition() + (int)(leftInchesBack * COUNTS_PER_INCH);
            newRBTarget = robot.rightBack.getCurrentPosition() + (int)(rightInchesBack * COUNTS_PER_INCH);
            robot.leftFront.setTargetPosition(newLFTarget);
            robot.rightFront.setTargetPosition(newRFTarget);
            robot.leftBack.setTargetPosition(newLBTarget);
            robot.rightBack.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            robot.setDrivetrainMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.setMotorPowers(speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy()) &&
                    (robot.leftBack.isBusy() && robot.rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLFTarget,  newRFTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        robot.leftFront.getCurrentPosition(), robot.rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.setMotorPowers(0.0);
            // Turn off RUN_TO_POSITION
            robot.setDrivetrainMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }

    }// encoderDrive
}
