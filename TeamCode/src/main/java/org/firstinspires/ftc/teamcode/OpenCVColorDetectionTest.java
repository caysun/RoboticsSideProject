package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

/**
 * Use Open CV to detect cone color
 */

@Autonomous(name="OpenCVColorDetectionTest", group="Robot")
//@Disabled
public class OpenCVColorDetectionTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    OpenCvWebcam webcam_1 = null;
    public static final int CAMERA_WIDTH = 1280;
    public static final int CAMERA_HEIGHT = 720;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam_1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);
        webcam_1.setPipeline(new CustomizedPipeline());

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
    }
    class CustomizedPipeline extends OpenCvPipeline {

        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        double leftavgfin;
        double rightavgfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(255, 0 , 0);

        //The SDK calls this method for every frame to get the Matrix
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            Rect leftRect = new Rect (1, 1, CAMERA_WIDTH/2 - 1, CAMERA_HEIGHT - 1);
            Rect rightRect = new Rect(CAMERA_WIDTH/2, 1, CAMERA_WIDTH/2 - 1, CAMERA_HEIGHT - 1);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];

            telemetry.addData("leftavg", leftavgfin);
            telemetry.addData("rightavg", rightavgfin);
            if(leftavgfin > rightavgfin){
                telemetry.addLine("More Red on the Left side");
            }
            else{
                telemetry.addLine("More Red on the Right side");
            }
            telemetry.update();

            return(output);
        }
    }
}
