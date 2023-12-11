package org.firstinspires.ftc.teamcode.EasyOpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.ArrayList;

@Autonomous(name="SimpleOpenCVOpMode")
public class SimpleOpenCVOpMode extends OpMode {
    static final int STREAM_WIDTH = 640; // modify for your camera
    static final int STREAM_HEIGHT = 480; // modify for your camera
    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        });

    }

    @Override
    public void loop() {
        telemetry.addData("Image Analysis:",pipeline.getYAnalysis());


        telemetry.addData("Image Analysis:",pipeline.getCrAnalysis());


        telemetry.addData("Image Analysis:",pipeline.getCbAnalysis());
        telemetry.update();
    }


}

class SamplePipeline extends OpenCvPipeline {

    Mat YCrCb = new Mat();
    Mat Y = new Mat();
    Mat Cr = new Mat();
    Mat Cb = new Mat();
    int avg0;
    int avg1;
    int avg2;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable
     */
    void inputToYCrCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        ArrayList<Mat> yCrCbChannels = new ArrayList<Mat>(3);
        Core.split(YCrCb, yCrCbChannels);
        Y = yCrCbChannels.get(0);
        Cr = yCrCbChannels.get(1);
        Cb = yCrCbChannels.get(2);
    }
    @Override
    public void init(Mat firstFrame) {
        inputToYCrCb(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToYCrCb(input);
        System.out.println("processing requested");
        avg0 = (int) Core.mean(Y).val[0];
        avg1 = (int) Core.mean(Cb).val[0];
        avg2 = (int) Core.mean(Cr).val[0];
        YCrCb.release(); // don't leak memory!
        Y.release(); // don't leak memory!
        Cr.release(); // don't leak memory!
        Cb.release(); // don't leak memory!
        return input;
    }

    public int getYAnalysis() {
        return avg0;
    }
    public int getCrAnalysis() {
        return avg1;
    }
    public int getCbAnalysis() {return avg2;}


}
