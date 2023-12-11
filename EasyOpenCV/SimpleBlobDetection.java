package org.firstinspires.ftc.teamcode.EasyOpenCV;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.Mat;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

/*
Object dimensions

2-4 in

2-4 in
 */

public class SimpleBlobDetection extends OpMode {

    //Resolution
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera
    OpenCvWebcam webcam;
    OpenCVBlobPipeline pipeline;

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}

class OpenCVBlobPipeline extends OpenCvPipeline {

    Mat Blobs;
    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, Blobs, Imgproc.COLOR_RGB2GRAY);

        

        System.out.println("processing requested");
        return null;
    }
}