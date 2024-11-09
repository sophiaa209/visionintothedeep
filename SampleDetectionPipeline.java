package org.firstinspires.ftc.teamcode.drive.writtenCode;

import static org.opencv.core.Core.bitwise_and;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.core.Mat.zeros;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

///TEORIE
//metoda principala e processFrame si are in ea metoda imageProc pt color detection
//doua clase nested SampleDetectionPipeline && SamplePipeline
//obiectul SampleDetectionPipeline e creat cu constructor in switch caseu mare de auto

///FUNCTIONALITATE
//se detecteaza o culoare in functie de stateu pasat in constructor


public class SampleDetectionPipeline {

    public OpenCvWebcam webcam;
    public static int rezX = 1920;
    public static int rezY = 1080;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public ColorState currentColor = null;

    public SampleDetectionPipeline(HardwareMap map, Telemetry tel, ColorState col) {
        hardwareMap = map;
        telemetry = tel;
        currentColor = col;
    }

    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SamplePipeline());
        webcam.openCameraDevice();
    }

    void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {

        }
    }

    public void stop() {
        webcam.stopStreaming();
        webcam.stopRecordingPipeline();
    }

    public void start() {
        webcam.startStreaming(rezX, rezY, OpenCvCameraRotation.SIDEWAYS_RIGHT);
    }

    public enum ColorState {
        BLUE,
        RED,
        YELLOW,
        NOTHING
    }
    class SamplePipeline extends OpenCvPipeline {


        @Override
        public Mat processFrame(Mat input) {
            Mat processedFrame = imageProc(input);
            return processedFrame;
        }

        public Mat imageProc(Mat frame) {
            Mat img = new Mat();
            Imgproc.cvtColor(frame, img, Imgproc.COLOR_BGR2HSV);
            Mat mask = new Mat();

            Scalar lowerBlue = new Scalar(100, 150, 50);
            Scalar higherBlue = new Scalar(140, 255, 255);

            Scalar lowerRed = new Scalar(0, 150, 50);
            Scalar higherRed = new Scalar(10, 255, 255);

            Scalar lowerYellow = new Scalar(20, 150, 50);
            Scalar higherYellow = new Scalar(30, 255, 255);


            // metoda inRange() pt fiecare culoare
        switch (currentColor) {
            case BLUE:
                Core.inRange(img, lowerBlue, higherBlue, mask);
                break;
            case RED:
                Core.inRange(img, lowerRed, higherRed, mask);
                break;
            case YELLOW:
                Core.inRange(img, lowerYellow, higherYellow, mask);
                break;
            case NOTHING:
                zeros(img.size(), CV_8U);
                break;
        }

            // bitwise_and() o aplic doar o data

            Mat result = new Mat();
            bitwise_and(frame, frame, result, mask);
            return result;
        }
    }
}