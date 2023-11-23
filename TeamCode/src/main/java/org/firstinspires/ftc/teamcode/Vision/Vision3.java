package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class Vision3 extends OpMode {
    OpenCvCamera camera1 = null;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camara1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera1.setPipeline(new examplePipeline());

        camera1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera1.startStreaming(800, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void loop() {

    }

    class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat hsvLeft = new Mat();
        Mat hsvRight = new Mat();
        Mat maskLeft = new Mat();
        Mat maskRight = new Mat();
        Mat hierarchyLeft = new Mat();
        Mat hierarchyRight = new Mat();
        List<MatOfPoint> contoursLeft = new ArrayList<>();
        List<MatOfPoint> contoursRight = new ArrayList<>();
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("Center");

            Rect leftRect = new Rect(1, 1, 425, 719);
            Rect rightRect = new Rect(850, 1, 425, 719);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            Mat leftCrop = YCbCr.submat(leftRect);
            Mat rightCrop = YCbCr.submat(rightRect);

            // Convertir a HSV para detectar mejor el color de la torre (izquierda)
            Imgproc.cvtColor(leftCrop, hsvLeft, Imgproc.COLOR_YCrCb2RGB);
            Scalar lowerBoundLeft = new Scalar(20, 50, 50); // Ajusta según el color de la torre
            Scalar upperBoundLeft = new Scalar(30, 255, 255); // Ajusta según el color de la torre
            Core.inRange(hsvLeft, lowerBoundLeft, upperBoundLeft, maskLeft);

            // Convertir a HSV para detectar mejor el color de la torre (derecha)
            Imgproc.cvtColor(rightCrop, hsvRight, Imgproc.COLOR_YCrCb2RGB);
            Scalar lowerBoundRight = new Scalar(20, 50, 50); // Ajusta según el color de la torre
            Scalar upperBoundRight = new Scalar(30, 255, 255); // Ajusta según el color de la torre
            Core.inRange(hsvRight, lowerBoundRight, upperBoundRight, maskRight);

            // Encontrar contornos en la máscara (izquierda)
            Imgproc.findContours(maskLeft, contoursLeft, hierarchyLeft, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            // Encontrar contornos en la máscara (derecha)
            Imgproc.findContours(maskRight, contoursRight, hierarchyRight, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Dibujar contornos en la imagen de salida (izquierda)
            Imgproc.drawContours(outPut, contoursLeft, -1, new Scalar(0, 255, 0), 2);
            // Dibujar contornos en la imagen de salida (derecha)
            Imgproc.drawContours(outPut, contoursRight, -1, new Scalar(0, 255, 0), 2);

            // Realizar acciones basadas en la presencia de contornos (izquierda)
            if (contoursLeft.size() > 0) {
                telemetry.addLine("Contorno de torre de ajedrez detectado en la parte izquierda");
            }

            // Realizar acciones basadas en la presencia de contornos (derecha)
            if (contoursRight.size() > 0) {
                telemetry.addLine("Contorno de torre de ajedrez detectado en la parte derecha");
            }

            return outPut;
        }
    }
}