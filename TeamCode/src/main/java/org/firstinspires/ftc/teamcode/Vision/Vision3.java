package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
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

        camera1.setPipeline(new ExamplePipeline());

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
        // No es necesario implementar el loop para este caso
    }

    // ...

    class ExamplePipeline extends OpenCvPipeline {
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2YCrCb);

            Rect chessTowerRect = new Rect(1, 1, 425, 719);

            Mat chessTowerCrop = hsv.submat(chessTowerRect);

            Scalar lowerBound = new Scalar(0, 100, 100); // Ajusta según el rango de rojo en HSV
            Scalar upperBound = new Scalar(10, 255, 255); // Ajusta según el rango de rojo en HSV

            Mat mask = new Mat();
            Core.inRange(chessTowerCrop, lowerBound, upperBound, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            Mat output = input.clone();
            Imgproc.rectangle(output, chessTowerRect, rectColor, 2);

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > 500 && area < 5000) {  // Ajusta según tus necesidades
                    double epsilon = 0.02 * Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
                    MatOfPoint2f approxCurve = new MatOfPoint2f();
                    Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), approxCurve, epsilon, true);

                    // Verificar si el contorno es cuadrado y ajustar según la forma específica si es necesario
                    if (approxCurve.total() == 4) {
                        // Puedes agregar condiciones adicionales aquí para verificar la forma específica si es necesario
                        Imgproc.drawContours(output, contours, -1, new Scalar(0, 255, 0), 2);
                        telemetry.addLine("Contorno de torre de ajedrez detectado");
                        break;  // Detener el bucle una vez que se encuentra un contorno válido
                    }
                }
            }

            return output;
        }
    }
}