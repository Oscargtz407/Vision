package org.firstinspires.ftc.teamcode.Vision;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class ChessTowerDetection extends OpenCvPipeline {
    private UGContourRingPipeline pipeline;

    private Rect towerRect;

    public ChessTowerDetection() {
        towerRect = null;
    }

    public Mat processFrame(Mat input) {
        pipeline.processFrame(input);

        Mat filteredImage = new Mat();
        Scalar lowerBound = new Scalar(0, 0, 0);
        Scalar upperBound = new Scalar(255, 255, 255);
        Core.inRange(input, lowerBound, upperBound, filteredImage);

        List<MatOfPoint> contours = (List<MatOfPoint>) new MatOfPoint();
        Mat hierarchy = new Mat();
        Imgproc.findContours(filteredImage, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > 1000) {
                towerRect = Imgproc.boundingRect(contour);
                break;
            }
        }

        if (towerRect != null) {
            Imgproc.rectangle(input, towerRect.tl(), towerRect.br(), new Scalar(0, 255, 0), 2);
        }

        return input;
    }

    public Rect getTowerRect() {
        return towerRect;
    }
}
