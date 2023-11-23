package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class Vision extends LinearOpMode {
   public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
            private final AtomicReference<Bitmap> lastFrame =
                    new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

            @Override
            public void init(int width, int height, CameraCalibration calibration) {
                lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
            }

            @Override
            public Object processFrame(Mat frame, long captureTimeNanos) {
                Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(frame, b);
                lastFrame.set(b);
                return null;
            }

            @Override
            public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                    float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                    Object userContext) {
                // do nothing
            }

            @Override
            public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
                continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
            }
        }
    ChessTowerDetection chessTowerDetection = new ChessTowerDetection();

    private static final int CAMERA_WIDTH = 1280; // Ancho deseado de resolución de la cámara
    private static final int CAMERA_HEIGHT = 720; // Alto deseado de resolución de la cámara

    private static final int HORIZON = 100; // Valor del horizonte para ajustar

    private static final boolean DEBUG = false; // Si se desea depuración, cambia a true

    private static final boolean USING_WEBCAM = true; // Cambia a true si estás usando una cámara web
    private static final String WEBCAM_NAME = "Webcam 1"; // Inserta el nombre de la cámara web desde la configuración si estás usando una cámara web

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera = new OpenCvCamera() {
        @Override
        public int openCameraDevice() {
            return 0;
        }

        @Override
        public void openCameraDeviceAsync(AsyncCameraOpenListener cameraOpenListener) {

        }

        @Override
        public void closeCameraDevice() {

        }

        @Override
        public void closeCameraDeviceAsync(AsyncCameraCloseListener cameraCloseListener) {

        }

        @Override
        public void showFpsMeterOnViewport(boolean show) {

        }

        @Override
        public void pauseViewport() {

        }

        @Override
        public void resumeViewport() {

        }

        @Override
        public void setViewportRenderingPolicy(ViewportRenderingPolicy policy) {

        }

        @Override
        public void setViewportRenderer(ViewportRenderer renderer) {

        }

        @Override
        public void startStreaming(int width, int height) {

        }

        @Override
        public void startStreaming(int width, int height, OpenCvCameraRotation rotation) {

        }

        @Override
        public void stopStreaming() {

        }

        @Override
        public void setPipeline(OpenCvPipeline pipeline) {

        }

        @Override
        public int getFrameCount() {
            return 0;
        }

        @Override
        public float getFps() {
            return 0;
        }

        @Override
        public int getPipelineTimeMs() {
            return 0;
        }

        @Override
        public int getOverheadTimeMs() {
            return 0;
        }

        @Override
        public int getTotalFrameTimeMs() {
            return 0;
        }

        @Override
        public int getCurrentPipelineMaxFps() {
            return 0;
        }

        @Override
        public void startRecordingPipeline(PipelineRecordingParameters parameters) {

        }

        @Override
        public void stopRecordingPipeline() {

        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {

        }
    };

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );

        if (USING_WEBCAM) {
            camera.setPipeline(chessTowerDetection);
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        final CameraStreamProcessor processor = new CameraStreamProcessor();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(BuiltinCameraDirection.BACK)
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 10);

        waitForStart();

        while (opModeIsActive()) {
            String height = "[HEIGHT]" + " " + pipeline.getHeight();
            telemetry.addData("[Ring Stack] >>", height);
            telemetry.update();
        }
    }
}
