package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class OpenCvExample extends OpMode {
    OpenCvWebcam webcam1;
    @Override
    public void init() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class,"Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened(){

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
        Mat leftCrop;
        Mat rightCrop;
        Mat middleCrop;
        double leftavgfin;
        double rightavgfin;
        double middleavgfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(255.0,255.0,0.0);


        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1,1,310,480);
            Rect rightRect = new Rect(330,1,310,480);
            Rect middleRect = new Rect(310,1,20,480);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2);
            Imgproc.rectangle(output, middleRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            middleCrop = YCbCr.submat(middleRect);

            Scalar leftAvg = Core.mean(leftCrop);
            Scalar rightAvg = Core.mean(rightCrop);
            Scalar middleAvg = Core.mean(middleCrop);

            leftavgfin = leftAvg.val[0];
            rightavgfin = rightAvg.val[0];
            middleavgfin = middleAvg.val[0];

            if(leftavgfin > rightavgfin){
                telemetry.addLine("Left");
            }
            else if(leftavgfin > rightavgfin && leftavgfin < rightavgfin){
                telemetry.addLine("Middle");
                //  cone = 1;
                //  telemetry.addLine("Cone Val: "+cone);
            }
            else if(leftavgfin < rightavgfin){
                telemetry.addLine("Right");
            }


            return(output);
        }
    }
}
