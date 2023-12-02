package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.vision.PDP_DualCamera;
import org.firstinspires.ftc.teamcode.vision.PDP_LeftCam;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline_DualZone;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline;

@Disabled
@Config
@TeleOp(group="c")
public class NewPropDetectionTest extends LinearOpMode
{
    OpenCvCamera webcamL, webcamR;
    PDP_LeftCam pipelineLeft;
    PDP_DualCamera pipelineRight;
    public static int avg1, avg2, avg3, avg4, avg5;
    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);


        webcamL = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        pipelineLeft = new PDP_LeftCam(true, General.AllianceLocation.BLUE_NORTH);
        webcamL.setPipeline(pipelineLeft);
        webcamL.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcamL.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamR = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);
        pipelineRight = new PDP_DualCamera(true, General.AllianceLocation.RED_NORTH);
        webcamR.setPipeline(pipelineRight);
        webcamR.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcamR.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });




        FtcDashboard dash;
        dash = FtcDashboard.getInstance();
        waitForStart();

        while (opModeIsActive())
        {

            //pipelineRight.getAnalysis(pipelineLeft.getReadings());




            telemetry.addData("Analysis", pipelineRight.getAnalysis(pipelineLeft.getReadings()));
            avg1 = pipelineLeft.getReadings()[0];
            avg2 = pipelineLeft.getReadings()[1];
            avg3 = pipelineLeft.getReadings()[2];
            avg4 = pipelineLeft.getReadings()[3];
            avg5 = pipelineLeft.getReadings()[4];
            telemetry.addData("avg1", avg1);
            telemetry.addData("avg2", avg2);
            telemetry.addData("avg3", pipelineRight.getReadings()[0]);
            telemetry.addData("avg1g", avg3);
            telemetry.addData("avg2g", avg4);
            telemetry.addData("avg3g", pipelineRight.getReadings()[1]);
            telemetry.addData("DIF 1", avg1-avg3);
            telemetry.addData("DIF 2", avg2-avg4);
            telemetry.addData("DIF 3", pipelineRight.getReadings()[0]-pipelineRight.getReadings()[1]);
            telemetry.addData("avg5 (calibration)", avg5);
            telemetry.update();


            TelemetryPacket p = new TelemetryPacket();
            p.put("avg1",avg1);
            p.put("avg2", avg2);
            p.put("avg3", avg3);
            p.put("avg4", avg4);
            dash.sendTelemetryPacket(p);
            // Don't burn CPU cycles busy-looping in this sample
            // SIKE! BURN THE CPU!!!!!!
            //sleep(50);
        }
    }


}