package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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


@Config
@TeleOp(group="c")
public class RightCameraTest extends LinearOpMode
{
    OpenCvCamera webcamL, webcamR;
    PDP_LeftCam pipelineLeft;
    PDP_DualCamera pipelineRight;
    public static int avg1, avg2, avg3, avg4, avg5;
    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamR = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
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

            ;

            telemetry.addData("avg3", pipelineRight.getReadings()[0]);

            telemetry.addData("avg3g", pipelineRight.getReadings()[1]);

            telemetry.addData("DIF 3", pipelineRight.getReadings()[0]-pipelineRight.getReadings()[1]);
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