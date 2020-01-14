package org.firstinspires.ftc.teamcode.competition.autonomous.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Locale;

/**
 * EasyOpenCV testing using a custom vision pipeline for determining the Skystone
 */
@TeleOp(name="Vision Testing", group="OpenCV")
public class VisionTeleop extends LinearOpMode {

    @Override
    public void runOpMode(){

        // Instantiate the phone camera using the back camera
        // Also turn on camera monitoring
        // TODO: Crop the frame so it only looks in a certain area
        OpenCvCamera phoneCamera = new OpenCvInternalCamera(
                // Sets if using front or back of camera
                OpenCvInternalCamera.CameraDirection.FRONT,
                // ID of the camera monitor relative to the app context
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName())
        );

        // Starts connection to camera
        phoneCamera.openCameraDevice();

        // Sets the current image processing pipeline to detect the skystone
        OpenCvPipeline detector = new SkystonePipeline();
        phoneCamera.setPipeline(detector);

        // Start the camera streaming and set the phones rotation
        // In this case the phone is going to be upright (portrait)
        phoneCamera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("selected", SkystonePipeline.screenPosition.x < 90 ? 3 : (SkystonePipeline.screenPosition.x < 145 ? 2 : 1));
            telemetry.addData("X-Position", SkystonePipeline.screenPosition.x);
            telemetry.addData("Y-Position", SkystonePipeline.screenPosition.y);
            telemetry.addData("Viewport stage", SkystonePipeline.stageToRenderToViewport);
            telemetry.addLine("==========");
            telemetry.addData("Frame Count", phoneCamera.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCamera.getFps()));
            telemetry.addData("Total frame time ms", phoneCamera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCamera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCamera.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCamera.getCurrentPipelineMaxFps());
            telemetry.update();
            sleep(100);
        }

        // Stops camera streaming
        phoneCamera.stopStreaming();
    }
}


