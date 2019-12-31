package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.PurePursuit.CheckPoint;
import org.firstinspires.ftc.teamcode.PurePursuit.WayPoint;
import org.firstinspires.ftc.teamcode.competition.autonomous.vision.SkystonePipeline;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import static org.firstinspires.ftc.teamcode.competition.helperclasses.ThreadPool.pool;
import static org.firstinspires.ftc.teamcode.competition.autonomous.vision.SkystonePipeline.screenPosition;

@Autonomous(name = "OpenCVSkyStoneAutoPurePursuit", group = "Auto")
public class OpenCVSkyStoneAutoPurePursuit extends AutoMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
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
        PurePursuit purePursuit = new PurePursuit(robot);
        int blockPosition=1;
        while(!isStarted()&&!isStopRequested())
        {
            double xPosition = screenPosition.x;
            if (xPosition > 145)

                blockPosition =
                        xPosition < 190 ?
                                2 : 1;
            else
                blockPosition = 3;
            telemetry.addData("Block Selected", blockPosition);
            updateOdometryTelemetry();
        }
        if(blockPosition == 1){
            ArrayList<WayPoint> path1pos1 = new ArrayList();
            path1pos1.add(new WayPoint(30,-10 , -Math.PI/2));
            CheckPoint check1pos1 = new CheckPoint(30, -10, 2, robot);
            ArrayList<WayPoint> path2pos1 = new ArrayList();
            path2pos1.add(new WayPoint(15,-10,-Math.PI/2));
            path2pos1.add(new WayPoint(15, -60, -Math.PI/2));
            CheckPoint check2pos1 = new CheckPoint(15,-60,2,robot);
            purePursuit.initPath(path1pos1);
            pool.submit(check1pos1);
            while (!isStarted() && opModeIsActive()) {
                updateOdometryTelemetry();
                telemetry.addLine("Position 1");
            }
            while (opModeIsActive() && !check1pos1.isHit) {
                telemetry.addData("PID", purePursuit.pos.getPID());
                updateOdometryTelemetry();
                purePursuit.followPath(path1pos1, 4, 2, 1.2);
            }
            //OPEN INTAKE
            //.5 is open intake and 1 is close intake
            robot.intakeRelease.setPosition(.5);
            while(opModeIsActive() && !check2pos1.isHit){
                telemetry.addData("PID", purePursuit.pos.getPID());
                updateOdometryTelemetry();
                purePursuit.followPath(path2pos1,4,1.5,1.2);
            }
        }
        if(blockPosition == 2){
            ArrayList<WayPoint> path1pos2 = new ArrayList();
            path1pos2.add(new WayPoint(30,-12 , -Math.PI/2));
            CheckPoint check1pos2 = new CheckPoint(30, -12, 2, robot);

            ArrayList<WayPoint> path2pos2 = new ArrayList();
            path2pos2.add(new WayPoint(15,0,-Math.PI/2));
            path2pos2.add(new WayPoint(15, -50, -Math.PI/2));
            CheckPoint check2pos2 = new CheckPoint(15,-50,2,robot);

            while (opModeIsActive() && !isStarted()) {
                telemetry.addLine("Position 2");
                updateOdometryTelemetry();
            }
            while(opModeIsActive() && !check1pos2.isHit){
                telemetry.addData("PID", purePursuit.pos.getPID());
                updateOdometryTelemetry();
                purePursuit.followPath(path1pos2,4,1.5,1.2);
            }
            robot.intakeRelease.setPosition(.5);
            while(opModeIsActive() && !check2pos2.isHit){
                telemetry.addData("PID", purePursuit.pos.getPID());
                updateOdometryTelemetry();
                purePursuit.followPath(path2pos2,4,1.5,1.2);
            }
        }
        if(blockPosition == 3){
            ArrayList<WayPoint> path1pos3 = new ArrayList();
            path1pos3.add(new WayPoint(30,-21,-Math.PI/2));
            CheckPoint check1pos3 = new CheckPoint(30,-21,2,robot);

            ArrayList<WayPoint> path2pos3 = new ArrayList();
            path2pos3.add(new WayPoint(15,0,-Math.PI/2));
            path2pos3.add(new WayPoint(15, -50, -Math.PI/2));
            CheckPoint check2pos3 = new CheckPoint(15,-50,2,robot);
            while (opModeIsActive() && !isStarted()) {
                telemetry.addLine("Position 3");
                updateOdometryTelemetry();
            }
            while(opModeIsActive() && !check1pos3.isHit){
                telemetry.addData("PID", purePursuit.pos.getPID());
                updateOdometryTelemetry();
                purePursuit.followPath(path1pos3,4,1.5,1.2);
            }
            robot.intakeRelease.setPosition(.5);
            while(opModeIsActive() && !check2pos3.isHit){
                telemetry.addData("PID", purePursuit.pos.getPID());
                updateOdometryTelemetry();
                purePursuit.followPath(path2pos3,4,1.5,1.2);
            }
        }
    }
}