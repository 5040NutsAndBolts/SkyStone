package org.firstinspires.ftc.teamcode.competition.autonomous;


import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.media.Image;
import android.net.NetworkInfo;
import android.net.wifi.p2p.WifiP2pInfo;
import android.net.wifi.p2p.WifiP2pManager;
import android.util.Log;
import android.widget.Toast;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.competition.autonomous.vision.WifiP2PPipeLine;
import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.opencv.android.Utils;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.Buffer;

@Autonomous(group = "Auto",name = "wifi direct test")

public class WifiDirectTest extends OpMode
{

    Hardware robot = new Hardware();
    DatagramSocket socket;
    InetAddress address;

    OpenCvCamera phoneCamera;


    WifiP2PPipeLine pipeLine = new WifiP2PPipeLine();

    ByteArrayOutputStream stream = new ByteArrayOutputStream();

    public void sendEcho(Bitmap msg) {
        msg.compress(Bitmap.CompressFormat.PNG, 100, stream);
        byte[] byteArray = stream.toByteArray();
        msg.recycle();

        DatagramPacket packet
                = new DatagramPacket(byteArray, byteArray.length, address, 4445);
        try {
            socket.send(packet);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }



    public void close() {
        socket.close();
    }

    @Override
    public void init() {

        robot.init(hardwareMap);
        phoneCamera = new OpenCvInternalCamera(
                // Sets if using front or back of camera
                OpenCvInternalCamera.CameraDirection.FRONT,
                // ID of the camera monitor relative to the app context
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName())
        );



        phoneCamera.setPipeline(pipeLine);
        // Starts connection to camera
        phoneCamera.openCameraDevice();

        phoneCamera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);

    }


    InetAddress host;

    {
        try {
            host = InetAddress.getLocalHost();
        } catch (UnknownHostException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {

        Mat tmp = pipeLine.rawImage;
        Bitmap bmp=null;
        try {
            //Imgproc.cvtColor(seedsImage, tmp, Imgproc.COLOR_RGB2BGRA);
            Imgproc.cvtColor(tmp,tmp, Imgproc.COLOR_GRAY2RGBA, 4);
            bmp = Bitmap.createBitmap(tmp.cols(), tmp.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(tmp, bmp);
        }
        catch (CvException e){
            Log.d("Exception",e.getMessage());}

        Socket socket = new Socket();
        String str=null;
        String error = null;
        try {

            socket.connect((new InetSocketAddress(host, 5056)), 100);
            InputStream is=socket.getInputStream();
            ObjectInputStream ois =new ObjectInputStream(is);
            str = (String) ois.readObject();

            ois.close();
            is.close();
            socket.close();
            error = "";

        } catch (Exception e) {
            error=e.toString();
        }
        telemetry.addData("error",error);
        telemetry.addData("String",str);
        telemetry.addData("connection",socket.isConnected());
        telemetry.update();

    }
}
