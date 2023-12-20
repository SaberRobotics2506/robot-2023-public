package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Camera{
    UsbCamera camera1;
    UsbCamera camera2;
    VideoSink server;
    XboxController opCon;
    boolean cameraToggle = false;

    public Camera(XboxController opController)
    {
        this.opCon = opController;
    }
    public void cameraStart() {
        camera1 = CameraServer.startAutomaticCapture(0);
        //camera2 = CameraServer.startAutomaticCapture(1);
        server = CameraServer.getServer();
    } 
    public void cameraSwitcher(){
        if(opCon.getBButtonPressed())
        {
            cameraToggle = !cameraToggle;
            if (cameraToggle) 
            {
                SmartDashboard.putString("Camera Selected:", "Front Camera");
                server.setSource(camera2);
            } 
            else
            {
                SmartDashboard.putString("Camera Selected:", "Claw Camera");
                server.setSource(camera1);
            }
        }
    }
}
