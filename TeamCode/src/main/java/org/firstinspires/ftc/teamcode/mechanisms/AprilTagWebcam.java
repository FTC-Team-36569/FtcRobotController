package org.firstinspires.ftc.teamcode.mechanisms;


import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;


    // DECIMATION: how much the picture is shrunk before looking for tags.
// Smaller number = sees tags from farther away, but fewer pictures per second (FPS).
// About how far it can see our 3.25" HIVE tags (straight on, less at an angle):
//   3 = about 6.5 feet, 30 FPS  (SDK default - lost tags at about 70")
//   2 = about 9.5 feet, 22 FPS
//   1 = about 16 feet,  10 FPS
    public static float DECIMATION = 2;


    public void init(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        aprilTagProcessor.setDecimation(DECIMATION);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwmap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size( 640 , 480 ));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update (){
        detectedTags =aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId){
        if(detectedId == null) {return;}
        // SDK 12.0: detections are either a single tag or a tag cluster
        if (detectedId instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detectedId;
            if (singleDet.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
            }
        } else {
            AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detectedId;
            telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        }

    }

    public AprilTagDetection getTagByspecificID(int id){
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagSingleDetection && ((AprilTagSingleDetection) detection).id == id) {
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection getTagByspecificIDs(List<Integer> ids){
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagSingleDetection && ids.contains(((AprilTagSingleDetection) detection).id)) {
                return detection;
            }
//            else if (detection instanceof AprilTagClusterDetection && ids.contains(((AprilTagClusterDetection) detection).tag)) {
//                return detection;
//            }
        }
        return null;
    }

    public AprilTagClusterDetection getClosestCluster (String color) {
        AprilTagClusterDetection closest = null;

        for (AprilTagDetection detection : detectedTags) {
            if (!(detection instanceof  AprilTagClusterDetection)){
                continue;
            }

            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;

            if (cluster.ftcPose == null || !cluster.metadata.name.startsWith(color)) {
                continue;
            }

            if (closest == null || cluster.ftcPose.range < closest.ftcPose.range) {
                closest = cluster;
            }

        }
        return closest;
    }

    public void stop(){
        if (visionPortal != null){
            visionPortal.close();
        }
    }
}