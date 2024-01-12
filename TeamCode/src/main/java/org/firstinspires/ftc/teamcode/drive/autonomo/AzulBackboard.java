package org.firstinspires.ftc.teamcode.drive.autonomo;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.SavePose;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "Azul")
public class AzulBackboard extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d start_pose = new Pose2d(12.09, 65.34, Math.toRadians(270));
        drive.setPoseEstimate(start_pose);

        // Medio y estacionar
        TrajectorySequence uno = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(10.82, 31.15))
                .setReversed(true)
                .splineTo(new Vector2d(59.95, 61.46), Math.toRadians(31.67))
                .build();



        // Medio y ponerse en el backboard
        TrajectorySequence Azul_Backboard_Medio = drive.trajectorySequenceBuilder(start_pose)
                //.setReversed(false)
                .lineTo(new Vector2d(11.74, 32.58))
                //.setReversed(true)
//                .UNSTABLE_addTemporalMarkerOffset(3,() ->
//                        drive.setIntakePower(-.2))
                .splineTo(new Vector2d(47.47, 36.44), Math.toRadians(0.00))
                .setReversed(true)
                .turn(Math.toRadians(180))
                .build();

//        TrajectorySequence Azul_Backboard_Izquierda = drive.trajectorySequenceBuilder(new Pose2d(12.09, 65.34, Math.toRadians(90.00)))
//                .setReversed(true)
//                .lineTo(new Vector2d(13.49, 32.58))
//                .UNSTABLE_addTemporalMarkerOffset(3,() ->
//                        drive.setIntakePower(-.2))
//                .splineTo(new Vector2d(31.71, 15.24), Math.toRadians(15.89))
//                .setReversed(false)
//                .splineTo(new Vector2d(47.47, 36.96), Math.toRadians(5.50))
//                .setReversed(true)
//                .build();


//        TrajectorySequence Azul_Backboard_Derecha = drive.trajectorySequenceBuilder(new Pose2d(12.09, 65.34, Math.toRadians(90.00)))
//                .setReversed(true)
//                .lineTo(new Vector2d(10.51, 32.76))
//                .UNSTABLE_addTemporalMarkerOffset(3,() ->
//                        drive.setIntakePower(-.2))
//                .splineTo(new Vector2d(47.47, 36.96), Math.toRadians(5.86))
//                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(uno);

        SavePose.currentPose = drive.getPoseEstimate();
    }
}
