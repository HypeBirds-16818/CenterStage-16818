package org.firstinspires.ftc.teamcode.drive.autonomo;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.SavePose;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "Rojo")
public class RojoBackboard extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d start_pose = new Pose2d();
        drive.setPoseEstimate(start_pose);

        // Medio y estacionar
        TrajectorySequence dos = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(11.91, -33.11))
                .lineTo(new Vector2d(62.36, -62.36))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(dos);

        SavePose.currentPose = drive.getPoseEstimate();
    }
}
