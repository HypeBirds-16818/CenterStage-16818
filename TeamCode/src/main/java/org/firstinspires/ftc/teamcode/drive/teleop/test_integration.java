package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Disabled
@TeleOp(name = "Integrated Test")
public class test_integration extends LinearOpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // Creacion de drive
    CRServo planeServo; // Declarar servo del avion (Meter al MechanumDrive)
    public static int target = 0; // target para tunear con Dashboard (temporal)
    public static String section = "Top"; // seccion para tunear con Dashboard (temporal)

    SampleMecanumDrive.intakeState intakeState = SampleMecanumDrive.intakeState.IDLE; // Inicializar maquina de estado en "IDLE"

    @Override
    public void runOpMode() throws InterruptedException {
        drive.setSlideMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Resetear encoders a 0 apenas se ponga play
        planeServo = hardwareMap.get(CRServo.class, "planeServo"); // Conseguir servo avion del hardware map (Meter al MechanumDrive)
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) { //loop
            //setPid(drive, section, target); // PID para tunear (temporal)

            if(gamepad1.a){
                planeServo.setPower(1); // Probar a donde tiene que girar el servo del avion
            }
            if(gamepad1.b){
                planeServo.setPower(-1); // Probar a donde tiene que girar el servo del avion
            }
            if(gamepad1.x){
                planeServo.setPower(0); // Parar el servo
            }

            switch (intakeState) { // Maquina de estado
                case IDLE:
                    if(gamepad2.a){
                        //Tirar intake abajo
                        intakeState = SampleMecanumDrive.intakeState.TAKING;
                    }
                break;
                case TAKING:
                    //rodar intake x segundos hacia adentro
                    if(gamepad2.a){ //Presionar a cuando agarre las cosas
                        //subir intaka y pararlo
                        intakeState = SampleMecanumDrive.intakeState.RAISING1;
                    }
                break;
                case RAISING1:
                    //subir el intake primer etapa
                    //setPid(drive, "Bottom", firstStage);
                    //setPid(drive, "Top", firstStage);
                    intakeState = SampleMecanumDrive.intakeState.RAISING2;
                    break;
                case RAISING2:
                    //subir el intake etapa final
                    //setPid(drive, "Bottom", finalStage);
                    //setPid(drive, "Top", finalStage);
                    intakeState = SampleMecanumDrive.intakeState.TURNING;
                    break;
                case TURNING:
                    //voltear los servos de la caja para dejarla pegada al backdrop
                    //servo1.setPosition()
                    //servo2.setPosition()

                    if(gamepad2.a) {  //esperar input para voltear
                        intakeState = SampleMecanumDrive.intakeState.OPENING;
                    }
                    break;
                case OPENING:

                    //abrir servo de caja para dejar caer los pixeles
                    //servo3.setPosition()

                    if(gamepad2.a){ //esperar input para empezar descenso
                        intakeState = SampleMecanumDrive.intakeState.TURNINGBACK;
                    }

                    break;
                case TURNINGBACK:

                    //cerrar caja
                    //servo3.setPosition()

                    //voltearla a posicion inicial
                    //servo1.setPosition()
                    //servo2.setPosition()

                    intakeState = SampleMecanumDrive.intakeState.FALLING1;

                    break;
                case FALLING1:

                    //caida etapa inicial
                    //setPid(drive, "Bottom", firstStage);
                    //setPid(drive, "Top", firstStage);

                    intakeState = SampleMecanumDrive.intakeState.FALLING2;

                    break;
                case FALLING2:

                    //caida etapa final (posicion inicial)
                    //setPid(drive, "Bottom", finalStage);
                    //setPid(drive, "Top", finalStage);

                    //bajar motor lentamente para asegurar que este hasta abajo
                    //allmotors.setPower(0.05)

                    //reiniciar encoders para asegurar el 0
                    //drive.setModeIntake(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    intakeState = SampleMecanumDrive.intakeState.IDLE;

                    break;
            }
        }
    }

    /*
    public void setPid(SampleMecanumDrive drive, String PIDcase, int target){
        if (PIDcase.equals("Top")){
            drive.getTopPID(target);
        }
        else if (PIDcase.equals("Bottom")){
            drive.getBottomPID(target);
        }



    }
    */


}


