package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class Autonomie extends LinearOpMode{

    DcMotor slider;
    Servo exsus, intinzator, cleste;
    CRServo exbazast, exbazadr;

    TouchSensor touch;

    double vitezaSlider = 0.8;


    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-32.87,-63.97,0);
        drive.setPoseEstimate(startPose);

        slider = hardwareMap.dcMotor.get("slider");
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setDirection(DcMotorSimple.Direction.REVERSE);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        exbazast = hardwareMap.crservo.get("exbazast"); // in pinul 0
        exbazast.setDirection(CRServo.Direction.REVERSE);
        exbazadr = hardwareMap.crservo.get("exbazadr"); // in pinul 1
        exsus = hardwareMap.servo.get("exsus");
        intinzator = hardwareMap.servo.get("intinzator");

        cleste = hardwareMap.servo.get("cleste");

        touch = hardwareMap.touchSensor.get("touch");



        
        TrajectorySequence startStack = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(-32.87,-45),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//47.244
                .lineToConstantHeading(new Posde2d(-30.68, -10.26, Math.toRadians(180))
                .build();


        waitForStart();
        reset();
        cleste.setPosition(0.6);
        sleep(1000);
        actionarSlider(1150);
        drive.followTrajectorySequence(startStack);
        cleste.setPosition(0.2);
    }
    public void reset() {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(-10000);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slider.isBusy()) {
            if (!touch.isPressed()) {
                slider.setPower(vitezaSlider);
            }
            else {
                slider.setPower(0);
                break;
            }
        }
    }
    public void actionarSlider(int pos) {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (slider.isBusy()) {
            slider.setPower(vitezaSlider);
        }
    }
    public void sliderJos(int pos) {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(-pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (slider.isBusy()) {
            slider.setPower(-vitezaSlider);
        }
    }
}
