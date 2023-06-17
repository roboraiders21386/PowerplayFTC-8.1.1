package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.drive.MecanumDrive.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.VisionEasyOpenCV;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * RoboRaisers # 21386 Autonomous using EasyOpenCV & for dropping preloaded cone, picking and dropping 2 cones and park
 */
@Autonomous(name = "RoboRaiders Autonomous Full")
@Disabled
public class AutoOpMode extends LinearOpMode{
    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }


    public static START_POSITION startPosition;


    public VisionEasyOpenCV visionEasyOpenCV;
    private OpenCvCamera camera;
    public DriveTrain driveTrain;
    private String webcamName = "Webcam 1";
    private DcMotor Slide;
    private Servo Claw;

    private  double PARK_DISTANCE = 24;
    private  double STRAFE_DISTANCE = 24;
    private double CAMERA_OFFSET = 4 ;

    private double CLAW_MID_POS = 0.50;

    //Claw
    // 1: is for Opening claw
    //0 is for claw close
    private double CLAW_OPEN_POSITION = 0.32;
    private double CLAW_CLOSE_POSITION = 0;

    private int HIGH_JUNCTION_Y = 3200;//3800;
    private int MEDIUM_JUNCTION_B = 2800;//2700;
    private int LOW_JUNCTION_A = 1600;//1500;
    public static int ZERO_POS = 0;//1500;

    @Override
    public void runOpMode() throws InterruptedException {
        /*Create your Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        visionEasyOpenCV = new VisionEasyOpenCV();

        //Initialize claw and slide motor
        Slide = hardwareMap.get(DcMotor.class, "motor");
        Claw = hardwareMap.get(Servo.class, "Claw");

        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setDirection(DcMotorSimple.Direction.REVERSE);
        Claw.scaleRange(0,1);


        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();

        // Initiate Camera on INIT.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        camera.setPipeline(visionEasyOpenCV);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        //Set DEFAULT as LEFT
        VisionEasyOpenCV.ParkingPosition cameraPosition = VisionEasyOpenCV.ParkingPosition.ONE;

        //Build Autonomous trajectory to be used based on starting position selected
        buildAllianceAuto(cameraPosition);
        driveTrain.getLocalizer().setPoseEstimate(initPose);

        while (!isStopRequested() && !opModeIsActive()) {
            //Run visionEasyOpenCV.getPosition() and keep watching for the identifier in the Signal Cone.
            telemetry.clearAll();
            telemetry.addData("Start RoboRaiders Autonomous Mode adopted for Team","21386");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Alliance Position", startPosition);
            //telemetry.addData("Camera Detected: ", visionEasyOpenCV.getPosition());
            telemetry.addData("Camera Detected: ", cameraPosition);
            telemetry.update();
        }
        //Set DEFAULT as LEFT
        //VisionEasyOpenCV.ParkingPosition cameraPosition = VisionEasyOpenCV.ParkingPosition.LEFT;

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) { //21386
            //Start is pressed

            //Stop VisionEasyOpenCV process
            cameraPosition = visionEasyOpenCV.getPosition();
            camera.stopStreaming();

            Claw.setPosition(CLAW_CLOSE_POSITION);

            //Set the arm to ground junction height
            /*
            Slide.setTargetPosition(600);
            Slide.setPower(0.3);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Slide.isBusy()) {
                telemetry.addData("Current Position", Slide.getCurrentPosition());
                telemetry.addData("Target Position", Slide.getTargetPosition());
                telemetry.update();
            }*/

            //buildAllianceAuto();
            //Build parking trajectory based on last detected target by visionEasyOpenCV
            //buildParkingFromMidwayPose(cameraPosition);


            //run Autonomous trajectory
            //runAutoAndParking(position);
            runAuto(Slide, Claw, cameraPosition);
        }

        //Trajectory is completed, display Parking complete
        parkingComplete(cameraPosition);
    }

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryAuto, trajectoryParking ;
    TrajectorySequence trajectoryParkingLEFT ;
    TrajectorySequence trajectoryParkingRIGHT ;
    TrajectorySequence trajectoryParkingCENTER ;

    TrajectorySequence trajectoryAutoNew ;

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    Pose2d midWayPose;
    Pose2d pickConePose;
    Pose2d dropConePose0, dropConePose1, dropConePose2;
    Pose2d parkPose;
    Pose2d dropConePose, Pose2, Pose3, Pose4, Pose5, Pose1_5;
    Pose2d firstPose;

    Trajectory goToCone, goToCone1_5;
    Trajectory goToCone2;
    Trajectory scoreCone;
    Trajectory goAwayCone;
    Trajectory goToParking;
    Trajectory parkRobot;

    Pose2d p1, p2, p3, p4, p5, p6;
    Trajectory t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15;

    public void runAuto(DcMotor Slide, Servo Gripper, VisionEasyOpenCV.ParkingPosition cameraPosition) {
        //Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.followTrajectory(t1);

        driveTrain.followTrajectory(t2);
        driveTrain.followTrajectory(t3);
        //Run to high junction
        Slide.setTargetPosition(3000);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(0.7);
        while (Slide.isBusy()) {
            telemetry.addData("Linear Slide Current Position: ", Slide.getCurrentPosition());
            telemetry.update();
        }
        sleep(250);
        driveTrain.followTrajectory(t4);
        Gripper.setPosition(CLAW_OPEN_POSITION);//Open Claw
        sleep(500);
        driveTrain.followTrajectory(t5);
        //Run to high junction
        Slide.setTargetPosition(800);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(0.7);

        while (Slide.isBusy()) {
            telemetry.addData("Linear Slide Current Position: ", Slide.getCurrentPosition());
            telemetry.update();
        }
        sleep(250);
        driveTrain.followTrajectory(t6);
        driveTrain.followTrajectory(t7);
        driveTrain.followTrajectory(t8);
        Gripper.setPosition(CLAW_CLOSE_POSITION);//Close Claw
        sleep(250);
        Slide.setTargetPosition(1200);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(0.7);

        while (Slide.isBusy()) {
            telemetry.addData("Linear Slide Current Position: ", Slide.getCurrentPosition());
            telemetry.update();
        }
        sleep(250);
        driveTrain.followTrajectory(t9);
        driveTrain.followTrajectory(t10);
        driveTrain.followTrajectory(t11);
        Slide.setTargetPosition(3000);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(0.7);

        while (Slide.isBusy()) {
            telemetry.addData("Linear Slide Current Position: ", Slide.getCurrentPosition());
            telemetry.update();
        }
        sleep(250);
        driveTrain.followTrajectory(t12);
        Gripper.setPosition(CLAW_OPEN_POSITION);
        sleep(500);
        driveTrain.followTrajectory(t13);
        Gripper.setPosition(CLAW_CLOSE_POSITION);
        Slide.setTargetPosition(0);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(0.7);

        while (Slide.isBusy()) {
            telemetry.addData("Linear Slide Current Position: ", Slide.getCurrentPosition());
            telemetry.update();
        }
        sleep(250);
        driveTrain.followTrajectory(t14);
        driveTrain.followTrajectory(t15);
        driveTrain.followTrajectory(parkRobot);
    }

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAllianceAuto(VisionEasyOpenCV.ParkingPosition cameraPosition) {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-70, 32, Math.toRadians(0)); //Starting pose
                dropConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                midWayPose = new Pose2d(-42, 36, Math.toRadians(0));
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-64, -26, Math.toRadians(270));//Starting pose
                midWayPose = new Pose2d(-12, -12, Math.toRadians(270)); //Choose the pose to move forward towards signal cone
                p1 = new Pose2d(-64, -9, Math.toRadians(270)); //Push the signal sleeve a bit further out
                p2 = new Pose2d(0, -12, Math.toRadians(270)); // Come back inline to the High Junction
                p3 = new Pose2d(0, -17, Math.toRadians(270)); //Close to cone
                p4 = new Pose2d(0, -19, Math.toRadians(270));// On top of cone - open claw will happen next
                p5 = new Pose2d(-12, -12, Math.toRadians(270)); // Step back after scoring
                p6 = new Pose2d(-12, -60, Math.toRadians(270));
                t1 = driveTrain.trajectoryBuilder(initPose)
                        .lineToLinearHeading(p1)
                        .build();
                t2 = driveTrain.trajectoryBuilder(p1)
                        .lineToLinearHeading(p2)
                        .build();
                t3 = driveTrain.trajectoryBuilder(p2)
                        .lineToLinearHeading(p3)
                        .build();
                t4 = driveTrain.trajectoryBuilder(p3)
                        .lineToLinearHeading(p4)
                        .build();
                t5 = driveTrain.trajectoryBuilder(p4)
                        .lineToLinearHeading(p3)
                        .build();
                t6 = driveTrain.trajectoryBuilder(p3)
                        .lineToLinearHeading(p2)
                        .build();
                t7 = driveTrain.trajectoryBuilder(p2)
                        .lineToLinearHeading(p5)
                        .build();
                t8 = driveTrain.trajectoryBuilder(p5)
                        .lineToLinearHeading(p6)
                        .build();
                t9 = driveTrain.trajectoryBuilder(p6)
                        .lineToLinearHeading(p5)
                        .build();
                t10 = driveTrain.trajectoryBuilder(p5)
                        .lineToLinearHeading(p2)
                        .build();
                t11 = driveTrain.trajectoryBuilder(p2)
                        .lineToLinearHeading(p3)
                        .build();
                t12 = driveTrain.trajectoryBuilder(p3)
                        .lineToLinearHeading(p4)
                        .build();
                t13 = driveTrain.trajectoryBuilder(p4)
                        .lineToLinearHeading(p3)
                        .build();
                t14 = driveTrain.trajectoryBuilder(p3)
                        .lineToLinearHeading(p2)
                        .build();
                t15 = driveTrain.trajectoryBuilder(p3)
                        .lineToLinearHeading(p5)
                        .build();
                //Park based on the camera detected signal LEFT: Forward, RIGHT: Backward, CENTER: stay where you are!

                switch (cameraPosition) {

                    case TWO:
                        parkRobot = driveTrain.trajectoryBuilder(p5)
                                .lineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(270)))
                                .build();
                        break;
                    case THREE:
                        parkRobot = driveTrain.trajectoryBuilder(p5)
                                .lineToLinearHeading(new Pose2d(-12, -60, Math.toRadians(270)))
                                .build();
                        break;
                }
                break;
            case RED_LEFT:
                initPose = new Pose2d(70, -32, Math.toRadians(180));//Starting pose
                dropConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                midWayPose = new Pose2d(42, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                break;
            case RED_RIGHT:
                initPose = new Pose2d(70, 32, Math.toRadians(180)); //Starting pose
                dropConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                midWayPose = new Pose2d(40, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                Pose1_5 = new Pose2d(-4, 36, Math.toRadians(270));
                Pose2 = new Pose2d(0, 36, Math.toRadians(270));
                Pose3 = new Pose2d(0, 32, Math.toRadians(270));
                Pose4 = new Pose2d(0, 29, Math.toRadians(270));
                Pose5 = new Pose2d(0, 36, Math.toRadians(270));

                goToCone = driveTrain.trajectoryBuilder(initPose)
                        .lineToLinearHeading(Pose1_5)
                        .build();
                goToCone1_5 = driveTrain.trajectoryBuilder(Pose1_5)
                        .lineToLinearHeading(Pose2)
                        .build();
                goToCone2 = driveTrain.trajectoryBuilder(Pose2)
                        .lineToLinearHeading(Pose3)
                        .build();
                scoreCone = driveTrain.trajectoryBuilder(Pose3)
                        .lineToLinearHeading(Pose4)
                        .build();
                goAwayCone = driveTrain.trajectoryBuilder(Pose4)
                        .lineToLinearHeading(Pose5)
                        .build();
                goToParking = driveTrain.trajectoryBuilder(Pose5)
                        .lineToLinearHeading(midWayPose)
                        .build();

                switch (cameraPosition) {
                    case ONE:
                        parkRobot = driveTrain.trajectoryBuilder(midWayPose)
                                .forward(PARK_DISTANCE )
                                .build();
                        break;
                    case TWO:
                        parkRobot = driveTrain.trajectoryBuilder(midWayPose)
                                //.forward(1)
                                .build();
                        break;
                    case THREE:
                        parkRobot = driveTrain.trajectoryBuilder(midWayPose)
                                .back(PARK_DISTANCE )
                                .build();
                        break;
                }
                break;
        }

        //Drop Preloaded Cone, Pick 5 cones and park


    }


    public void buildParkingFromMidwayPose(VisionEasyOpenCV.ParkingPosition position){
        //Build trajectory for parking based on the Camera color detected and position identified as LEFT, CENTER, RIGHT
        trajectoryParkingLEFT = driveTrain.trajectorySequenceBuilder(midWayPose)
                //.strafeLeft(STRAFE_DISTANCE + CAMERA_OFFSET)
                .forward(STRAFE_DISTANCE + CAMERA_OFFSET)
                .build();

        trajectoryParkingRIGHT = driveTrain.trajectorySequenceBuilder(midWayPose)
                //.strafeRight(STRAFE_DISTANCE)
                .back(STRAFE_DISTANCE)
                .build();

        trajectoryParkingCENTER = driveTrain.trajectorySequenceBuilder(midWayPose)
                .forward(1)
                .build();

     }

    public void buildParkingFromInitPose(VisionEasyOpenCV.ParkingPosition position){
        trajectoryParkingLEFT = driveTrain.trajectorySequenceBuilder(initPose)
                .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .forward(46)
                .lineToLinearHeading(midWayPose)
                .strafeLeft(STRAFE_DISTANCE + CAMERA_OFFSET)
                .build();

        trajectoryParkingRIGHT = driveTrain.trajectorySequenceBuilder(initPose)
                .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .forward(46)
                .lineToLinearHeading(midWayPose)
                .strafeRight(STRAFE_DISTANCE)
                .build();

        trajectoryParkingCENTER = driveTrain.trajectorySequenceBuilder(initPose)
                .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .forward(46)
                .lineToLinearHeading(midWayPose)
                .forward(1)
                .build();
    }

    //Run Auto trajectory and parking trajectory
    public void runAutoAndParking(VisionEasyOpenCV.ParkingPosition position){
        telemetry.setAutoClear(false);
        telemetry.addData("Running RoboRaiders Autonomous Mode adopted for Team:","21386");
        telemetry.addData("---------------------------------------","");
        telemetry.update();
        //Run the trajectory built for Auto and Parking
        driveTrain.followTrajectorySequence(trajectoryAutoNew);

        //Run trajectory based on camera direction detected

        switch (position) {
            case ONE: driveTrain.followTrajectorySequence(trajectoryParkingLEFT); break;
            case TWO: driveTrain.followTrajectorySequence(trajectoryParkingCENTER); break;
            case THREE: driveTrain.followTrajectorySequence(trajectoryParkingRIGHT);break;
        }
    }

    //Write a method which is able to pick the cone from the stack depending on your subsystems
    public void pickCone(int coneCount) {
        /*TODO: Add code to pick Cone 1 from stack*/
        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
    }

    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(int coneCount){
        /*TODO: Add code to drop cone on junction*/
        Claw.setPosition(.35);
        //ServoPosition += ServoSpeed;
        telemetry.addData("Opening","Claw is opening");

        if (coneCount == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", coneCount);
        }
        telemetry.update();
    }

    public void moveSlide(int slide_pos){
        /*Move slide to required position*/
        Slide.setTargetPosition(slide_pos);
        Slide.setPower(0.3);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Slide.isBusy()) {
            telemetry.addData("Current Position", Slide.getCurrentPosition());
            //telemetry.update();
        }
        telemetry.update();
    }

    public void parkingComplete(VisionEasyOpenCV.ParkingPosition position){
        Claw.setPosition(0);
        Slide.setTargetPosition(0);
        Slide.setPower(0.3);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Slide.isBusy()) {
            telemetry.addData("Current Position", Slide.getCurrentPosition());
        }
        telemetry.addData("Parked in Location", position);
        telemetry.update();
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing RoboRaiders Autonomous Mode adopted for Team:","21386");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }



}

