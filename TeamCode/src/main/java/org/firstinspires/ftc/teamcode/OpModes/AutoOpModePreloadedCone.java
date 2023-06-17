package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.VisionEasyOpenCV;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * RoboRaisers # 21386 Autonomous using EasyOpenCV & for dropping preloaded cone, picking and dropping 2 cones and park
 */
@Autonomous(name = "RoboRaiders Autonomous 1 Cone")
public class AutoOpModePreloadedCone extends LinearOpMode{
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


        //Build Autonomous trajectory to be used based on starting position selected
        buildAllianceAuto();
        driveTrain.getLocalizer().setPoseEstimate(initPose);

        while (!isStopRequested() && !opModeIsActive()) {
            //Run visionEasyOpenCV.getPosition() and keep watching for the identifier in the Signal Cone.
            telemetry.clearAll();
            telemetry.addData("Start RoboRaiders Autonomous Mode adopted for Team","21386");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Alliance Position", startPosition);
            telemetry.addData("Camera Detected: ", visionEasyOpenCV.getPosition());
            //telemetry.addData("Camera Detected: ", cameraPosition);
            telemetry.update();
        }
        //Set DEFAULT as LEFT
        VisionEasyOpenCV.ParkingPosition cameraPosition = VisionEasyOpenCV.ParkingPosition.ONE;

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) { //21386
            //Start is pressed

            //Stop VisionEasyOpenCV process
            cameraPosition = visionEasyOpenCV.getPosition();
            camera.stopStreaming();

            Claw.setPosition(CLAW_CLOSE_POSITION);

            //Build parking trajectory based on last detected target by visionEasyOpenCV
            buildParking(cameraPosition);

            //run Autonomous trajectory
            runAuto(Slide, Claw, cameraPosition);
        }

        //Trajectory is completed, display Parking complete
        parkingComplete(cameraPosition);
    }

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryParkingONE ;
    TrajectorySequence trajectoryParkingTWO ;
    TrajectorySequence trajectoryParkingTHREE ;

    TrajectorySequence trajectoryAutoNew ;

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    Pose2d midWayPose;
    Pose2d pickConePose;
    Pose2d dropConePose0, dropConePose1, dropConePose2;
    Pose2d parkPose;
    Pose2d dropConePose, Pose2, Pose3, Pose4, Pose5, Pose1_5;

    Trajectory goToCone, goToCone1_5;
    Trajectory goToCone2;
    Trajectory scoreCone;
    Trajectory goAwayCone;
    Trajectory goToParking;
    Trajectory parkRobot;

    Pose2d p0,p1, p2, p3;
    Trajectory t0,t1, t2, t3,  t4,  t5;

    public void runAuto(DcMotor Slide, Servo Gripper, VisionEasyOpenCV.ParkingPosition cameraPosition) {

        Gripper.setPosition(CLAW_CLOSE_POSITION);//Open Claw
        Slide.setTargetPosition(200);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(0.5);
        while (Slide.isBusy()) {
            telemetry.addData("Linear Slide Current Position: ", Slide.getCurrentPosition());
            telemetry.update();
        }
        driveTrain.followTrajectory(t0);
        driveTrain.followTrajectory(t1);
        driveTrain.followTrajectory(t2);

        //Run to high junction
        Slide.setTargetPosition(3900);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(0.5);
        while (Slide.isBusy()) {
            telemetry.addData("Linear Slide Current Position: ", Slide.getCurrentPosition());
            telemetry.update();
        }
        sleep(250);

        driveTrain.followTrajectory(t3);   //top of cone

        Gripper.setPosition(CLAW_OPEN_POSITION);//Open Claw

        sleep(250);
        driveTrain.followTrajectory(t4);

        Gripper.setPosition(CLAW_CLOSE_POSITION);
        Slide.setTargetPosition(0);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(0.5);

        while (Slide.isBusy()) {
            telemetry.addData("Linear Slide Current Position: ", Slide.getCurrentPosition());
            telemetry.update();
        }
        sleep(250);

        driveTrain.followTrajectory(t5);
        driveTrain.followTrajectory(parkRobot);
    }

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAllianceAuto() {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-64, 31, Math.toRadians(90)); //Starting pose
                midWayPose = new Pose2d(-12, 8, Math.toRadians(90));
                p0 = new Pose2d(-54, 31, Math.toRadians(90)); //Go back 1 tile
                p1 = new Pose2d(-54, 8, Math.toRadians(90)); //Go back 1 tile
                p2 = new Pose2d(4, 6, Math.toRadians(90)); // Come back inline to the High Junction
                p3 = new Pose2d(4, 11, Math.toRadians(90)); //Close to cone
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-64, -28, Math.toRadians(270));//Starting pose
                midWayPose = new Pose2d(-12, -8, Math.toRadians(270)); //Choose the pose to move forward towards signal cone
                p0 = new Pose2d(-58, -28, Math.toRadians(270)); //Go back 1 tile
                p1 = new Pose2d(-58, -6, Math.toRadians(270)); //Go back 1 tile
                p2 = new Pose2d(4, -6, Math.toRadians(270)); // Come back inline to the High Junction
                p3 = new Pose2d(4, -11, Math.toRadians(270)); //Close to cone


/*
                t0 = driveTrain.trajectoryBuilder(initPose)
                        .lineToLinearHeading(p0)
                        .build();
                t1 = driveTrain.trajectoryBuilder(t0.end())
                        .lineToLinearHeading(p1)
                        .build();
                t2 = driveTrain.trajectoryBuilder(t1.end())
                        .lineToLinearHeading(p2)
                        .build();
                t3 = driveTrain.trajectoryBuilder(t2.end())
                        .lineToLinearHeading(p3)
                        .build();

                t6 = driveTrain.trajectoryBuilder(t3.end())
                        .lineToLinearHeading(p2)
                        .build();

                t15 = driveTrain.trajectoryBuilder(t6.end())
                        .lineToLinearHeading(midWayPose)
                        .build();
*/
                break;
            case RED_LEFT:
                initPose = new Pose2d(64, -32, Math.toRadians(270));//Starting pose
                midWayPose = new Pose2d(12, -8, Math.toRadians(270)); //Choose the pose to move forward towards signal cone
                p0 = new Pose2d(58, -28, Math.toRadians(270)); //Go back 1 tile
                p1 = new Pose2d(58, -6, Math.toRadians(270)); //Go back 1 tile
                p2 = new Pose2d(-4, -6, Math.toRadians(270)); // Come back inline to the High Junction
                p3 = new Pose2d(-4, -11, Math.toRadians(270)); //Close to cone
                break;
            case RED_RIGHT:
                initPose = new Pose2d(64, 28, Math.toRadians(90)); //Starting pose
                midWayPose = new Pose2d(12, 8, Math.toRadians(90)); //Choose the pose to move forward towards signal cone
                p0 = new Pose2d(58, 28, Math.toRadians(90)); //Go back 1 tile
                p1 = new Pose2d(58, 6, Math.toRadians(90)); //Go back 1 tile
                p2 = new Pose2d(-4, 6, Math.toRadians(90)); // Come back inline to the High Junction
                p3 = new Pose2d(-4, 11, Math.toRadians(90)); //Close to cone
                break;
        }

        //Drop Preloaded Cone

        t0 = driveTrain.trajectoryBuilder(initPose)
                .lineToLinearHeading(p0)
                .build();
        t1 = driveTrain.trajectoryBuilder(t0.end())
                .lineToLinearHeading(p1)
                .build();
        t2 = driveTrain.trajectoryBuilder(t1.end())
                .lineToLinearHeading(p2)
                .build();
        t3 = driveTrain.trajectoryBuilder(t2.end())
                .lineToLinearHeading(p3)
                .build();

        t4 = driveTrain.trajectoryBuilder(t3.end())
                .lineToLinearHeading(p2)
                .build();

        t5 = driveTrain.trajectoryBuilder(t4.end())
                .lineToLinearHeading(midWayPose)
                .build();

    }

    public void buildParking(VisionEasyOpenCV.ParkingPosition cameraPosition){
        switch (startPosition) {
            case BLUE_LEFT:
                switch(cameraPosition){
                    case ONE: parkPose = new Pose2d(-10, 50, Math.toRadians(90)); break; // Location 1
                    case TWO: parkPose = new Pose2d(-10, 30, Math.toRadians(90)); break; // Location 2
                    case THREE: parkPose = new Pose2d(-10, 7, Math.toRadians(90)); break; // Location 3
                }
                break;
            case BLUE_RIGHT:

                switch(cameraPosition){
                    case ONE: parkPose = new Pose2d(-10, -7, Math.toRadians(270)); break; // Location 1
                    case TWO: parkPose = new Pose2d(-10, -30, Math.toRadians(270)); break; // Location 2
                    case THREE: parkPose = new Pose2d(-10, -50, Math.toRadians(270)); break; // Location 3
                }
                /*
                switch(cameraPosition) {

                    case ONE:
                        parkRobot = driveTrain.trajectoryBuilder(t15.end())
                                .lineToLinearHeading(new Pose2d(-12, -7, Math.toRadians(270)))
                                .build();
                        break;
                    case TWO:
                        parkRobot = driveTrain.trajectoryBuilder(t15.end())
                                .lineToLinearHeading(new Pose2d(-12, -31, Math.toRadians(270)))
                                .build();
                        break;
                    case THREE:
                        parkRobot = driveTrain.trajectoryBuilder(t15.end())
                                .lineToLinearHeading(new Pose2d(-12, -50, Math.toRadians(270)))
                                .build();
                        break;
                }
                */

                break;
            case RED_LEFT:
                switch(cameraPosition){
                    case ONE: parkPose = new Pose2d(10, -50, Math.toRadians(270)); break; // Location 1
                    case TWO: parkPose = new Pose2d(10, -30, Math.toRadians(270)); break; // Location 2
                    case THREE: parkPose = new Pose2d(10, -7, Math.toRadians(270)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                switch(cameraPosition){
                    case ONE: parkPose = new Pose2d(10, 7, Math.toRadians(90)); break; // Location 1
                    case TWO: parkPose = new Pose2d(10, 30, Math.toRadians(90)); break; // Location 2
                    case THREE: parkPose = new Pose2d(10, 50, Math.toRadians(90)); break; // Location 3
                }
                break;
        }

        parkRobot = driveTrain.trajectoryBuilder(t5.end())
                .lineToLinearHeading(parkPose)
                .build();
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

