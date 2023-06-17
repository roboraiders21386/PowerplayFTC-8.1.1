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
@Autonomous(name = "RoboRaiders Autonomous Full Copy")
@Disabled
public class AutoOpModeCopy extends LinearOpMode{
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
    private DcMotor Arm;
    private Servo Claw;

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
        Arm = hardwareMap.get(DcMotor.class, "motor");
        Claw = hardwareMap.get(Servo.class, "Claw");
        double ServoPosition;
        double ServoSpeed;
        ServoPosition = 1;
        ServoSpeed = 1;
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);
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
            telemetry.update();
        }
        //Set DEFAULT as LEFT
        VisionEasyOpenCV.ParkingPosition position = VisionEasyOpenCV.ParkingPosition.ONE;

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) { //21386
            //Start is pressed

            //Stop VisionEasyOpenCV process
            position = visionEasyOpenCV.getPosition();
            camera.stopStreaming();

            Claw.setPosition(CLAW_CLOSE_POSITION);

            //Set the arm to ground junction height
            Arm.setTargetPosition(600);
            Arm.setPower(0.3);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Arm.isBusy()) {
                telemetry.addData("Current Position", Arm.getCurrentPosition());
                telemetry.addData("Target Position", Arm.getTargetPosition());
                telemetry.update();
            }

            //buildAllianceAuto();
            //Build parking trajectory based on last detected target by visionEasyOpenCV
            buildParkingFromMidwayPose(position);
            //buildParkingFromInitPose(position);


            //run Autonomous trajectory
            //runAutoAndParking(position);
            runAuto(Arm, Claw);
        }

        //Trajectory is completed, display Parking complete
        parkingComplete(position);
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
    Pose2d dropConePose, Pose2, Pose3, Pose4, Pose5;
    Pose2d firstPose;

    Trajectory goToCone, goToCone2, scoreCone, goAwayCone, goToParking;


    public void runAuto(DcMotor Slide, Servo Gripper) {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.followTrajectory(goToCone);
        driveTrain.followTrajectory(goToCone2);
        Slide.setTargetPosition(3200);
        Slide.setPower(0.6);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Slide.isBusy()) {
            telemetry.addData("Linear Slide Current Position: ", Slide.getCurrentPosition());
            telemetry.update();
        }
        driveTrain.followTrajectory(scoreCone);
        Gripper.setPosition(CLAW_OPEN_POSITION);//Open Claw
        sleep(1000);
        driveTrain.followTrajectory(goAwayCone);
        Gripper.setPosition(0);//Close Claw
        Slide.setTargetPosition(0); //Lower Lift
        Slide.setPower(0.6);
        while (Slide.isBusy()) {
            telemetry.addData("Linear Slide Current Position: ", Slide.getCurrentPosition());
            telemetry.update();
        }
        driveTrain.followTrajectory(goToParking);
    }

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAllianceAuto() {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-70, 32, Math.toRadians(0)); //Starting pose
                dropConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                midWayPose = new Pose2d(-42, 36, Math.toRadians(0));
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-72, -37, Math.toRadians(90));//Starting pose
                dropConePose = new Pose2d(-59, -48, Math.toRadians(90)); //B4 High Junction
                midWayPose = new Pose2d(-42, -36, Math.toRadians(90)); //Choose the pose to move forward towards signal cone
                Pose2 = new Pose2d(0, -36, Math.toRadians(90));
                Pose3 = new Pose2d(0, -32, Math.toRadians(90));
                Pose4 = new Pose2d(0, -29, Math.toRadians(90));
                Pose5 = new Pose2d(0, -36, Math.toRadians(90));

                goToCone = driveTrain.trajectoryBuilder(initPose)
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
                break;
            case RED_LEFT:
                initPose = new Pose2d(70, -32, Math.toRadians(180));//Starting pose
                dropConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                midWayPose = new Pose2d(42, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                break;
            case RED_RIGHT:
                initPose = new Pose2d(70, 32, Math.toRadians(180)); //Starting pose
                dropConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                midWayPose = new Pose2d(42, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                break;
        }

        //Drop Preloaded Cone, Pick 5 cones and park

        trajectoryAutoNew = driveTrain.trajectorySequenceBuilder(initPose)
                //.forward(12)
                //.strafeRight(10)
                .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(dropConePose)
                //.forward(14)
                .addDisplacementMarker(() -> {
                    Claw.setPosition(CLAW_OPEN_POSITION);
                    telemetry.addData("Claw Opening ServoPos:", Claw.getPosition());
                    telemetry.update();
                })
                .back(6)
                .addDisplacementMarker(() -> {
                    Claw.setPosition(0);
                    telemetry.addData("Claw Closing ServoPos:", Claw.getPosition());
                    telemetry.update();
                })
                .lineToLinearHeading(new Pose2d(-60, -37, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    Arm.setTargetPosition(0);
                    Arm.setPower(0.3);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (Arm.isBusy()) {
                        telemetry.addData("Current Position", Arm.getCurrentPosition());
                        telemetry.addData("Target Position", Arm.getTargetPosition());
                        telemetry.update();
                    }
                })
                //.forward(46)
                //.lineToLinearHeading(midWayPose)
                //.strafeLeft(28)
                .build();
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
        Arm.setTargetPosition(slide_pos);
        Arm.setPower(0.3);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Arm.isBusy()) {
            telemetry.addData("Current Position", Arm.getCurrentPosition());
            //telemetry.update();
        }
        telemetry.update();
    }

    public void parkingComplete(VisionEasyOpenCV.ParkingPosition position){
        Claw.setPosition(0);
        Arm.setTargetPosition(0);
        Arm.setPower(0.3);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Arm.isBusy()) {
            telemetry.addData("Current Position", Arm.getCurrentPosition());
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

