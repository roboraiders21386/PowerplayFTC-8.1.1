package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.drive.MecanumDrive.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
 * RoboRaisens # 21386 Autonomous using EasyOpenCV and Park only mode
 */
@Autonomous(name = "RoboRaiders ParkOnly Short", group = "00-Autonomous")
public class AutoOnlyPark extends LinearOpMode{

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    private VisionEasyOpenCV visionEasyOpenCV;
    private OpenCvCamera camera;
    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam1";
    public DriveTrain driveTrain;

    private  double STRAFE_DISTANCE = 26; //24;
    private double CAMERA_OFFSET = 3 ;
    private DcMotor Arm;
    private Servo Claw;
    private double CLAW_OPEN_POSITION = 0.32;
    private double CLAW_CLOSE_POSITION = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        /*Create your Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        visionEasyOpenCV = new VisionEasyOpenCV();
        Claw = hardwareMap.get(Servo.class, "Claw");


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

        while (!isStopRequested() && !opModeIsActive()) {
            //Run visionEasyOpenCV.getPosition() and keep watching for the identifier in the Signal Cone.
            telemetry.clearAll();
            telemetry.addData("Start RoboRaiders Autonomous Park Only Mode adopted for Team","21386");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Park Position Identified by Camera: ", visionEasyOpenCV.getPosition());
            telemetry.update();
        }
        //Set DEFAULT as ONE
        VisionEasyOpenCV.ParkingPosition position = VisionEasyOpenCV.ParkingPosition.ONE;

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {

            //Stop VisionEasyOpenCV process
            position = visionEasyOpenCV.getPosition();
            camera.stopStreaming();

            //Build parking trajectory based on last detected target by visionEasyOpenCV
            buildParking(position);
            driveTrain.getLocalizer().setPoseEstimate(initPose);

            //run Autonomous trajectory
            runParking(position);
        }

        //Trajectory is completed, display Parking complete
        parkingComplete(position);
    }

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryParking ;
    TrajectorySequence trajectoryParkingONE ;
    TrajectorySequence trajectoryParkingTHREE ;
    TrajectorySequence trajectoryParkingTWO ;

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    Pose2d midWayPose;
    Pose2d parkPose;
    Pose2d firstPose;

    Trajectory goToCone, goToCone2 ,scoreCone, goAwayCone;





    //Build parking trajectory based on target detected by visionEasyOpenCV
    public void buildParking(VisionEasyOpenCV.ParkingPosition position){
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-70, 32, Math.toRadians(0)); //Starting pose
                midWayPose = new Pose2d(-42, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                switch(position){
                    case ONE: parkPose = new Pose2d(-40, 60, Math.toRadians(0)); break; // Location 1
                    case TWO: parkPose = new Pose2d(-38, 35, Math.toRadians(0)); break; // Location 2
                    case THREE: parkPose = new Pose2d(-40, 11, Math.toRadians(0)); break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-70, -40, Math.toRadians(0));//Starting pose
                midWayPose = new Pose2d(-42, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                switch(position){
                    case ONE: parkPose = new Pose2d(-40, -11, Math.toRadians(0)); break; // Location 1
                    case TWO: parkPose = new Pose2d(-38, -35, Math.toRadians(0)); break; // Location 2
                    case THREE: parkPose = new Pose2d(-40, -60, Math.toRadians(0)); break; // Location 3
                }
                break;
            case RED_LEFT:
                initPose = new Pose2d(70, -32, Math.toRadians(180));//Starting pose
                midWayPose = new Pose2d(42, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                switch(position){
                    case ONE: parkPose = new Pose2d(40, -60, Math.toRadians(180)); break; // Location 1
                    case TWO: parkPose = new Pose2d(38, -36, Math.toRadians(180)); break; // Location 2
                    case THREE: parkPose = new Pose2d(40, -11, Math.toRadians(180)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                initPose = new Pose2d(70, 32, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(42, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                switch(position){
                    case ONE: parkPose = new Pose2d(40, 11, Math.toRadians(180)); break; // Location 1
                    case TWO: parkPose = new Pose2d(38, 35, Math.toRadians(180)); break; // Location 2
                    case THREE: parkPose = new Pose2d(40, 60, Math.toRadians(180)); break; // Location 3
                }
                break;
        }

        trajectoryParking = driveTrain.trajectorySequenceBuilder(initPose)
                .forward(46)
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(parkPose)
                .build();

        trajectoryParkingONE = driveTrain.trajectorySequenceBuilder(initPose)
                .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .forward(46)
                .lineToLinearHeading(midWayPose)
                .strafeLeft(STRAFE_DISTANCE + CAMERA_OFFSET)
                .build();

        trajectoryParkingTHREE = driveTrain.trajectorySequenceBuilder(initPose)
                .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .forward(46)
                .lineToLinearHeading(midWayPose)
                .strafeRight(STRAFE_DISTANCE)
                .build();

        trajectoryParkingTWO = driveTrain.trajectorySequenceBuilder(initPose)
                .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .forward(46)
                .lineToLinearHeading(midWayPose)
                .forward(1)
                .build();
    }

    //Run Auto trajectory and parking trajectory
    public void runParking(VisionEasyOpenCV.ParkingPosition position){
        telemetry.setAutoClear(false);
        telemetry.addData("Running RoboRaiders Autonomous Mode adopted for Team:","21386");
        telemetry.addData("---------------------------------------","");
        telemetry.update();
        //Run the trajectory built for Auto and Parking
        //driveTrain.followTrajectorySequence(trajectoryParking);

        //Run trajectory based on camera direction detected
        switch (position) {
            case ONE: driveTrain.followTrajectorySequence(trajectoryParkingONE); break;
            case TWO: driveTrain.followTrajectorySequence(trajectoryParkingTWO); break;
            case THREE: driveTrain.followTrajectorySequence(trajectoryParkingTHREE);break;
        }
    }

    public void parkingComplete(VisionEasyOpenCV.ParkingPosition position){
        Claw.setPosition(CLAW_CLOSE_POSITION);
        telemetry.addData("Parked in Location", position);
        telemetry.update();
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing RoboRaiders AutoOnlyPark for Team:","21386");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("    Blue LEFT   ", "(X)");
            telemetry.addData("    Blue RIGHT ", "(Y)");
            telemetry.addData("    Red LEFT    ", "(B)");
            telemetry.addData("    Red RIGHT  ", "(A)");
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

