package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

/**
 * RoboRaidersTeleOp Example
 *
 */
@TeleOp(name = "RoboRaiders TeleOp", group = "00-Teleop")
@Disabled
public class TeleOpModeSample extends LinearOpMode {

    public DriveTrain driveTrain;

    @Override
    /*
     * Constructor for passing all the subsystems in order to make the subsystem be able to use
     * and work/be active
     */
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);

        telemetry.clearAll();
        telemetry.addData("Running RoboRaiders TeleOp adopted for Team","21386");
        telemetry.update();
        /* Wait for Start or Stop Button to be pressed */
        waitForStart();

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {
            while (opModeIsActive()) {
                driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
                driveTrain.gamepadInput = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
                driveTrain.gamepadInputTurn = -gamepad1.right_stick_x;
                driveTrain.driveTrainPointFieldModes();
            }
        }
    }
}
