package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.util.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.SwerveModule;

@TeleOp(name = "Drive With Joysticks", group = "Linear OpMode")

public class SwerveRobot extends LinearOpMode {

	private ElapsedTime runtime = new ElapsedTime();
	private AnalogInput frontEncoder;
	private AnalogInput backEncoder;
	private DcMotor fr;
	private DcMotor fl;
	private DcMotor br;
	private DcMotor bl;

	public SwerveModule frontSwerveModule;
	public SwerveModule backSwerveModule;

	public SwerveDrivetrain drivetrain;

	@Override
	public void runOpMode() {
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		fr = hardwareMap.get(DcMotor.class, "fr");
		fl = hardwareMap.get(DcMotor.class, "fl");
		br = hardwareMap.get(DcMotor.class, "br");
		bl = hardwareMap.get(DcMotor.class, "bl");

		frontEncoder = hardwareMap.get(AnalogInput.class, "backEncoder");
		backEncoder = hardwareMap.get(AnalogInput.class, "frontEncoder");

		frontSwerveModule = new SwerveModule(frontEncoder, DrivetrainConstants.frontEncoderOffset, fr, fl,
				DrivetrainConstants.gearing);
		backSwerveModule = new SwerveModule(backEncoder, DrivetrainConstants.backEncoderOffset, br, bl,
				DrivetrainConstants.gearing);

		drivetrain = new SwerveDrivetrain(frontSwerveModule, backSwerveModule)
				.withGearing(DrivetrainConstants.gearing)
				.withMaxSpeed(DrivetrainConstants.maxSpeed)
				.withWheelDiameter(DrivetrainConstants.wheelDiameter)
				.withWheelSeparation(DrivetrainConstants.wheelSeparation);

		waitForStart();
		runtime.reset();

		telemetry.addData("Status", "Running");
		telemetry.update();

		while (opModeIsActive()) {
			// frontSwerveModule.runMotors(-this.gamepad1.left_stick_y,
			// -this.gamepad1.left_stick_x);
			// backSwerveModule.runMotors(-this.gamepad1.right_stick_y,
			// -this.gamepad1.right_stick_x);
			drivetrain.driveWithJoysticks(this.gamepad1.left_stick_y, -this.gamepad1.left_stick_x,
					-this.gamepad1.right_stick_x);

			telemetry.addData("FrontModuleState", frontSwerveModule.getState());
			telemetry.addData("FrontModuleSetpoint", frontSwerveModule.setpoint);
			telemetry.addData("BackModule", backSwerveModule.getEncoder().getVoltage());
			telemetry.addData("BackModuleSetpoint", backSwerveModule.setpoint);
			telemetry.update();

		}
	}

}
