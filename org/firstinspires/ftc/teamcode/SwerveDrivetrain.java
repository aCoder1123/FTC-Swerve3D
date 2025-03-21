package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.SwerveModule;
import org.firstinspires.ftc.teamcode.SwerveUtil.ModuleState;

public class SwerveDrivetrain {
	private SwerveModule frontModule;
	private SwerveModule backModule;
	private double wheelSeparation = 0;
	public double wheelDiameter = 0;
	public double gearing = 0;
	public double maxSpeed = 0;

	public SwerveDrivetrain(SwerveModule frontModule, SwerveModule backModule) {
		this.frontModule = frontModule;
		this.backModule = backModule;
	}

	public SwerveDrivetrain withWheelSeparation(double wheelSeparation) {
		this.wheelSeparation = wheelSeparation;
		return this;
	}

	public SwerveDrivetrain withWheelDiameter(double wheelDiameter) {
		this.wheelDiameter = wheelDiameter;
		return this;
	}

	public SwerveDrivetrain withGearing(double gearing) {
		this.gearing = gearing;
		return this;
	}

	public SwerveDrivetrain withMaxSpeed(double maxSpeed) {
		this.maxSpeed = maxSpeed;
		return this;
	}

	public double getHeadingDegrees() {
		return 0;
	}

	public boolean initialized() {
		return frontModule != null && backModule != null;// && wheelSeparation != 0 && wheelDiameter != 0 && gearing !=
															// 0 && maxSpeed != 0;
	}

	public double[] normalizeRobotVelocity(double x, double y, double omega) {
		// double rotScalingFactor = 0.5;
		double[] normalizedVelocities = new double[] { x, y, omega };
		double desiredWheelSpeed = Math.sqrt(x * x + y * y) + omega;
		if (desiredWheelSpeed > 1) {
			for (int i = 0; i < 3; i++) {
				normalizedVelocities[i] = normalizedVelocities[i] * (1 / desiredWheelSpeed);
			}
		}

		return normalizedVelocities;
	}

	public ModuleState[] calculateModuleStates(double[] robotVelocity) {
		ModuleState[] states = new ModuleState[2];
		if (robotVelocity[0] == 0 && robotVelocity[1] == 0 && robotVelocity[2] == 0) {
			states[0] = new ModuleState(0, 0);
			states[1] = new ModuleState(0, 0);
			return states;
		}
		double v_rot = robotVelocity[2] * this.maxSpeed * 2 * Math.PI * (this.wheelSeparation / 2);
		double theta;

		if (robotVelocity[1] != 0) {
			theta = Math.atan(robotVelocity[0] / robotVelocity[1]);
		} else {
			theta = 0;
		}

		double wheel_1_x = robotVelocity[0] * this.maxSpeed + v_rot * Math.cos(theta);
		double wheel_1_y = robotVelocity[1] * this.maxSpeed + v_rot * Math.sin(theta);
		double wheel_1_v = Math.sqrt(wheel_1_y * wheel_1_y + wheel_1_x * wheel_1_x);
		double wheel_1_theta = Math.toDegrees(Math.atan(wheel_1_y / wheel_1_x));
		if (wheel_1_x == 0) {
			wheel_1_theta = 0;
		}
		states[0] = new ModuleState(wheel_1_v * this.gearing / (Math.PI * this.wheelDiameter),
				wheel_1_theta + this.getHeadingDegrees());

		double wheel_2_x = robotVelocity[0] * this.maxSpeed - v_rot * Math.cos(theta);
		double wheel_2_y = robotVelocity[1] * this.maxSpeed - v_rot * Math.sin(theta);
		double wheel_2_v = Math.sqrt(wheel_2_y * wheel_2_y + wheel_2_x * wheel_2_x);
		double wheel_2_theta = Math.toDegrees(Math.atan(wheel_2_y / wheel_2_x));
		if (wheel_2_x == 0) {
			wheel_2_theta = 0;
		}
		states[1] = new ModuleState(wheel_2_v * this.gearing / (Math.PI * this.wheelDiameter),
				wheel_2_theta + this.getHeadingDegrees());

		return states;
	}

	public void setModuleStates(ModuleState[] states) {
		if (!this.initialized())
			return;
		this.frontModule.setState(states[0]);
		// this.backModule.setState(states[1]);
	}

	public boolean driveWithJoysticks(double x, double y, double omega) {
		if (!this.initialized())
			return false;
		setModuleStates(calculateModuleStates(normalizeRobotVelocity(x, y, omega)));
		return true;
	}

}