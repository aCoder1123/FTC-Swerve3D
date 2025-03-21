package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SwerveUtil.ModuleState;

public class SwerveModule {
	private DcMotor motorOne;
	private DcMotor motorTwo;
	private AnalogInput encoder;
	private double offset;
	private double speedSetpoint = 0;
	private double angleSetpoint = 0;
	private double angleTolerance = 1;
	private double angleThreshold = 15;
	private double gearing;
	private double maxRPM = 6000;
	public ModuleState setpoint;
	private double turnP = .005;

	public SwerveModule(AnalogInput encoder, double encoderOffset, DcMotor motorOne, DcMotor motorTwo, double gearing) {
		this.motorOne = motorOne;
		this.motorTwo = motorTwo;
		this.encoder = encoder;
		this.offset = encoderOffset;
		this.gearing = gearing;
	}

	public AnalogInput getEncoder() {
		return encoder;
	}

	public double getAngle() {
		return encoder.getVoltage() * (360. / encoder.getMaxVoltage()) - offset;
	}

	public ModuleState getState() {
		return new ModuleState(this.maxRPM * this.gearing * (this.motorOne.getPower() - this.motorTwo.getPower()),
				this.getAngle());
	}

	public void runMotors(double motorOnePower, double motorTwoPower) {
		motorOne.setPower(motorOnePower);
		motorTwo.setPower(motorTwoPower);

	}

	public boolean setState(ModuleState state) {

		this.setpoint = state;// ModuleState.optimizeState(state, this.getState());
		// TODO optimize turning
		if (Math.abs(state.theta - this.getAngle()) < this.angleThreshold) {
			double power = state.rpm;// / this.maxRPM;
			if (Math.abs(state.theta - this.getAngle()) < this.angleTolerance) {
				this.runMotors(power, -power);
			} else {
				double diff = 0;
				diff = this.turnP * (state.theta - this.getAngle());
				if (diff > (1 - Math.abs(power))) {
					diff = (1 - Math.abs(power));
				}
				this.runMotors(power + diff, -power + diff);
			}
			return true;
		} else {
			double power = Math.copySign(1, state.theta - this.getAngle());
			this.runMotors(power, power);

			return false;
		}
	}
}
