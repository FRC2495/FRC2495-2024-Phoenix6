/**
 * 
 */
package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix6.StatusCode;

//import java.util.Timer;
//import java.util.TimerTask;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
/*import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;*/
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.*;
import frc.robot.RobotContainer;
//import frc.robot.Ports;

/**
 * The {@code Drawer} class contains fields and methods pertaining to the function of the drawer.
 */
public class Drawer extends SubsystemBase implements IDrawer {

	
	// general settings
	public static final int LENGTH_OF_TRAVEL_TICKS = 700000; // 700000; // TODO adjust as needed (halve for Talon FX)
	public static final int LENGTH_OF_MIDWAY_TICKS = 350000; // TODO adjust as needed (halve for Talon FX)

	static final double MAX_PCT_OUTPUT = 1.0;
	static final int WAIT_MS = 1000;
	static final int TIMEOUT_MS = 5000;

	static final int TALON_TIMEOUT_MS = 20;

	// move settings
	static final int PRIMARY_PID_LOOP = 0;
	
	static final int SLOT_0 = 0;
	
	static final double REDUCED_PCT_OUTPUT = 0.8; // 0.9;
	
	static final double MOVE_PROPORTIONAL_GAIN = 0.6; // 1.2 for SRX // TODO switch to 0.6 if required if switching to Talon FX (as encoder resolution is halved)
	static final double MOVE_INTEGRAL_GAIN = 0.0;
	static final double MOVE_DERIVATIVE_GAIN = 0.0;
	
	static final int TALON_TICK_THRESH = 512; // 128; //256
	static final double TICK_THRESH = 2048; // 512;
	public static final double TICK_PER_100MS_THRESH = 64; // about a tenth of a rotation per second 
	
	private final static int MOVE_ON_TARGET_MINIMUM_COUNT= 20; // number of times/iterations we need to be on target to really be on target

	private final static int MOVE_STALLED_MINIMUM_COUNT = MOVE_ON_TARGET_MINIMUM_COUNT * 2 + 30; // number of times/iterations we need to be stalled to really be stalled

	TalonFX drawer; 
	//BaseMotorController drawer_follower;

	DutyCycleOut drawerStopOut = new DutyCycleOut(0);
	DutyCycleOut drawerRedOut = new DutyCycleOut(REDUCED_PCT_OUTPUT);
	DutyCycleOut drawerMaxOut = new DutyCycleOut(MAX_PCT_OUTPUT);

	PositionDutyCycle drawerExtendPosition = new PositionDutyCycle(-LENGTH_OF_TRAVEL_TICKS);
	PositionDutyCycle drawerExtendMidwayPosition = new PositionDutyCycle(-LENGTH_OF_MIDWAY_TICKS);
	PositionDutyCycle drawerHomePosition = new PositionDutyCycle(0);
	
	boolean isMoving;
	boolean isExtending;
	boolean isReallyStalled;

	double tac;

	private int onTargetCount; // counter indicating how many times/iterations we were on target 
	private int stalledCount; // counter indicating how many times/iterations we were stalled
	
	
	public Drawer(TalonFX drawer_in/*, BaseMotorController drawer_follower_in*/) {
		
		drawer = drawer_in;
		//drawer_follower = drawer_follower_in;

		drawer.getConfigurator().apply(new TalonFXConfiguration());
		//drawer_follower.configFactoryDefault();
		
		// Mode of operation during Neutral output may be set by using the setNeutralMode() function.
		// As of right now, there are two options when setting the neutral mode of a motor controller,
		// brake and coast.
		TalonFXConfiguration drawerConfig = new TalonFXConfiguration();

		drawer.getConfigurator().apply(drawerConfig);

		//drawer.setNeutralMode(NeutralMode.Brake);
		drawerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		
		//drawer_follower.setNeutralMode(NeutralMode.Brake);
				
		// Sensor phase is the term used to explain sensor direction.
		// In order for limit switches and closed-loop features to function properly the sensor and motor has to be in-phase.
		// This means that the sensor position must move in a positive direction as the motor controller drives positive output.
		
		//drawer.setSensorPhase(false); // false for SRX // TODO switch to true if required if switching to Talon FX
		// When using a remote sensor, you can invert the remote sensor to bring it in phase with the Talon FX.
		
		//Enable forward limit switches
		drawerConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        drawerConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        drawerConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
		//drawer.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, TALON_TIMEOUT_MS);
		
		//Enable reverse limit switches
		drawerConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        drawerConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        drawerConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
		//drawer.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, TALON_TIMEOUT_MS);
		//drawer.overrideLimitSwitchesEnable(true);
	
		// Motor controller output direction can be set by calling the setInverted() function as seen below.
		// Note: Regardless of invert value, the LEDs will blink green when positive output is requested (by robot code or firmware closed loop).
		// Only the motor leads are inverted. This feature ensures that sensor phase and limit switches will properly match the LED pattern
		// (when LEDs are green => forward limit switch and soft limits are being checked).
		drawerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // change value or comment out if needed

		//drawer.setInverted(false);  // TODO switch to false if required if switching to Talon FX
		//drawer_follower.setInverted(true);  // TODO comment out if switching to Talon FX
		
		// Both the Talon SRX and Victor SPX have a follower feature that allows the motor controllers to mimic another motor controller's output.
		// Users will still need to set the motor controller's direction, and neutral mode.
		// The method follow() allows users to create a motor controller follower of not only the same model, but also other models
		// , talon to talon, victor to victor, talon to victor, and victor to talon.
		//drawer_follower.follow(drawer);

		// Motor controllers that are followers can set Status 1 and Status 2 to 255ms(max) using setStatusFramePeriod.
		// The Follower relies on the master status frame allowing its status frame to be slowed without affecting performance.
		// This is a useful optimization to manage CAN bus utilization.
		//drawer_follower.setStatusFramePeriod(StatusFrame.Status_1_General, 255, TALON_TIMEOUT_MS);
		//drawer_follower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255, TALON_TIMEOUT_MS);

		setPIDParameters();
		
		// use slot 0 for closed-looping
 		//drawer.selectProfileSlot(SLOT_0, PRIMARY_PID_LOOP);
		
		// set peak output to max in case if had been reduced previously
		setPeakOutputs(MAX_PCT_OUTPUT);

	
		// Sensors for motor controllers provide feedback about the position, velocity, and acceleration
		// of the system using that motor controller.
		// Note: With Phoenix framework, position units are in the natural units of the sensor.
		// This ensures the best resolution possible when performing closed-loops in firmware.
		// CTRE Magnetic Encoder (relative/quadrature) =  4096 units per rotation		
		// FX Integrated Sensor = 2048 units per rotation
		//drawer.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PRIMARY_PID_LOOP, TALON_TIMEOUT_MS); // .CTRE_MagEncoder_Relative for SRX // TODO switch to FeedbackDevice.IntegratedSensor if switching to Talon FX
		drawerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		// this will reset the encoder automatically when at or past the forward limit sensor
		/*drawer.configSetParameter(ParamEnum.eClearPositionOnLimitF, 1, 0, 0, TALON_TIMEOUT_MS);
		drawer.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, TALON_TIMEOUT_MS);*/
		drawerConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
		drawerConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;

		/* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = drawer.getConfigurator().apply(drawerConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
		
		isMoving = false;
		isExtending = false;
		isReallyStalled = false;
		stalledCount = 0;
	}
	
	@Override
	public void periodic() {
		// Put code here to be run every loop
	}

	// This method should be called to assess the progress of a move
	public boolean tripleCheckMove() {
		if (isMoving) {
			
			//double error = drawer.getClosedLoopError(PRIMARY_PID_LOOP);
			double error = drawer.getClosedLoopError().getValueAsDouble();
			
			boolean isOnTarget = (Math.abs(error) < TICK_THRESH);
			
			if (isOnTarget) { // if we are on target in this iteration 
				onTargetCount++; // we increase the counter
			} else { // if we are not on target in this iteration
				if (onTargetCount > 0) { // even though we were on target at least once during a previous iteration
					onTargetCount = 0; // we reset the counter as we are not on target anymore
					System.out.println("Triple-check failed (drawer moving).");
				} else {
					// we are definitely moving
				}
			}
			
			if (onTargetCount > MOVE_ON_TARGET_MINIMUM_COUNT) { // if we have met the minimum
				isMoving = false;
			}
			
			if (!isMoving) {
				System.out.println("You have reached the target (drawer moving).");
				//drawer.set(ControlMode.PercentOutput,0);
				if (isExtending)	{
					stop(); // adjust if needed
				} else {
					stop(); // adjust if needed
				}
			}
		}
		return isMoving; 
	}

	// return if drivetrain might be stalled
	public boolean tripleCheckIfStalled() {
		if (isMoving) {
			
			double velocity = getEncoderVelocity();
			
			boolean isStalled = (Math.abs(velocity) < TICK_PER_100MS_THRESH);
			
			if (isStalled) { // if we are stalled in this iteration 
				stalledCount++; // we increase the counter
			} else { // if we are not stalled in this iteration
				if (stalledCount > 0) { // even though we were stalled at least once during a previous iteration
					stalledCount = 0; // we reset the counter as we are not stalled anymore
					System.out.println("Triple-check failed (detecting stall).");
				} else {
					// we are definitely not stalled
					
					//System.out.println("moving velocity : " + velocity);
				}
			}
			
			if (isMoving && stalledCount > MOVE_STALLED_MINIMUM_COUNT) { // if we have met the minimum
				isReallyStalled = true;
			}
					
			if (isReallyStalled) {
				System.out.println("WARNING: Stall detected!");
				stop(); // WE STOP IF A STALL IS DETECTED				 
			}
		}
		
		return isReallyStalled;
	}

	public int getEncoderVelocity() {
		//return (int) (drawer.getSelectedSensorVelocity(PRIMARY_PID_LOOP));
		return (int) drawer.getVelocity().getValueAsDouble();
	}
	
	public void extend() {
		
		//setPIDParameters();
		System.out.println("Extending");
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		//tac = -LENGTH_OF_TRAVEL_TICKS;
		
		//drawer.set(ControlMode.Position,tac);
		drawer.setControl(drawerExtendPosition); //TODO fix
		
		isMoving = true;
		isExtending = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void extendMidway() {
		
		//setPIDParameters();
		System.out.println("Extending to Midway");
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		//tac = -LENGTH_OF_MIDWAY_TICKS;
		
		//drawer.set(ControlMode.Position,tac);
		drawer.setControl(drawerExtendMidwayPosition);
		
		isMoving = true;
		isExtending = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}
	
	public void retract() {
		
		//setPIDParameters();
		System.out.println("Retracting");
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		//tac = 0; // adjust as needed
		//drawer.set(ControlMode.Position,tac);
		drawer.setControl(drawerHomePosition);
		
		isMoving = true;
		isExtending = false;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public double getEncoderPosition() {
		//return drawer.getSelectedSensorPosition(PRIMARY_PID_LOOP);
		return drawer.getPosition().getValueAsDouble();
	}
	
	public void stay() {	 		
		isMoving = false;		
		isExtending = false;
	}

	public synchronized void stop() {
		//drawer.set(ControlMode.PercentOutput, 0);
		drawer.setControl(drawerStopOut);
		
		setPeakOutputs(MAX_PCT_OUTPUT); // we undo what me might have changed
		
		isMoving = false;
		isExtending = false;
	}
	
	private void setPIDParameters() {		
		//drawer.configAllowableClosedloopError(SLOT_0, TALON_TICK_THRESH, TALON_TIMEOUT_MS);
		
		// P is the proportional gain. It modifies the closed-loop output by a proportion (the gain value)
		// of the closed-loop error.
		// P gain is specified in output unit per error unit.
		// When tuning P, it's useful to estimate your starting value.
		// If you want your mechanism to drive 50% output when the error is 4096 (one rotation when using CTRE Mag Encoder),
		// then the calculated Proportional Gain would be (0.50 X 1023) / 4096 = ~0.125.
		
		// I is the integral gain. It modifies the closed-loop output according to the integral error
		// (summation of the closed-loop error each iteration).
		// I gain is specified in output units per integrated error.
		// If your mechanism never quite reaches your target and using integral gain is viable,
		// start with 1/100th of the Proportional Gain.
		
		// D is the derivative gain. It modifies the closed-loop output according to the derivative error
		// (change in closed-loop error each iteration).
		// D gain is specified in output units per derivative error.
		// If your mechanism accelerates too abruptly, Derivative Gain can be used to smooth the motion.
		// Typically start with 10x to 100x of your current Proportional Gain.

		// Feed-Forward is typically used in velocity and motion profile/magic closed-loop modes.
		// F gain is multiplied directly by the set point passed into the programming API.
		// The result of this multiplication is in motor output units [-1023, 1023]. This allows the robot to feed-forward using the target set-point.
		// In order to calculate feed-forward, you will need to measure your motor's velocity at a specified percent output
		// (preferably an output close to the intended operating range).

		var talonFXConfigs = new TalonFXConfiguration();

		// set slot 0 gains and leave every other config factory-default
		var slot0Configs = talonFXConfigs.Slot0;
		slot0Configs.kV = 0;
		slot0Configs.kP = MOVE_PROPORTIONAL_GAIN;
		slot0Configs.kI = MOVE_INTEGRAL_GAIN;
		slot0Configs.kD = MOVE_DERIVATIVE_GAIN;
		//slot0Configs.kS = SHOOT_DERIVATIVE_GAIN; //TODO change value (replacemet for nominal output)

		// apply all configs, 20 ms total timeout
		drawer.getConfigurator().apply(talonFXConfigs, TALON_TIMEOUT_MS);
		
		/*drawer.config_kP(SLOT_0, MOVE_PROPORTIONAL_GAIN, TALON_TIMEOUT_MS);
		drawer.config_kI(SLOT_0, MOVE_INTEGRAL_GAIN, TALON_TIMEOUT_MS);
		drawer.config_kD(SLOT_0, MOVE_DERIVATIVE_GAIN, TALON_TIMEOUT_MS);
		drawer.config_kF(SLOT_0, 0, TALON_TIMEOUT_MS);*/
	}
	
	// NOTE THAT THIS METHOD WILL IMPACT BOTH OPEN AND CLOSED LOOP MODES
	public void setPeakOutputs(double peakOutput)
	{
		/*drawer.configPeakOutputForward(peakOutput, TALON_TIMEOUT_MS);
		drawer.configPeakOutputReverse(-peakOutput, TALON_TIMEOUT_MS);
		
		drawer.configNominalOutputForward(0, TALON_TIMEOUT_MS);
		drawer.configNominalOutputReverse(0, TALON_TIMEOUT_MS);*/
	}
	
	public synchronized boolean isMoving() {
		return isMoving;
	}

	public synchronized boolean isExtending() {
		return isExtending;
	}

	public boolean isExtended() {
		return Math.abs(getEncoderPosition()) > LENGTH_OF_TRAVEL_TICKS * 9/10;
	}
	
	public boolean isRetracted() {
		return Math.abs(getEncoderPosition()) < LENGTH_OF_TRAVEL_TICKS * 1/10;
	}
	
	public boolean isMidway() {
		return !isExtended() && !isRetracted();
	}

	public boolean isDangerous() {
		return isRetracted();
	}

	// return if stalled
	public boolean isStalled() {
		return isReallyStalled;
	}
	
	// for debug purpose only
	public void joystickControl(Joystick joystick)
	{
		if (!isMoving) // if we are already doing a move we don't take over
		{
			//drawer.set(ControlMode.PercentOutput, +joystick.getY()); // adjust sign if desired
			//drawer.setControl(drawerMaxOut, +joystick.getY());
			drawer.setControl(drawerRedOut.withOutput(+joystick.getY()));

		}
	}

	public void gamepadControl(XboxController gamepad)
	{
		if (!isMoving) // if we are already doing a move we don't take over
		{
			//drawer.set(ControlMode.PercentOutput, -MathUtil.applyDeadband(gamepad.getRightX(),RobotContainer.GAMEPAD_AXIS_THRESHOLD)*0.6/*0.7*/); // adjust sign if desired
			drawer.setControl(drawerRedOut.withOutput(-MathUtil.applyDeadband(gamepad.getRightX(),RobotContainer.GAMEPAD_AXIS_THRESHOLD)*0.6/*0.7*/));
		}
	}

	public double getTarget() {
		return tac;
	}	

	public boolean getForwardLimitSwitchState() {
		//return drawer.getSensorCollection().isFwdLimitSwitchClosed();
		return drawer.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
	}

	public boolean getReverseLimitSwitchState() {
		//return drawer.getSensorCollection().isRevLimitSwitchClosed();
		return drawer.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
	}

	// MAKE SURE THAT YOU ARE NOT IN A CLOSED LOOP CONTROL MODE BEFORE CALLING THIS METHOD.
	// OTHERWISE THIS IS EQUIVALENT TO MOVING TO THE DISTANCE TO THE CURRENT ZERO IN REVERSE! 
	public void resetEncoder() {
		//drawer.set(ControlMode.PercentOutput,0); // we stop AND MAKE SURE WE DO NOT MOVE WHEN SETTING POSITION
		drawer.setControl(drawerStopOut);
		//drawer.setSelectedSensorPosition(0, PRIMARY_PID_LOOP, TALON_TIMEOUT_MS); // we mark the virtual zero
		drawer.setPosition(0, TALON_TIMEOUT_MS);
	}

}
