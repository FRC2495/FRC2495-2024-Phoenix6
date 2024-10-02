package frc.robot.util;

public class Magic {
	
	public static double getEncoderCounts(double distance)
	{
		return -(-0.0011*distance*distance+0.5374*distance-3.9787); //-((-6.8622*distance*distance+1542.1*distance-17642)-5000)/2048; //divide by 2048 to convert from ticks to revolutions
	}
}

