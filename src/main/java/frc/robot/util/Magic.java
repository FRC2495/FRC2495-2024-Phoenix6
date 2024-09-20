package frc.robot.util;

public class Magic {
	
	public static double getEncoderCounts(double distance)
	{
		return -((-6.8622*distance*distance+1542.1*distance-17642)-5000)/1650; // /2048; //divide by 2048 to convert from ticks to revolutions
	}
}
