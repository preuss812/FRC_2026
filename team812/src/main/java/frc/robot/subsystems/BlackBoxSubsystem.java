/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlackBoxSubsystem extends SubsystemBase {

	private static int flagBits = 0;
	private final boolean debug = false;


  private final Joystick m_BlackBox = new Joystick (OIConstants.kControlBox);
  /**
   * Creates a new BlackBoxSubsystem.
   */
  public BlackBoxSubsystem() {

  }

  @Override
  public void periodic() {
	// This method will be called once per scheduler run
	if (debug) printBits();	
  }
    // This function reads each button on the control box and sets 
    // a corresponding single bit in the integer variable flagBits
    // using bit shifts; e.g., << operator
    public void readBits() {
    	flagBits = 0;
    	for (int i = 1; i<=7; i++) {
    		if(m_BlackBox.getRawButton(i)) {
    			flagBits |= 1 << i;
    		}
    	}
    }
    
    // Creates a mask by bit shifting to the flag number
    // to be inspected and then logically ands that against
    // the flagBits and if the mask set is equal to the value
    // returned, then the bit is set.
    public boolean isSet(int flag) {
		int flagMask = 1 << flag;
		readBits();
    	if( (flagBits & flagMask) == flagMask ) {
    		return true;
    	} else {
    		return false;
    	}
    }
    public void printBits() {
    	readBits();
//		System.out.println("ControlBox bits: " + Integer.toBinaryString(flagBits));
	//	Robot.nttable.getEntry("Controlbox flagbits").setString(Integer.toBinaryString(flagBits));
    	String prespace = "";
//    	System.out.print("Switches: ");
    	for (int i = 1; i<=7; i++) {
//			System.out.print(prespace + i + "=" + (isSet(i) ? "on":"off"));
			String iString = String.format( prespace + i + "=" + (isSet(i) ? "on":"off") );
			String eString = String.format("ControlBox bit %d", i);
			if (debug) SmartDashboard.putString(eString, iString);
			//Robot.nttable.getEntry(eString).setString(iString);
    		prespace = " ";
		}	
		if (debug) SmartDashboard.putNumber("Pot 0:", getPotValueScaled(0,0.0,5.0));
		if (debug) SmartDashboard.putNumber("Pot 1:", getPotValueScaled(1,0.0,0.1));


    /*
    Robot.nttable.getEntry("ControlBox pot 0").setDouble(getPotValue(0));
		Robot.nttable.getEntry("ControlBox pot 1").setDouble(getPotValue(1));
		Robot.nttable.getEntry("ControlBox pot 0 scaled 0.0-5.0").setDouble(getPotValueScaled(0,0.0,5.0));
		Robot.nttable.getEntry("ControlBox pot 1 scaled 0.0-2.0").setDouble(getPotValueScaled(1, 0.0, 2.0));

		Robot.nttable.getEntry("ControlBox sw left").setString((isSwitchLeft() ? "true":"false"));
		Robot.nttable.getEntry("ControlBox sw center").setString((isSwitchCenter() ? "true":"false"));
    Robot.nttable.getEntry("ControlBox sw right").setString((isSwitchRight() ? "true":"false"));
    */

/*
    	System.out.println("");
    	System.out.println("ControlBox pot 0:  " + getPotValue(0));
    	System.out.println("ControlBox pot 1:  " + getPotValue(1));
    	
    	System.out.println("Three position switch status:");
    	System.out.println("\tleft: " + (isSwitchLeft() ? "true":"false"));
    	System.out.println("\tcenter: " + (isSwitchCenter() ? "true":"false"));
		System.out.println("\tright: " + (isSwitchRight() ? "true":"false"));
		*/
    }
    
    /*
    The three position switch on the external control box requires two bits
    The switches are 1 and 2
     1  |  2  | Position
    --------------------
     0     0    Left (or Out for FRC 2017)
     0     1    Center (or Off for FRC 2017)
     1     0    Don't care
     1     1    Right (or IN for FRC 2017)
     */
       
    public Boolean isSwitchLeft() {
    	return( ! isSet(1) && ! isSet(2) );
    }
       
    public Boolean isSwitchCenter() {
    	return( ! isSet(1) &&  isSet(2)) ;
    }
    
    public Boolean isSwitchRight() {
    	return( isSet(1) && isSet(2));
    }
       
    public double getPotValue(int axis) {
      return m_BlackBox.getRawAxis(axis);
	}
	
	public double getPotValueScaled(int axis, double to_min, double to_max) {
		double from_min = -1.0;
	 	double from_max = 1.0;
		double x;
		double scaled_x = 0.0;
		if( to_max > to_min  && from_max > from_min )
		{
			x =  m_BlackBox.getRawAxis(axis);
			scaled_x = ((x - from_min) * (to_max - to_min)) / 
						(from_max - from_min) +
						to_min;
		}
		return scaled_x;
	}
}
