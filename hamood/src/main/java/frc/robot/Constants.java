// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
        //Detail Motor Components
    //every time when you are change port in RobotMap change it in Robot Characterization python file!!!!!!!

    //-------------------------------------------------------------------------------------------------------------\\
    public interface MotorPorts {
        public static final int
        chassisRightBack = 14, //Victor
        chassisRightFront = 15, //victor
        chassisLeftBack = 0, //Victor
        chassisLeftFront = 1, //Victor
        collector = 12; //Victor
    }
    //-------------------------------------------------------------------------------------------------------------\\
    //Detail Piston Components
    public interface PistonPorts {
    
        
    }
    //-------------------------------------------------------------------------------------------------------------\\
    
    //Detail Digital components
    public interface DigitalPorts {
    }
    //-------------------------------------------------------------------------------------------------------------\\
    public interface AnalogPorts{
    }
    //-------------------------------------------------------------------------------------------------------------\\
    
    //Detail Joysticks used
    public interface JoystickPorts {
        public static final int
            rightChassisJS = 0,
            leftChassisJS = 1,
            operatorJS = 2;        
    }
    //-------------------------------------------------------------------------------------------------------------\\

    //Detail the Buttons of each Joystick
    public interface ButtonPorts {
        public static final int 
            // Operator joystick buttons \\
            OPERATOR_JS_X = 1,
            OPERATOR_JS_A = 2,
            OPERATOR_JS_B = 3,
            OPERATOR_JS_Y = 4,
            OPERATOR_JS_LB = 5,
            OPERATOR_JS_RB = 6,
            OPERATOR_JS_LT = 7,
            OPERATOR_JS_RT = 8,
            OPERATOR_JS_Back = 9,
            OPERATOR_JS_Start = 10,
            OPERATOR_JS_LeftS = 11,
            OPERATOR_JS_RS =12,

            // Right joystick ports \\
            RIGHT_JS_COLLECTOR=0,

            // Left joystick ports \\
            LEFT_JS_REVERSE_CONTROLS = 0;  
    }
    //-------------------------------------------------------------------------------------------------------------\\
    //Detail Cameras used
    public interface CameraPorts {
        public static int 
        backCamera = 0;
        
    }
    //-------------------------------------------------------------------------------------------------------------\\
    // Math constants \\
    public interface MathConsts {

    public static final
    double DONT_KILL_THE_MOTORS = 0.4;
    }
}
