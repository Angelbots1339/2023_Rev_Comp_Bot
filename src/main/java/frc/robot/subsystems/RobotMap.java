package frc.robot.subsystems;

    public class RobotMap {

        public static final double MAIN_PERIOD_S = 1.0/50.0; // Main loop 200Hz
    
        /*************************************************************************
         *                        ROBORIO WIRING MAP                             *
         *************************************************************************/
    
        // Joysticks///////////////////////////////////////////////////////////////
        public static final int DRIVER_JOYSTICK = 0;
        public static final int OPERATOR_JOYSTICK = 1;
        public static final int BUTTON_BOX_1 = 2;
        public static final int BUTTON_BOX_2 = 3;
        public static final int DRIVER_OPERATOR_E_BACKUP = 4;
        public static final int PID_TEST_JOYSTICK = 5;
    
    
    
        // Digital IO Channels//////////////////////////////////////////////////////
        // Channels 0-9 on RoboRio
        public static final int ENTRANCE_LINE_BREAK = 5; //TODO SET
        public static final int EXIT_LINE_BREAK = 6;
    
        //Channels 10-25 on MXP (PWM and DIO)
        public static final int PRACTICE_BOT_JUMPER = 24;
    
        //Analog Input Channels////////////////////////////////////////////////////
        //Channels 0-3 on Roborio
    
        // Channels 4-7 on MXP
    
    
        /*************************************************************************
        *                         Solenoids                                      *
        *************************************************************************/
        //Double Soldenoids PCM ID = 0 ///////////////////////////////////////////
        public static final int INTAKE_DISENGAGE_PCM = 0;
        public static final int INTAKE_ENGAGE_PCM = 1;
      
    
       
        
    
        /*************************************************************************
        *                         PDP/CAN DEVICES                                 *
        *************************************************************************/
      
        public static final int INTAKE_MOTOR_PDP = 5;
     
    
        //PCMs
        public static final int PCM_CAN_ID_BELLYPAN = 0;
        public static final int PCM_CAN_ID_SHOOTER = 1;
}
