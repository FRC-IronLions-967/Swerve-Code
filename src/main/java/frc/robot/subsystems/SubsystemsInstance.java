package frc.robot.subsystems;

public class SubsystemsInstance {
    public Drivetrain drivetrain;
   
   
    private static SubsystemsInstance inst;

    private SubsystemsInstance() {
        drivetrain = new Drivetrain();
       

    }

    public static SubsystemsInstance getInstance () {
        if(inst == null) inst = new SubsystemsInstance();

        return inst;

    }
    
}