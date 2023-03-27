package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class IntakeOutElevatorDown extends CommandBase{
    private Intake m_intake;
    private Elevator m_Elevator;

    public IntakeOutElevatorDown(Intake intake, Elevator elevator){
        m_intake = intake;
        m_Elevator = elevator;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        System.out.println("Running Down");
        m_intake.SetSolenoid(true);

    }

    @Override
    public void end(boolean interrupted){
        m_intake.SetSolenoid(false);

    }

    @Override
    public boolean isFinished(){
        System.out.println("Done bottom");
        return m_Elevator.GetElevatorPos() < 500;
    }
    
    
}
