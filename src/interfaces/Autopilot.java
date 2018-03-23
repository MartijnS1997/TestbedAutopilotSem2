package interfaces;

/**
 * Created by Martijn on 23/03/2018.
 */
public interface Autopilot {
    AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs inputs);
    AutopilotOutputs timePassed(AutopilotInputs inputs);
    void simulationEnded();
}