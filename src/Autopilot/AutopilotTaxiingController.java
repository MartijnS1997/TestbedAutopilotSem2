package Autopilot;

import Physics.PhysXEngine;
import interfaces.*;

import static java.lang.Math.*;

/**
 * Taxiing Controller
 * finite state machine for the controller
 * startTurn --> full brake --> taxiingToTarget --> full brake
 */
public class AutopilotTaxiingController extends Controller {

    public AutopilotTaxiingController(AutoPilot autopilot){
        super(autopilot);
    }

    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs) {
        //set the current inputs
        this.setCurrentInputs(inputs);
        //set the current outputs
        ControlOutputs outputs = new ControlOutputs();

        this.getControlActionsActiveState(outputs);
        //maybe add AOA?

        return outputs;

    }

    private void getControlActionsActiveState(ControlOutputs outputs){
        TaxiingState state = this.getTaxiingState();
        switch (state){
            case INIT_TURN:
                initialTurn(outputs);
                break;
            case FULL_BRAKE:
                fullBrake(outputs);
                break;
            case MOVING_TO_TARGET:
                moveToTarget(outputs);
                break;
        }
    }

    /*
    Initial turn constants
     */

    /**
     * The angle between the heading of the drone and the vector between the drone and the target
     * that has to be reached before the full brake may be initialized
     */
    private final static float MAX_ANGLE_ERROR = (float) (2*PI/180);
    private final static float THRUST_RATIO = 0.125f;

    /**
     * Getter for the control actions for the phase where we take out initial turn
     * @param outputs the outputs to write the control actions to
     */
    private void initialTurn(ControlOutputs outputs){

        Vector target = this.getTarget();
        AutopilotInputs_v2 inputs = this.getCurrentInputs();
        float maxBrake = this.getConfig().getRMax();
        float maxThrust = this.getConfig().getMaxThrust();
        //calculate the angle between the heading vector and the target vector
        float angle = angleToTarget(target, inputs);
//        System.out.println("Angle between heading and diff: " + angle);
//        System.out.println();
        //check if we've reached our target
        if(checkHasFinishedTurn(angle)){
            //invoke the next controller
            this.setTaxiingState(TaxiingState.FULL_BRAKE);
            this.fullBrake(outputs);
            return;
        }
        //if not, continue normal procedure

        // if the angle is negative we need to steer to the right if it is positive we need to steer to the left
        // to go left we should activate the left brake, to go right activate the right brake
        if(angle > 0){
            //we need to go to the left
            outputs.setLeftBrakeForce(maxBrake);
        }else{
            outputs.setRightBrakeForce(maxBrake);
        }
        //keep a steady and slow output a minimal thrust (1/8th of max)
        outputs.setThrust(maxThrust*THRUST_RATIO);

        //Todo set the brake forces accordingly
    }

    /**
     * Checks if the drone has finished its initial turn
     * @param angle the angle between the drone heading vector and the vector from the drone to the target
     */
    private boolean checkHasFinishedTurn(float angle){
        //the angle between the heading vector of the drone and the target is small enough, we may go to the next state
        //the full brake state
        if(MAX_ANGLE_ERROR >= abs(angle)){
            return true;
        }
        //else do nothing
        return false;
    }

    /*
    Full brake constants
     */
    private final static float BRAKE_THRUST = 0f;
    private final static float STANDSTILL_VELOCITY = 0.1f;

    /**
     * The controller used to brake to our full capabilities... nothing special, just braking
     * @param outputs the outputs to write our control actions to
     */
    private void fullBrake(ControlOutputs outputs){
//        System.out.println("Entered full brake ##########");
        //we just brake to the full extent
        float maxBrake = this.getConfig().getRMax();
        //check if we've come to a standstill && reached the target
        if(droneInStandstill() && targetReached(this.getCurrentInputs())){
            //do not change the inputs, we're finished
            return;
        }
        //if we've only came to a standstill, but we didn't reach anything, call the move controller
        if(droneInStandstill()){
            this.setTaxiingState(TaxiingState.MOVING_TO_TARGET);
            //invoke the moving to target controller
            this.moveToTarget(outputs);
            return;
        }
        //doing full brake
        //otherwise, continue with full brake
        outputs.setRightBrakeForce(maxBrake);
        outputs.setLeftBrakeForce(maxBrake);
        outputs.setFrontBrakeForce(maxBrake);
        outputs.setThrust(BRAKE_THRUST);
    }


    /**
     * Checks if the drone came to an (approximate) standstill
     * @return true if the total velocity of the drone is <= STANDSTILL_VELOCITY
     */
    private boolean droneInStandstill(){
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();

        //approx the velocity
        Vector approxVelocity = getVelocityApprox(prevInputs, currentInputs);
        //check if the size exceeds the minimum
        boolean hasReachedMinimum = approxVelocity.getSize() <= STANDSTILL_VELOCITY;
        if(hasReachedMinimum){
            return true;
        }

        //if not return false
        return false;
    }

    /*
  Moving to target constants
   */

    // reference is 5m/s
    private final static float REACHING_DISTANCE = 10f; // the distance we need to enter before we can call it a day
    // also added a bit of padding for the brakes

    private void moveToTarget(ControlOutputs outputs){

        //acquire the current inputs (needed for angle calculation)
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();
        //check if we are done with moving
        if(targetReached(currentInputs)){
            //if so, change the state and call the brake method, return afterwards
            this.setTaxiingState(TaxiingState.FULL_BRAKE);
            this.fullBrake(outputs);
            return;
        }
//        System.out.println("position: " + Controller.extractPosition(currentInputs));
        brakeTurningControls(outputs, currentInputs, prevInputs);
        cruiseControl(outputs, prevInputs,currentInputs);


    }

    private boolean targetReached(AutopilotInputs_v2 inputs){
        //extract the current state
        //check if the current target has been rached, this happens if we've got close enough
        //to the target
        Vector target = this.getTarget();
        //get the current pos
        Vector position = Controller.extractPosition(inputs);

        return position.distanceBetween(target) <=  REACHING_DISTANCE;
    }

    /**
     * The controller that regulates the velocity during the taxiing phase of the drone
     * @param outputs the outputs where the control actions are written to
     * @param prevInputs the previous inputs of the autopilot
     * @param currentInputs the current inputs of the autopilot
     * note: only invoke this function AFTER writing the brake controls, otherwise the behaviour
     *       of the drone is unknown
     */
    private void cruiseControl(ControlOutputs outputs, AutopilotInputs_v2 prevInputs, AutopilotInputs_v2 currentInputs){
        //we need to get an approx for the velocity before we can make any further calculations
        Vector approxVel = Controller.getVelocityApprox(prevInputs, currentInputs);
        float totalVelocityApprox = approxVel.getSize();
        //get the total mass of the drone (needed for the calculations)
        float totalMass = this.getTotalMass();
        //get the velocity controller
        PIDController velocityController = this.getVelocityController();
        //get the delta time, note this is an extrapolation, we assume the time step stays the same across the simulation
        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
        float errorVelocity = calcRefVelocity(currentInputs) - totalVelocityApprox;
//        System.out.println(approxVel);
        //now calculate the output of the PID
        float errorVelPID = velocityController.getPIDOutput(-errorVelocity, deltaTime);
//        System.out.println("error on velocity: "+ errorVelPID);
        //based on the error in the velocity we can calculate our control actions
        //if the error is positive we need to accelerate (setpoint - velocity = error)
        //if the error is negative we need to brake
        if(errorVelPID > 0){
//            System.out.println("Adjusting thrust");
            float maxThrust = this.getConfig().getMaxThrust();
            float thrust = getCorrectionThrust(totalMass, errorVelPID, maxThrust, deltaTime);
            outputs.setThrust(thrust);
        }else if(errorVelPID < 0){
//            System.out.println("Adjusting brakes");
            float maxBrake = this.getConfig().getRMax();
            Vector brakeVector = getCorrectionBrakes(totalMass, errorVelPID, maxBrake, deltaTime, outputs);
            setBrakeVector(outputs, brakeVector);
        }



        // we are finished
//        System.out.println(outputs);
//        System.out.println();
    }

    /**
     * Calculates the reference velocity used by the taxiing controller
     * slows down when closer to target
     * @param inputs the current inputs to extract the needed state data from
     * @return the velocity needed for the reference
     */
    private float calcRefVelocity(AutopilotInputs_v2 inputs){
        Vector target = this.getTarget();
        //extract the position
        Vector position = Controller.extractPosition(inputs);
        //get the distance to the target
        float distanceToTarget = position.distanceBetween(target);

        return distanceToTarget > LOW_VEL_RADIUS ? HIGH_REF_VELOCITY : LOW_REF_VELOCITY;
    }

    /**
     * The high reference velocity
     */
    private static float HIGH_REF_VELOCITY = 10f;
    /**
     * The low reference velocity
     */
    private static float LOW_REF_VELOCITY = 7.5f;
    /**
     * The radius before we go in low velocity mode
     */
    private static float LOW_VEL_RADIUS = 500f;

    /**
     * Calculates the corrective thrust action needed to keep the velocity up to level
     * @param totalMass the total mass, needed for the calcultations
     * @param deltaVelocity the error on the velocity
     * @param maxThrust the maximum thrust the drone can deliver
     * @param deltaTime the time difference
     * @return the thrust needed to make the error on the velocity zero, capped on the max thrust
     */
    private static float getCorrectionThrust(float totalMass, float deltaVelocity, float maxThrust, float deltaTime){
        //first calculate the thrust needed to reach the desired velocity
        float desiredThrust = deltaVelocity*totalMass/deltaTime;
        //now check if we can reach it
        return min(desiredThrust, maxThrust);

    }

    /**
     * Calculates the correction needed by the brakes to compensate for the error on the velocity
     * @param totalMass the total mass of the drone
     * @param deltaVelocity the velocity error
     * @param maxBrake the maximum brake force that the drone can exert on a tyre
     * @param deltaTime the time difference between two simulation steps
     * @param outputs the outptus that were already written before this method was invoked
     * @return a vector containing the brake forces needed to correct the erroneous velocity
     *         the front, left and right brake forces are the x, y and z components of the vector respectively
     */
    private static Vector getCorrectionBrakes(float totalMass, float deltaVelocity, float maxBrake, float deltaTime, ControlOutputs outputs){
        //TODO corrective forces of the brakes may override the forces calculated by the steering controller, maybe scale the brake force down until the calculated ones for turning are at cap
        //assumption, all the brakes carry the same brake force and this scalar may be divided by 3 to calc the force needed at each tyre
        float totalBrakeForce = -deltaVelocity*totalMass/deltaTime;//minus to make the result positive
        //now calc the brake force needed for each tyre and cap it to the max brake force
        float desiredTyreBrakeForce = min(totalBrakeForce/3f, maxBrake);


        //get the current brake vector
        Vector currentBrakeVector = extractBrakeVector(outputs);
        //cap the vector components
        return getCompatibleBrakeVector(maxBrake, desiredTyreBrakeForce, currentBrakeVector);
    }

    /**
     * note: ignore the whole documentation hogwash and read the example
     * Checks if the current desired brake force doesn't overwrite any of the control actions taken by the steering controls
     * @param maxBrake the maximum brake force
     * @param desiredTyreBrakeForce the desired brake force for the tyres
     * @param currentBrakeVector the brake vector currently stored in the outputs of the controller
     *                           front, left and right brake are the x, y and z components respectively
     * @return returns a new brake vector that is either the sum of the current brake force with the desired one
     * or a version where the desired brake force was capped in such a way that a uniform desired brake force vector
     * (with the same components on x,y,z) summed with the current brake vector produces the maximum brake force
     * for the largest component of the current brake force
     *
     * (eg if desired brake force is (1000,1000,1000) and current is (500,0,0)
     * with a maximum brake force of 1000, the resulting vector is (1000, 500, 500)
     */
    private static Vector getCompatibleBrakeVector(float maxBrake, float desiredTyreBrakeForce, Vector currentBrakeVector) {
        Vector desiredBrakeVector = new Vector(desiredTyreBrakeForce, desiredTyreBrakeForce, desiredTyreBrakeForce);
        //check if we will overwrite a previous control action
        Vector sumVector = currentBrakeVector.vectorSum(desiredBrakeVector);
        float maxVal = sumVector.getMaxComponent();
        //the difference between the max value of the sum and the maximum exertable brake force
        float diffBrakeForce = maxVal - maxBrake;
        //if the max value is larger than the max brake, we need to rescale
        if(maxVal > maxBrake){
            float modTyreBrakeForce = desiredBrakeVector.getzValue() - diffBrakeForce; // the size of the brake vector at z is equal to the break force exerted on the tyre (in ideal circumstances)
            Vector modBrakeVector = new Vector(modTyreBrakeForce, modTyreBrakeForce, modTyreBrakeForce);
            sumVector = modBrakeVector.vectorSum(currentBrakeVector);
        }
        return sumVector;
    }

    /**
     * Extracts a vector of brake forces from the given outputs
     * @param outputs the outputs that are currently generated (may be overwritten later)
     * @return an immutable vector containing the front, left and right brake force as its x, y and z components
     */
    private static Vector extractBrakeVector(ControlOutputs outputs) {
        float currentFrontBrake = outputs.getFrontBrakeForce();
        float currentLeftBrake = outputs.getLeftBrakeForce();
        float currentRightBrake = outputs.getRightBrakeForce();
        return new Vector(currentFrontBrake, currentLeftBrake ,currentRightBrake );
    }

    /**
     * Writes the provided brake vector to the provided outputs, overwriting the previous outputs
     * @param outputs the outputs where the brake vector is written to
     * @param brakeVector a vector containing the brake forces, the front, left and right brake forces
     *                    are respectively the x, y and z components of the vector
     */
    private static void setBrakeVector(ControlOutputs outputs, Vector brakeVector){
        outputs.setFrontBrakeForce(brakeVector.getxValue());
        outputs.setLeftBrakeForce(brakeVector.getyValue());
        outputs.setRightBrakeForce(brakeVector.getzValue());
    }

    /**
     * The controller for the brakes while navigating for turning the drone to the target
     * @param outputs the outputs to write the control actions to
     * @param currentInputs the current outputs used to extract data
     * @param prevInputs the current inputs used to extract data
     */
    private void brakeTurningControls(ControlOutputs outputs, AutopilotInputs_v2 prevInputs, AutopilotInputs_v2 currentInputs) {
        //get the time difference between two steps, note that this is an extrapolation, we assume here that
        //the time between simulation steps stay the same
        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
        //get the max R:
        float maxBrake = this.getConfig().getRMax();
        //get the target
        Vector target = this.getTarget();
        //then we need to calculate our angle
        float angle = angleToTarget(target, currentInputs);
        //then get the PID controller
        PIDController brakeController = this.getBrakeController();
        //get the outputs
        float pidOutputs = brakeController.getPIDOutput(angle, deltaTime);
        //will give negative output if we need to go right, positive if we need to go to the left
//        System.out.println("brakePidOutputs " + pidOutputs);
        outputs.setLeftBrakeForce(min(abs(min(pidOutputs, 0)), maxBrake));
        outputs.setRightBrakeForce(min(abs(max(pidOutputs, 0)), maxBrake));
//        System.out.println("Turning " + (outputs.getRightBrakeForce() > 0? "right" : "left"));
    }

    /**
     * Calculates the angle between vector difference of the current target and the drone, and the drone's heading vector
     * positive angles indicate a need for steering to the left
     * and negative angles indicate a need for steering to the right
     * @param target the target of the controller
     * @return the angle between the target and the heading vector of the drone
     *         the sign of the angle indicates in which direction the controller needs to steer
     */
    private static float angleToTarget(Vector target, AutopilotInputs_v2 inputs){
        Vector position = Controller.extractPosition(inputs);
        Vector orientation = Controller.extractOrientation(inputs);
        //calculate the difference vector between the target and our current position
        Vector diffVector = target.vectorDifference(position);
        //now project the vector on the xz-plane && normalize
        Vector normal = new Vector(0,1,0); // the normal vector of the xz-plane
        Vector projDiffVector = diffVector.orthogonalProjection(normal); // project onto the plane

        //now calculate the current heading vector of the drone in the world axis system
        Vector headingDrone = new Vector(0,0,-1);
        //transform
        Vector headingWorld = PhysXEngine.droneOnWorld(headingDrone, orientation);
        //project
        Vector projHeadingWorld = headingWorld.orthogonalProjection(normal);

        //now get the angle between both
        float angle = abs(projHeadingWorld.getAngleBetween(projDiffVector));

//        System.out.println("Projected heading vector: " + projHeadingWorld);
//        System.out.println("Projected diff vector: " + projDiffVector);
        //calculate the direction, we use the vector product of the heading vector and the difference vector
        //a positive vector indicates that the target is located to the left, a negative angle indicates the
        //target is located at the right of the drone (the result is completely located on the y-axis, the normal)
        float direction = signum(projHeadingWorld.crossProduct(projDiffVector).scalarProduct(normal));
//        System.out.println("Direction: " + direction);

        float result = angle*direction;
        //check for NaN
        if(Float.isNaN(result)){
            return 0f;
        }

        return result;
    }

    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 inputs) {
        return false;
    }

    @Override
    protected float getMainStableInclination() {
        return 0;
    }

    @Override
    protected float getStabilizerStableInclination() {
        return 0;
    }

    @Override
    protected float getRollThreshold() {
        return 0;
    }

    @Override
    protected float getInclinationAOAErrorMargin() {
        return 0;
    }

    @Override
    protected float getStandardThrust() {
        return 0;
    }

    /**
     * Getter for the target of the taxiing controller
     * @return the target of the taxiing controller
     */
    private Vector getTarget() {
        return target;
    }

    /**
     * Setter for the target of the taxiing controller (for use in the controller selector only)
     * @param target the target to navigate to
     */
    public void setTarget(Vector target) {
        this.target = target;
    }

    /**
     * Getter for the controller that controls the brakes of the drone for navigating to our target
     * @return the controller used for steering the wheels
     */
    public PIDController getBrakeController() {
        return brakeController;
    }

    /**
     * Getter for the controller that controls the velocity of the drone for navigating to our target
     * @return the controller used to keep our velocity
     */
    public PIDController getVelocityController() {
        return velocityController;
    }

    /**
     * Getter for the current taxi state, the state of our finite state taxiing machine
     * @return the current taxiing state
     */
    private TaxiingState getTaxiingState() {
        return taxiingState;
    }

    /**
     * Setter for the current Taxiing state
     * @param taxiingState the taxiing state of the drone
     */
    private void setTaxiingState(TaxiingState taxiingState) {
        this.taxiingState = taxiingState;
    }

    /**
     * The target of the taxiing controller
     */
    //TODO remove standard value if the functionality of the controller is tested and the controller selector can make use of it
    private Vector target = new Vector(-500, 0,1000);

    /**
     * The reference velocity used by the autopilot cruise control
     */
    private float referenceVelocity = 5f;
    /**
     * The PID controller used to determine the force that needs to be exerted on the brakes
     */
    private final static float BRAKE_GAIN = 500;
    private final static float BRAKE_INTEGRAL = 0;
    private final static float BRAKE_DERIVATIVE =0;
    private PIDController brakeController = new PIDController(BRAKE_GAIN, BRAKE_INTEGRAL, BRAKE_DERIVATIVE);

    /**
     * The controller used to determine the velocity of the drone
     */
    private final static float VELOCITY_GRAIN = 1;
    private final static float VELOCITY_INTEGRAL = 0;
    private final static float VELOCITY_DERIVATIVE = 0;
    private PIDController velocityController = new PIDController(VELOCITY_GRAIN, VELOCITY_INTEGRAL, VELOCITY_DERIVATIVE);

    /**
     * The current state of the taxiing, first we need to take an initial turn so we are in the right
     * position to taxi to the target, we always start with an init turn
     */
    private TaxiingState taxiingState = TaxiingState.MOVING_TO_TARGET;


    private enum TaxiingState {
        INIT_TURN, FULL_BRAKE, MOVING_TO_TARGET
    }
}


