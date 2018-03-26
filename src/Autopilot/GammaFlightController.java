package Autopilot;

import Exceptions.NoCubeException;
import Helper.SquareMatrix;
import interfaces.*;
import static java.lang.Math.*;

//TODO base the incrementation of the wings on the current outputs instead of the previous
/**
 * Created by Martijn on 19/02/2018.
 * A flight controller made for the 15Â° AOA assignment
 * TODO: Implement the controller fully
 */
public class GammaFlightController extends AutoPilotFlightController {

    public GammaFlightController(AutoPilot autoPilot) {
        super(autoPilot);
        System.out.println("using gamma controller");
    }


    public ControlOutputs getControlActions(AutopilotInputs_v2 inputs){
        this.setCurrentInputs(inputs);
        ControlOutputs outputs = new ControlOutputs();
        AutoPilotCamera APCamera = this.getAutopilot().getAPCamera();
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();
        PIDController xPIDController = this.getxPID();
        PIDController yPIDController = this.getyPID();

        APCamera.loadNewImage(currentInputs.getImage());
        float elapsedTime = this.getCurrentInputs().getElapsedTime();

        Vector center;

        try{
            center = APCamera.getCenterOfNCubes(1);
            System.out.println("CubeCenter: " + center);
        }catch(NoCubeException e){
            center = new Vector(-10, 0, 4);
        }

        //center = rollCorrectCenterCubes(inputs, center);

        //FOR DEBUGGING
        //System.out.println(center);
        //END FOR DEBUGGING

        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
        float xPosition = xPIDController.getPIDOutput(-center.getxValue(), deltaTime);
        float yPosition = yPIDController.getPIDOutput(center.getyValue(), deltaTime);
        int nbColumns = APCamera.getNbColumns();
        int nbRows = APCamera.getNbRows();
        float cubeCoeff = (float) min(MAX_CUBE_COEFF, sqrt(nbRows*nbColumns)/center.getzValue());
        //System.out.println("PID positions x= " + xPosition + " ; y= " + yPosition);
        //System.out.println("Cube coefficients: " + cubeCoeff);
        xControlActions(outputs, xPosition,cubeCoeff);
        yControlActions(outputs, yPosition, cubeCoeff, currentInputs.getPitch());
        setThrustOut(outputs, cubeCoeff);

        //System.out.println("Outputs Horizontal: " + outputs.getHorStabInclination()*RAD2DEGREE + "; Vertical: " + outputs.getVerStabInclination()*RAD2DEGREE );

        rollControl(outputs, this.getCurrentInputs());
        angleOfAttackControl(outputs, this.getPreviousInputs(), this.getCurrentInputs());

        return outputs;
    }

    /**
     * Corrects the inputs from the camera for roll effects from piloting the drone
     * @param inputs the inputs used to extract the current roll of the drone
     */
    private Vector rollCorrectCenterCubes(AutopilotInputs_v2 inputs, Vector currentCoord){
        //get the roll from the inputs
        float roll = inputs.getRoll();
        System.out.println("Current roll: " + roll);
        //make the transformation matrix
        SquareMatrix rollMatrix = new SquareMatrix(new float[]{(float) cos(roll), (float) -sin(roll), 0,
                (float) sin(roll), (float) cos(roll) , 0,
                0               ,  0                , 1});
        //then transform the inputs back to the original:
        Vector correctedVector = rollMatrix.matrixVectorProduct(currentCoord);
        //save the corrected form
        return correctedVector;
        //return
    }

    @Override
    protected void rollControl(Controller.ControlOutputs outputs, AutopilotInputs_v2 currentInput){
        float roll = currentInput.getRoll();

        if(roll >= this.getRollThreshold()&&isSteeringLeft(outputs)){
            outputs.setRightWingInclination(this.getMainStableInclination());
            outputs.setLeftWingInclination(this.getMainStableInclination());
        }
        else if(roll <= - this.getRollThreshold()&&isSteeringRight(outputs)){
            outputs.setLeftWingInclination(this.getMainStableInclination());
            outputs.setRightWingInclination(this.getMainStableInclination());
        }else{
            // change nothing
        }
    }

    private boolean isSteeringRight(Controller.ControlOutputs outputs){
        return outputs.getRightWingInclination() < this.getMainStableInclination();
    }

    private boolean isSteeringLeft(Controller.ControlOutputs outputs){
        return outputs.getRightWingInclination() > this.getMainStableInclination();
    }

    private void xControlActions(ControlOutputs outputs, float xPos, float cubeCoeff){
        float horizontalStabIncl = STABILIZER_STABLE_INCLINATION;
        float rightMainIncl = MAIN_STABLE_INCLINATION;
        float leftMainIncl = MAIN_STABLE_INCLINATION;
        float roll = this.getCurrentInputs().getRoll();
        if(xPos > X_THRESHOLD){
            // cube coeff: to increase pitch for faraway objects
            // squared for large corrections if large error
            horizontalStabIncl = (float) (signum(xPos) * min(MAX_HOR_STAB_INCLINATION, STANDARD_VER_STAB_INCL*pow(abs(xPos)/1f,2))); //*cube coeff
            leftMainIncl = (float) (signum(xPos)*sqrt(abs(roll))* TURNING_INCLINATION +  MAIN_STABLE_INCLINATION);
            rightMainIncl = 0;
        }else if(xPos < X_THRESHOLD*(signum(xPos))){
            horizontalStabIncl = (float) (signum(xPos) * min(MAX_HOR_STAB_INCLINATION, STANDARD_VER_STAB_INCL*pow(abs(xPos)/1f,2))); //*cube coeff
            rightMainIncl = (float) (-signum(xPos)*sqrt(abs(roll))*TURNING_INCLINATION +  MAIN_STABLE_INCLINATION);
            leftMainIncl = 0;
        }
        outputs.setHorStabInclination(horizontalStabIncl);
        outputs.setRightWingInclination(rightMainIncl);
        outputs.setLeftWingInclination(leftMainIncl);
    }

    private void yControlActions(ControlOutputs outputs, float yPos, float cubeCoeff, float pitch){
        float horizontalStabIncl;
        //TODO verify if this works
        yPos = (float) (yPos*(1+pitch*2/PI));
        if(abs(yPos) > Y_THRESHOLD){
            horizontalStabIncl = (float) (-signum(yPos) * min(MAX_HOR_STAB_INCLINATION, STANDARD_HOR_STAB_INCLINATION*cubeCoeff*pow(abs(yPos)/1f,2)));
        }else{
            horizontalStabIncl = STABILIZER_STABLE_INCLINATION;
        }
        outputs.setHorStabInclination(horizontalStabIncl);
    }

    private void setThrustOut(ControlOutputs outputs, float cubeCoeff){
        //Todo implement: write the output to the outputs
        float pitch = this.getCurrentInputs().getPitch();
        float maxThrust =  this.getAutopilot().getConfig().getMaxThrust();
        int threshold = Math.round(THRESHOLD_DISTANCE);
        float gravity = this.getAutopilot().getConfig().getGravity();

        // Thrust
        float thrust = (float) ((maxThrust/4) + THRUST_FACTOR*this.getTotalMass()*gravity*cubeCoeff);
        //System.out.println("thrust: " + thrust);
        outputs.setThrust(Math.max(Math.min(thrust, maxThrust), 0));
        if (getVelocityApprox().getzValue() < -50.f || pitch < 0){
            outputs.setThrust(0f);
        }
    }
    /**
     * Calculate an approximation of the velocity
     * @return the approximation of the velocity
     * elaboration: see textbook numerical math for derivative methods, the
     * derivative of f(k+1) - f(k-1) / (2*timeStep) has O(hÂ²) correctness
     */
    public Vector getVelocityApprox(){
        //get the inputs at moment k - 1 for the derivative
        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();
        //get the inputs at moment k
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        float prevTime = prevInputs.getElapsedTime();
        float currentTime = currentInputs.getElapsedTime();

        Vector prevPos = extractPosition(prevInputs);
        Vector currentPos = extractPosition(currentInputs);

        Vector posDiff = currentPos.vectorDifference(prevPos);
        float timeDiff = currentTime - prevTime;

        return posDiff.scalarMult(1/timeDiff);
    }
    /*
    Getters and setters
     */
    @Override
    protected float getMainStableInclination() {
        return MAIN_STABLE_INCLINATION;
    }
    @Override
    protected float getStabilizerStableInclination() {
        return STABILIZER_STABLE_INCLINATION;
    }
    @Override
    protected float getRollThreshold() {
        return ROLL_THRESHOLD;
    }
    @Override
    protected float getInclinationAOAErrorMargin() {
        return ERROR_INCLINATION_MARGIN;
    }

    public PIDController getxPID() {
        return xPID;
    }

    public PIDController getyPID() {
        return yPID;
    }

    private PIDController xPID = new PIDController(1.f, 0.0f, 0.0f);
    private PIDController yPID = new PIDController(1.f, 0.05f, 0.1f);
    // private PIDController rollPID = new PIDController(1f, 0.0f, 0.0f);


    //private static final float STANDARD_INCLINATION = (float) (5*PI/180);
    public  static final float MAIN_STABLE_INCLINATION = (float) (8*PI/180);//TODO Deze value is brak AF
    //public  static final float MAIN_MAX_INCLINATION = (float) (10*PI/180);
    private static final float MAX_HOR_STAB_INCLINATION = (float) (10*PI/180);
    private static final float STANDARD_HOR_STAB_INCLINATION = (float) (5*PI/180);
    //private static final float MAX_VER_STAB_INCLINATION = (float) (10*PI/180f);
    private static final float STANDARD_VER_STAB_INCL = (float) (5*PI/180f);
    private static final float TURNING_INCLINATION = (float) (10*PI/180);
    private static final float ERROR_INCLINATION_MARGIN = (float) (2*PI/180);
    //private static final int   BIAS = 0;
    private static final float THRESHOLD_DISTANCE = 1f;
    //private static final float STANDARD_THRUST = 32.859283f*2;
    private static final float THRUST_FACTOR = 2.0f;
    // private static final float THRESHOLD_THRUST_ANGLE = (float)(PI/20);
    private static final float MAX_CUBE_COEFF = 3f;
    public  static final float STABILIZER_STABLE_INCLINATION = 0.0f;
    //private static final float GRAVITY = 9.81f;
    private static final float ROLL_THRESHOLD = (float) (PI * 4f/180.0f);
    //private static final float RAD2DEGREE = (float) (180f/ PI);
    //private static final float CHECK_INTERVAL = 1/20.f;
    private static final float X_THRESHOLD = 0f;
    private static final float Y_THRESHOLD = 0f;
}
