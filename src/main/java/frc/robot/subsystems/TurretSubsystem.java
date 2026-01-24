package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.PostTest;
import static org.fairportrobotics.frc.posty.assertions.Assertions.*;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.proto.SimpleMotorFeedforwardProto;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class TurretSubsystem extends TestableSubsystem {
    
    private final double kSeekDelta = 0.1;
    private final double kMinAngle = 0.0;
    private final double kMaxAngle = 270;

    private final double kOnTargetRange = 2.0;

    private TalonFX mRotationMotor;
    // TODO: Configure On device PID here

    // If we can't use a talon, use control loops here
    private SimpleMotorFeedforward mRotationFF = new SimpleMotorFeedforward(0, 0, 0);
    private PIDController mRotationPID = new PIDController(0, 0, 0);


    private TalonFX mHoodMotor;

    private NetworkTable mCameraTrackingTable;
    private PIDController mCameraTrackingPID = new PIDController(0, 0, 0);

    private boolean mTrackingEnabled = true;
    private TrackingState mTrackingState;

    public TurretSubsystem(){

        mRotationMotor = new TalonFX(Constants.TurretConstants.kRotationMotorId);
        mHoodMotor = new TalonFX(Constants.TurretConstants.kHoodMotorId);

        mCameraTrackingTable = NetworkTableInstance.getDefault().getTable(Constants.TurretConstants.kLimelightCameraName);

        // This controller should be trying to get the tx value as close to zero as possible
        mCameraTrackingPID.setSetpoint(0); 
    }

    @Override
    public void periodic() {
        super.periodic();

        if(mTrackingEnabled){

            // Values come from https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
            boolean targetInView = mCameraTrackingTable.getValue("tv").getInteger() >= 1 ? true : false;
            double tx = mCameraTrackingTable.getValue("tx").getDouble();
            double ty = mCameraTrackingTable.getValue("ty").getDouble();

            double requestPos = mRotationMotor.getPosition().getValueAsDouble();

            if(!targetInView){
                // We are seeking for an AprilTag
                // Pan around until we find one
                mTrackingState = TrackingState.SEEKING;
                requestPos += kSeekDelta;
            }else{
                // We have target in view
                if(Math.abs(tx) <= kOnTargetRange){
                    // The target is in view but we aren't centered on it
                    mTrackingState = TrackingState.TRACKING_ONTARGET;
                }else{
                    // The target is in view and we are centered on it
                    mTrackingState = TrackingState.TRACKING_OFFTARGET;
                }

                requestPos += mCameraTrackingPID.calculate(tx);

            }

            // Wrap around if we go above rotations range
            if(requestPos > kMaxAngle){
                requestPos = kMinAngle;
            }else if(requestPos < kMinAngle){
                requestPos = kMaxAngle;
            }

            mRotationMotor.setControl(new PositionVoltage(requestPos));

        }

    }

    public void enableTracking(){
        mTrackingEnabled = true;
    }


    public void disableTracking(){
        mTrackingEnabled = false;
        mTrackingState = TrackingState.DISABLED;
    }

    public TrackingState getmTrackingState(){
        return mTrackingState;
    }

    @PostTest(name = "Turret Rotation Motor Connected")
    public void test_RotationsMotorConnected(){
        assertThat(mRotationMotor.isConnected()).isTrue();
    }

    @PostTest(name = "Turret Hood Motor Connected")
    public void test_HoodMotorConnected(){
        assertThat(mHoodMotor.isConnected()).isTrue();
    }

    private enum TrackingState{
        DISABLED,
        SEEKING,
        TRACKING_ONTARGET,
        TRACKING_OFFTARGET
    }
}
