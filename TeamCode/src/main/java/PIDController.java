import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    double error;
    double lastError = 0;
    double integralSum = 0;

    double kP;
    double kI;
    double kD;

    public ElapsedTime pidTimer = new ElapsedTime();

    double loopTime;

    public PIDController(double kP, double kI, double kD, double loopTime) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.loopTime = loopTime;
    }

    public double runPID(double target, double current, double currentPower) {
        error = target - current;

        integralSum += error * loopTime;
        double derivative = (error - lastError) / loopTime;
        double deltaPower = (kP * error) + (kI * integralSum) + (kD * derivative);

        lastError = error;

        return currentPower + deltaPower;
    }
}
