package pedroPathing;

import com.acmerobotics.dashboard.config.Config;
@Config
public class ConfigFile {
    public static double CONFIGkP = 0.005; //was 0.000016
    public static double CONFIGkI = 0;
    public static double CONFIGkD = 0.0000075; //was 0.0000071
    public static double CONFIGkS = 0.05;
    public static double CONFIGkV = 0.0003;
    public static double loopTime = 0.01; //10 msec
}