package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;

public class LemonLight {
    private static final String LIMELIGHT_TABLE_NAME = "limelight";
    
    //LED Modes
    public static final int LED_MODE_PIPELINE = 0;
    public static final int LED_MODE_OFF = 1;
    public static final int LED_MODE_BLINK = 2;
    public static final int LED_MODE_ON = 3;

    //Stream Modes
    public static final int STREAM_MODE_STANDARD = 0;
    public static final int STREAM_MODE_PIPMAIN = 1;
    public final static int STREAM_MODE_PIPSECONDARY = 2;

    //Camera Modes
    public static final int CAMERA_MODE_VISION = 0;
    public static final int CAMERA_MODE_DRIVER = 1;

    //Pipelines
    public static final int PIPELINE_DEFAULT = 0;
    public static final int PIPELINE_TARGET_TRACKING = 6;

    private NetworkTableInstance _networkTable;
    public LemonLight(){
        this._networkTable = NetworkTableInstance.getDefault();
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
    
    } 
    public double getHorizontalOffset() {
        return this._getValue("tx").doubleValue();
    }
    public double getVerticalOffset() {
        return this._getValue("ty").doubleValue();
    }
    public double getPipleline() {
        return this._getValue("getpipe").doubleValue();
    }
    public void setPipeline(int mode) {
        this._setValue("pipeline", mode);
    }
    public void setLED_Mode(int pipeline) {
        this._setValue("ledMode", pipeline);
    }
    public double getCamera_Stream() {
        return this._getValue("stream").doubleValue();
    }
    public int getLEDmode() {
        return this._getValue("ledMode").intValue();
    }
    public int getStreamMode() {
        return this._getValue("stream").intValue();
    }
    public int getCameraMode() {
        return this._getValue("camMode").intValue();
    }
    public void setStreamMode(int mode) {
        this._setValue("stream", mode);
    }
    public void setCameraMode(int mode) {
        this._setValue("camMode", mode);
    }
    private Number _getValue(String propertyName) {
        return this._networkTable.getTable(LIMELIGHT_TABLE_NAME).getEntry(propertyName).getNumber(0);
    }
    private void _setValue(String propertyName, Number value) {
         this._networkTable.getTable(LIMELIGHT_TABLE_NAME).getEntry(propertyName).setNumber(value);
    }
}