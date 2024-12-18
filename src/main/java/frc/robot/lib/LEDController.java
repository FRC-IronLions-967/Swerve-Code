package frc.robot.lib;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

public class LEDController {

    private final double kMinPulse = 0.1;
    private final double kMaxPulse = 0.99;
    private final int kCyclesPerFlash = 25;
    private final int kCyclesPerPulse = 100;
    
    private AddressableLED strip;
    private AddressableLEDBuffer stripBuffer;

    private Color allianceColor;
    private double pulse = kMinPulse;
    private boolean reversePulse;

    private int flashCount;
    private boolean alternatingColorFlip;

    private static BooleanSupplier intakeSupplier;
    private static BooleanSupplier readySupplier;

    /**
     * Control a single strand of LEDs
     * @param pwmPort on the rio for signaling
     * @param numLeds in the strand
     */
    public LEDController(int pwmPort, int numLeds) {
        strip = new AddressableLED(pwmPort);
        stripBuffer = new AddressableLEDBuffer(numLeds);
        strip.setLength(numLeds);

        strip.start();

        allianceColor = Color.kRed;
        
    }

    /**
     * Method to configure robot sensors that influence LED signaling
     * @param intakeSupplier supplier to return when the intake has a Note
     * @param readySupplier supplier to return when the scorer is ready to shoot
     */
    public static void ConfigureLedEvents(BooleanSupplier intakeSupplier, BooleanSupplier readySupplier) {
        LEDController.intakeSupplier = intakeSupplier;
        LEDController.readySupplier = readySupplier;
    }

    /**
     * Heartbeat based on 20ms clock tick to update behavior timers
     */
    public void heartbeat() {
        //update flash every half second
        if (flashCount >= kCyclesPerFlash ) {
            flashCount = 1;
            alternatingColorFlip = !alternatingColorFlip;

            //Get alliance
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if(alliance.get() == DriverStation.Alliance.Blue) {
                    allianceColor = Color.kBlue;
                } else {
                    allianceColor = Color.kRed;
                }
            }
        } else { flashCount++; }

        //Color Fade update
        //Is it getting brighter or dimmer
        if (pulse >= kMaxPulse){
            reversePulse = false;
        } else if (pulse <= kMinPulse) {
            reversePulse = true;
        }
        //Adjust color value
        if (reversePulse) {
            pulse += (kMaxPulse - kMinPulse)/kCyclesPerPulse;
        } else {
            pulse -= (kMaxPulse - kMinPulse)/kCyclesPerPulse;
        }

        strip.setData(stripBuffer);
    }

    /**
     * LED behavior when the robot is disabled
     */
    public void disabledPeriodic() {
        setAllPulse(allianceColor);
    }

    /**
     * LED behavior when the robot is enabled
     */
    public void enabledPeriodic() {
        boolean intakeIn = intakeSupplier.getAsBoolean();
        boolean scorerIn = readySupplier.getAsBoolean();
        if(intakeIn) {
            //Set all to Green
            setAllSolid(Color.kGreen);
        } else if (scorerIn) {
            //Flash alternating LEDs between green and black
            setAlternatingFlash(halfBrightness(Color.kGreen) , Color.kBlack);
        } else {
            //Set all to Red
            setAllSolid(halfBrightness(allianceColor));
        }
    }

    /**
     * Method to cut the brightness of a color preset in half
     * @param color at normal brightness
     * @return dim color
     */
    private Color halfBrightness(Color color) {
        return new Color(color.red/2, color.green/2, color.blue/2);
    }

    /**
     * Method to set alternating LEDs in the string to flash between two colors
     * @param color1
     * @param color2
     */
    private void setAlternatingFlash(Color color1, Color color2) {
        //Set to alternatingly flash Green
        for (int i = 0; i < stripBuffer.getLength(); i++){
            if (alternatingColorFlip) {
                if ( i % 2 == 0) {
                    stripBuffer.setLED(i, color1);
                } else {
                    stripBuffer.setLED(i, color2);
                } 
            } else {
                if ( i % 2 == 0) {
                    stripBuffer.setLED(i, color2);
                } else {
                    stripBuffer.setLED(i, color1);
                } 
            }
        }
    }

    /**
     * Method to set all of the LEDs to the specified color
     * @param color
     */
    private void setAllSolid(Color color) {
        for (int i = 0; i < stripBuffer.getLength(); i++){
            stripBuffer.setLED(i, color);
        }
    }

    /**
     * Method to flash all LEDs in the string between the specified colors
     * @param color1 
     * @param color2
     */
    private void setAllFlash(Color color1, Color color2){
        //Set to alternatingly flash Green
        for (int i = 0; i < stripBuffer.getLength(); i++){
            if (alternatingColorFlip) {
                stripBuffer.setLED(i, color1);
            } else {
                stripBuffer.setLED(i, color2);
            }
        }
    }

    /**
     * Method to gently pulse the brightness of all LEDs
     * @param color of the LEDs when they pulse
     */
    private void setAllPulse(Color color){
        Color pulseColor = new Color(color.red*pulse, color.green*pulse, color.blue*pulse);
        for (int i = 0; i < stripBuffer.getLength(); i++){
            stripBuffer.setLED(i, pulseColor);
        }
    }


}
