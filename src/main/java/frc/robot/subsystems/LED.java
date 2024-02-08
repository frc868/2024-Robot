public class LED {
    private AddressableLED led;
    private AddressableLEDbuffer ledBuffer;


    public LED (int pwmPort, int numLEDs) {
        led = new AddressableLED(pwmPort);
        ledbuffer = new AddressableLEDbuffer(numLEDS);
        led.setlength(ledBuffer.getlength());
        led.setData(ledBuffer);
        led.start();
    }


    private void setStr9ipColor(int red, int green, int blue)    {
        for (int i=0, i < ledBuffer.getlength(); i++)   {
            ledBuffer.setRGB(i, red, green, blue);
        }
        led.setData(ledBuffer);
    }
}