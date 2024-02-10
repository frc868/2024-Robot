public class ledStrip {
    private AddressableLED light;
    private AddressableLEDbuffer buffer;

    public ledStrip {
        light = new AddressableLED(9);
        buffer = new AddressableLEDbuffer(60);
        light.setLength(buffer.getLength());

        light.start();
    }
}