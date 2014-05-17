import java.net.*;
import java.io.*;

public class InputStreamByteWrapper {
    public static byte[] bfr = null;
    
    public InputStreamByteWrapper(int capasity) {
        bfr = new byte[capasity];
    }
    
    public InputStreamByteWrapper() {
        this(4096);
    }
    
    public int read(Socket mySocket, int offset, int length) throws IOException {
        InputStream in=mySocket.getInputStream();
        return in.read(bfr, offset, length);
    }
    
    public int read(Socket mySocket, int length) throws IOException {
        InputStream in=mySocket.getInputStream();
        return in.read(bfr, 0, length);
    }
}