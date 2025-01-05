package org.firstinspires.ftc.teamcode;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

public class ControlHub {

    static String[][] addresses = {
            //   MAC Address          Network               Comment
            {"C4:3C:B0:E9:1A:38", "8628-RC",          "8628 Competition Bot"},
            {"7C:A7:B0:07:BE:CF", "FTC-8WA",          "8628 Backup Bot"},
            {"C8:FE:0F:2C:56:14", "7462-RC",          "7462 Competition Bot"},
            {"7C:A7:B0:0F:CB:78", "FTC-7462-Spare",   "Rowan's Bot"},
            {"7C:A7:B0:09:82:54", "FTC-7462-TankGuy", "O'Brien Hub"},
            {"7C:A7:B0:07:76:91", "8628-RC",          "8628-RC themovie"}
    };

    private final String macAddress;

    public ControlHub() throws NullPointerException {
        StringBuilder macAddressStr = null;

        try {
            // Get the network interfaces on the system
            Enumeration<NetworkInterface> networks = NetworkInterface.getNetworkInterfaces();

            while (networks.hasMoreElements()) {
                NetworkInterface network = networks.nextElement();

                // Get the hardware address (MAC address)
                byte[] macAddressBytes = network.getHardwareAddress();

                if (macAddressBytes != null) {
                    // Convert the byte array to a readable MAC address format
                    macAddressStr = new StringBuilder();
                    for (int i = 0; i < macAddressBytes.length; i++) {
                        macAddressStr.append(String.format("%02X", macAddressBytes[i]));
                        if (i < macAddressBytes.length - 1) {
                            macAddressStr.append(":");
                        }
                    }
                }
            }
        } catch (
                SocketException e) {
            e.printStackTrace();
        }

        if (macAddressStr==null) {
            this.macAddress = null;
        } else {
            this.macAddress = macAddressStr.toString();
        }
    }

    public String getMacAddress() {
        return this.macAddress;
    }

    public String getNetworkName() {

        String networkName = "";

        for (String[] address : addresses) {
            if (address[0].equals(this.macAddress)) {
                networkName = address[1];
            }
        }
        return networkName;
    }

    public String getComment() {

        String comment = "";

        for (String[] address : addresses) {
            if (address[0].equals(this.macAddress)) {
                comment = address[2];
            }
        }
        return comment;
    }
}
