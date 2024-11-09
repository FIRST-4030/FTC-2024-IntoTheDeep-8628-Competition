package org.firstinspires.ftc.teamcode;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

public class ControlHub {

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

        for (int i=0 ; i< ControlHubNames.addresses.length ; i++) {
            if (ControlHubNames.addresses[i][0].equals(this.macAddress)) {
                networkName = ControlHubNames.addresses[i][1];
            }
        }
        return networkName;
    }

    public String getComment() {

        String comment = "";

        for (int i=0 ; i< ControlHubNames.addresses.length ; i++) {
            if (ControlHubNames.addresses[i][0].equals(this.macAddress)) {
                comment = ControlHubNames.addresses[i][2];
            }
        }
        return comment;
    }
}
