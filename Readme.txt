# DECT Mesh Communication System

## Contents:
1. Main Code: The main codebase includes functionality for managing DECT mesh network communication and integrating GPS data transmission.
2. GPS Module Code: This module handles UART communication with the GPS module, parsing NMEA sentences, and extracting relevant GPS information.

## Overview
This repository contains code for a DECT (Digital Enhanced Cordless Telecommunications) mesh communication system. The system enables transmission and reception of data packets between devices in a DECT mesh network.

## Setup
1. Ensure you have the necessary hardware and development environment set up to run the code.
2. Clone this repository to your local machine.
3. Select the overlay and conf file according to the device to be programmed and create build.
4. Set up the mode by changing the `RD_ID` variable in the code. Depending on the mode, the device can function as a transmitter, relay, or sink. You can set `RD_ID` according to the desired mode:
   - Transmitter (Mode 'T'): Set `RD_ID` to a value between 1 and 10.
   - Relay (Mode 'R'): Set `RD_ID` to a value between 11 and 40.
   - Sink (Mode 'S'): Set `RD_ID` to a value between 41 and 100, which corresponds to a sink in the mesh network.

## Functionality
### Transmitter (Mode 'T')
- The transmitter sends sensor data upon pressing buttons on the Development Kit (DK).
- Each button corresponds to sending data to one of the three clusters in the DECT mesh network.
- Sensor data includes temperature, pressure, and humidity, along with a device identifier.
- GPS data is periodically obtained from the GPS module and time and location is updated and printed every minute.

### Relay (Mode 'R')
- Relays receive and forward data packets between different clusters in the DECT mesh network.
- They facilitate communication between devices in separate clusters, ensuring data transmission across the entire network.
- Relays keep track of the sinks within their vicinity.
- If a sink fails to inform the relay of its presence at startup, the relay refrains from sending data to it.
- This ensures efficient data routing and avoids unnecessary transmissions to inactive sinks.

### Sink (Mode 'S')
- Sinks are endpoint devices that collect data from multiple transmitters or relays within their range.
- They act as data aggregation points, receiving and processing data packets from other devices in the mesh network.
- Upon startup, a sink sends an initial signal to the relays within its cluster to inform them of its presence.
- This signal serves to establish communication and coordination between the sink and relays, ensuring efficient data transfer within the cluster.

### Cluster
- In this context, a cluster refers to a group of devices within the DECT mesh network that are organized based on their geographic proximity or functional similarity.
- Each cluster may contain multiple devices, including relays, and sinks.
- Relays facilitate communication between clusters, enabling seamless data exchange across the network.

### Cluster Configuration

  Cluster 1:
    Relays: IDs 11 to 20
    Sinks: IDs 41 to 60

  Cluster 2:
    Relays: IDs 21 to 30
    Sinks: IDs 61 to 80

  Cluster 3:
    Relays: IDs 31 to 40
    Sinks: IDs 81 to 100

## Usage
- After setting up the mode and deploying the code to your devices, the system will start functioning accordingly.
- Monitor the device output for debugging information and status updates.
- Ensure all devices are powered and within range for effective communication.