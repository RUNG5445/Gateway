# README

This Arduino code is designed to run on a microcontroller board with connectivity options such as LoRa and LTE (using SIM7600 module). The purpose of the code is to collect data from various sensors (such as temperature, humidity, battery level) and transmit it to a server via either LoRa or LTE connectivity. Additionally, it retrieves configuration information and active node data from the server.

## Dependencies

- ArduinoJson: For parsing and generating JSON data.
- TinyGsmClientSIM7600: Library for SIM7600 module to enable LTE connectivity.
- LoRa: Library for LoRa communication.
- WiFi: Library for WiFi connectivity.
- TinyGPS++: Library for parsing GPS data.

## Setup

1. Connect the hardware components including sensors, LoRa module, and SIM7600 module to the microcontroller board as per the hardware specifications.
2. Install the required libraries mentioned in the dependencies section.
3. Upload the provided code to the microcontroller board.

## Configuration

Before uploading the code, make sure to configure the following parameters:

- `URI`: The URL of the server to which data needs to be sent.
- `PORT`: The port number of the server.
- `apiKey`: API key for server authentication.
- WiFi and LTE configuration parameters such as SSID, password, server address, and port.

## Usage

1. Once the code is uploaded, the microcontroller will start collecting sensor data.
2. It will establish a connection either via LoRa or LTE based on availability.
3. If connected, it will send the collected data to the server.
4. It will also retrieve configuration information and active node data from the server periodically.
5. The microcontroller will enter sleep mode to conserve power between data transmissions.

## Troubleshooting

- If facing connectivity issues, ensure that the hardware connections are correct and the configuration parameters (such as SSID, password, server details) are accurate.
- Check the serial output for any error messages or debug information to diagnose issues.

## Notes

- This code is designed to work with specific hardware components and may require modifications to work with different setups.
- Ensure compliance with regulations and data privacy policies while transmitting sensor data over networks.
- Refer to the documentation of the respective libraries for detailed usage instructions and additional features.

## License
This project is licensed under the [MIT License](LICENSE).
