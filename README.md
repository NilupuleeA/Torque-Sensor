# Torque-Sensor

## Power Supply

In our battery-powered setup, we utilize a regulator circuit to maintain stable voltage output, ensuring consistent power supply to both the microcontroller and strain gauges. We selected BD450M5 due to its low dropout voltage, which is typically 0.2V. Its maximum current rating is 0.5A.

<p align="center">
  <img src="https://github.com/user-attachments/assets/d51e83f2-ea49-449f-943b-da08a0e0aa66" alt="Power" width="50%" />
</p>


## Bridge Balancing Circuit
 
The initial unbalances in strain gauge circuits, caused by irregularities in gauge resistances, often exceed the actual strain-induced signal. The solution involves adjusting resistances within the Wheatstone Bridge configuration. However, manual methods pose challenges such as determining the arm needing adjustment and avoiding further imbalance.

<p align="center">
  <img src="https://github.com/user-attachments/assets/e328569a-934b-4277-9ce0-c3f675b4b2cb" alt="Power" width="50%" />
</p>

We choosed AD5171 as the digital potentiometer to adjust resistance because,
 - Stores the resistance value, maintaining the wiper position even when power is off and resuming it when power is restored.
 - Permanently stores a chosen resistance value after OTP activation.
 - Allows unlimited adjustments to the resistor value prior to OTP activation.
 - 64 The AD5171 has 64 discrete wiper positions.
 - This fine adjustment allows for precise control over the resistance settings.

 ## Instrumentation Amplifier Circuit
 
To enhance the output signal from a Wheatstone bridge, we employ an instrumentation amplifier, known for its precision in measurement applications. Specifically, we’ve chosen the AD8293G160 for its cost effectiveness and capability to amplify the signal without introducing significant noise. This amplifier maintains accuracy over time and operates efficiently with minimal power requirements. In addition, we have refined the signal further by incorporating filters to eliminate unwanted noise, ensuring a clean and reliable output for our measurement purposes. The circuit amplifies the voltage difference between the Wheatstone bridge strain gauge. 

<p align="center">
  <img src="https://github.com/user-attachments/assets/4e488c14-f177-4100-8c7a-a293a25b5840" alt="Power" width="50%" />
</p>

Here why we used AD8293G160.
 - Up to 160 MHz, suitable for high-speed signal processing.
 - Typically 20 µV offset voltage and 0.3 µV/°C drift, minimizing signal processing errors.
 - Typically 140 dB CMR, effective in rejecting common-mode noise.
 - Includes overvoltage protection and thermal shutdown for reliable operation.
 - The amplifier has a gain of 160, which is suitable for our application.

## Offset Circuit for Instrumentation Amplifier
 
The combination of an instrumentation amplifier, such as the AD8293G160, with a digital-to-analog converter (DAC) like the MCP4728, offers precise offset adjustment capabilities in electronic circuits. The instrumentation amplifier serves to accurately amplify small differential signals while rejecting common-mode noise. However, in some applications, it’s necessary to introduce an offset voltage to bias the amplifier’s output to a specific level or compensate for certain system characteristics. By integrating a DAC into the circuit, engineers can dynamically adjust this offset voltage digitally. The MCP4728 provides multiple channels of analog output with high resolution, typically through an I2C interface. This allows for precise control over the offset voltage applied to the instrumentation amplifier.

<p align="center">
  <img src="https://github.com/user-attachments/assets/2fcef8f0-8342-48f2-9c74-f07fe39ffbfb" alt="Power" width="50%" />
</p>

Here’s why we used MCP4728 over other DACs.
 - Provides up to 12-bit resolution for precise analog signal representation.
 - Designed for efficient power usage, beneficial for battery-powered applications.
 - Offers low noise, high linearity, and reliable operation in various conditions.
 - Includes on-board nonvolatile memory (EEPROM) for storing DAC codes and configuration settings

### Amplifier Circuit
 
To amplify the weak signal from the instrumentation amplifier, the circuit incorporates a secondary amplifier, the OPA336, which operates efficiently on low supply voltage and features a rail-to-rail output swing. Configured as a non-inverted amplifier with a single-pole low-pass filter, it ensures signal flexibility across different force ranges. Gain adjustments are made using the AD5171 digital potentiometer, which provides precise control with 64-step resolution for optimal signal amplification.

<p align="center">
  <img src="https://github.com/user-attachments/assets/567fecd6-c58e-4914-89c6-75039c618005" alt="Power" width="50%" />
</p>

Here are the advantages of the OPA336 operational amplifier (op-amp) compared to other typical op-amps like the LM741 and TL072.
 - Very low power consumption, ideal for battery-operated devices.
 - High speed (up to 10 MHz bandwidth) and precision with low offset voltage.
 - Low noise density and distortion, suitable for sensitive measurement.
 - Designed for efficient single-supply operation.

###  Microcontroller Circuit
 
The microcontroller is used both to display the output torque value and to act as the ADC converter. The unit converts the analog signal from the strain gauges into digital data, acting as an ADC, calculates the corresponding torque value, and controls the output signal, which may involve displaying the torque value or transmitting it to external devices. The ADC converts the analog input signal into a 10-bit digital value, providing 1024 discrete levels of representation.

<p align="center">
  <img src="https://github.com/user-attachments/assets/1bc94aab-a6e1-4470-81f2-9c05f05929ee" alt="Power" width="50%" />
</p>
