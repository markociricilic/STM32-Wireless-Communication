# STM32-Wireless-Communication
I completed this in a group of two as a final project for the course ECE342H1: Computer Hardware.

Using two **STM32F4s**, a series of high-performance MCUs with an ARM Cortex-M4 processor, we connected one to a transmitter and the other to a receiver where it receives data through **wireless communication**.

The figure above shows a circuit set-up but later on an antenna was added to each transmitter/receiver module so that it is mobile, and with it we were able to transmit data from a distance of approximately **15 meters** accurately, but reaching as far as 20 meters.

# Design
I implemented the interfacing between the STM32F4 and the computer (user) which was done using USART (serial communication).

The tool we used was the **Hercules SETUP utility**, a serial port terminal, where the user can input a string up to **80 characters** to transmit.

# Transmitter
To scan the data sent, a postamble ! was used, so for example to send the string Hello world, the user would input into Hercules SETUP utility: Hello world!.

Additionally, a DAC was used that converts the string after it has been encoded into a binary format with an appropriate buffer size (array) through DMA to be transmitted wirelessly to the receiver.

The data’s payload (transmission) triggers an interrupt when it is half-complete and when it is complete. This is because of performance and reliability as with smaller payloads it doesn’t require as fast or reliable of a connection to transmit data without large delays and/or without errors.

# Receiver
Once the receiver received the data it would output it on the receiver’s computer’s Hercules SETUP utility. But, to do this the data or payload had to be decoded; this meant ‘recontructing’ the byte/char to form the transmitted string. This time an ADC was used through DMA to convert the analog transmission with a very large buffer (16384 bytes) back into a binary format.

To determine the original transmission, a simple check was used; if the amount of 1’b1 in the payload was greater than 100 then it would be decoded as a 1’b1, else it would be considered a 1’b0. From this the binary values were decoded back to characters.

For an overview of the project concept, hardware details, and photos of the setup, please visit [markociricilic.com/projects/wireless-communication](https://markociricilic.com/projects/wireless-communication).
