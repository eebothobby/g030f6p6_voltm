# Dual-channel Breadboard Voltmeter

**Acknowledgement:**: I got the basic idea for the project from the following youtube video:
[2-channel miniature voltmeter for breadboards](https://www.youtube.com/watch?v=5cCy5Wh3Lyc)

However, I made several changes to make the design better suited for my needs.

1. The input impedance of the original design is too low  (13k9) since it uses a resistor voltage divider with
   10K and 3K9 resistors to increase the voltage range to about 12V from 3.3V.
   Such a low input impedance could significantly perturb the circuits being measured. 
   In my design I use significantly larger resistors ( 200K and 51K) so that the input impedance is more than 18x higher
   resulting in a much smaller impact on the circuit being measured.
2. Since I use much larger resistors for the voltage divider, I use a rail-to-rail opamp (MCP6022) as a unity gain buffer
   so that the impedance of the ADC does not impact the measurement.
3. The voltage range that can be measured is also larger (up to 16V).  I also use a 3.3V LDO to power the microcontroller
   that can use as input 4V - 16V providing greater flexibility. I also use protection diodes on the two voltage inputs.
4. Although the resistor voltage divider increases the voltage range that it measure, it decreases the resolution for lower
   voltages (up to 3.3V) that could otherwise be directly measured by the ADC.
   In order not to sacrifice higher resolution measurement of lower voltages, I use a MOSFET whose gate is driven by a GPIO pin
   to optionally disable the voltage divider for lower voltages.
5. I use a more widely available STM32G030F6P6 microcontroller instead of a CH32 microcontroller.
   The ADCs on the STM32G030 have 12-bit resolution that I increase to 16 bits by 16x oversampling, resulting in
   better resolution.  I use DMA to read the ADC data into memory.

   Here is a photo showing the front and back fo the board:
   
   ![STM32G030 breadboard voltmenter](voltm_photo.jpg)
   