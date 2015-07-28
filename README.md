# SolarSupercap
Arduino source code for a solar powered cell phone charger with Maximum Power Point Tracking (MPPT)
===

The charger consists four modules: the solar panel, a step-down converter, a supercapacitor and a step-up converter. The supercapacitor is used as an intermediate energy storage device to ensure that the charger can operate even with varying solar intensity.
An Arduino microcontroller is used to control the step-up and step-down converters. This requires two control algorithms, both of which must take measurements using the analog to digital converter (ADC). Pulse width modulation (PWM) is used to control the converters.
Additionally a basic Maximum Power Point Tracking algorithm is implemented in order to extract maximum power from the solar panel.
