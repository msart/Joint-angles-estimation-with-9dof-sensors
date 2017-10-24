# Joint-angles-estimation-with-9dof-sensors

The arduino code and the processing visualizator are just minor modifications from Adafruit sensor fusion example from their library and Madgwick visualizator to work together.

The julia code is a traduction from the c open source AHRS implementation from Madgwick, but is in construction.

In the python source directory you can find the working implementation of angle estimation.

There is a simple implementation of a fuzz effect in Puredata plug and play for a bass or guitar.

Instructions:

Connect the 2 sensors, the first one must be in forearm.
Start puredata and open the GuitarExtended_SimpleFuzz.pd
Connect the instrument
Then run the Joint_Angles_Estimation.py 

Check if the communication between pd and the python program is workign as intended
