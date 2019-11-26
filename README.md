# IsolationMonitorBender
The Bender ISO165 unit is designed to monitor isolation in a EV and it also measures two voltages (up to 600v).  The purpose of the unit is to detect if insulation of the high voltage system has deteriorated and potentially created a dangerous condition.

Code to drive the Bender ISO165C isolation monitoring device.  The code is a built on the CANSenderTool framework (posted in another repository) but configured with the CAN frames and frame decoding to work with the ISO165C.

Documentation states that the ISO165C runs at 250kpbs, however my unit runs at 500kpbs CAN.

The code starts up and simply prepares the CAN interface to the ISO165C.  In this prototype code the user must enter commands to initialize the ISO165C and requests voltage updates.  The tool will decode some of the CAN frames sent out by the ISO165C.  These commands are included at the top of the INO file.  The commands instruct the software to send pre-prepared CAN frames to the ISO165C.

ISO165C has two pairs (pos/neg) for high voltage detection, one of which also measures isulation effectiveness.  The HV2 lines measure all the time, however the HV1 lines need to have their pos and neg relays triggered to start voltage measurements and also isulation measurements.  Insulation measurements are also output at 1Hz automatically (without being requested), whereas the high voltage readings must be requested.  Based on my testing (which may not be 100% perfect), the high voltage readings can be requested more often than 1Hz, but the high voltage value reported still updates on a 1Hz basis.  Additionally (again based on my testing), the unit can take several seconds to detect the full value of the high voltage line.  My results suggest that the high voltage reading trends up to the full value of the high voltage line over several seconds.

In the TesLorean, the ISO165C will be measuring the voltage  and isolation at the HVJB (high voltage junction box).  The voltage reading will be able to detect that the pre-charge circuit is working correctly (i.e. that current/voltage is connected via the ppre-charge resistor before closing the positive contactor).  The isolation monitoring will provide guidance to the Trip Computer, touch screen display, and warning lights on the instrumemnt cluster should the insulation fall to a warning or fault level.  If the ISO165C reports an insulation fault, the BCM (battery control module) will open the contactors and remove high voltage from the car systems.

