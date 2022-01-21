First to do, read through the paper that explains the work of the independent study to get up to speed on that was done and what needs to be looked at and what changes should be made. See PDF.

The folder contains two python files. 
The file anchorSignalStrength was used to evaluate the signals strength for the DWM1001 devices to see what effect the metal coated windows has on the signal. The setup is explained in the paper PDF.
The file V207_PS is the position system that was made for the navio autopilot and the raspberry pi running the GCS Mission Planner. As the Navio do not have bluetooth the decawave-ble library can't be used and trying to fix the bluetooth is a pain, which was tried but not successful.
So look into the API Guide for the DWM1001, but also check how it is done in the file, but also the dwm_class.py.
Another tips would be to see how the UART communication works on the DWM1001, by printing out the the data that comes from the UART, e.g before a  "if len(data) >= 15:".
Then you will understand, why it was made as it is.

Any questions, then email to petternmnordin@gmail.com
rather then the RU email.
