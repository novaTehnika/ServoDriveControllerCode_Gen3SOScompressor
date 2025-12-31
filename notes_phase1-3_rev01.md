## Notes on FB_HandshakeManager
We should have a single place for enumerating the modes and faults.
Separate debouncing from handshake manager. We should be debouncing all digital inputs.
## Notes on PRG_Main.st
We need to add filtering for the analog and digital inputs. Debounce for the digital inputs. debounce is implemented for mode decode but not others. We should have it on all digital inputs (maybe a shorter debounce time for the limit switches? maybe even a two stage debounce for the homing limit switch?) Median filter followed by low-pass filter configured by time constant. 