#! /bin/bash

avrdude -p x32e5 -P usb -c avrispmkii -U flash:w:waveform.hex
