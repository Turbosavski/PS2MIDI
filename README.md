# PS2MIDI

PS/2 protocol and devices as midi interfaces: computer keyboard, mouse, touchpad

KiCad 5.1.5 files are provided for the small pcb board. For now existing / tested as a single sided prototypes for touchpad and computer keyboard. 

Previouly known as AI_SI Touchpad Piano - it is a midi trigger device based on a touchpad system found in any laptop computer – brought to a form of an oversimplified instrument that acts as (ahm...) piano. This module (-> https://www.serdashop.com/waveblaster) replaces the first version with Yamaha XG Midi daughter board (the 90s). My basic interest was just piano sounds.

The heart of the system is (of course) arduino. Arduino Promicro can be set as native usb device – so it was chosen (Sparkfun Pro micro 5V 16MHz version – a Chinese clone). An old laptop (sony vaio) provided a touchpad and parts to make the instrument housing. 

After the touchpad version an old ps/2 computer keyboard was used to some extent with adaptations to the relevant arduino library. Another option is of course the mouse - this would be probably very similar to touchpad. All these arduino sketches will be soon available here. 

Quite some work was done in 2019 with adding the AI part on the level of three arduino libraries: midimelodics, midirhythmics and midistatistics - not yet finished but soon to be uploaded.

Available here - in case the virus gets me...
