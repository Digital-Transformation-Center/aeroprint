M17 ; enable steppers
M862.1 P[nozzle_diameter] ; nozzle diameter check
M862.3 P "MK4" ; printer model check
M862.5 P2 ; g-code level check
M862.6 P"Input shaper" ; FW feature check
M115 U5.1.2+13478

G90 ; use absolute coordinates
M83 ; extruder relative mode

M140 S[first_layer_bed_temperature] ; set bed temp
M104 T0 S170 ; set extruder temp for bed leveling
M109 T0 R170 ; wait for temp

M84 E ; turn off E motor

G28 ; home all without mesh bed level

G1 X42 Y-4 Z5 F4800

M302 S160 ; lower cold extrusion limit to 160C

G1 E-2 F2400 ; retraction

M84 E ; turn off E motor

G29 P9 X10 Y-4 W32 H4

M106 S100

G0 Z40 F10000

M190 S60 ; wait for bed temp

M107

;
; MBL
;
M84 E ; turn off E motor
G29 P1 ; invalidate mbl & probe print area
G29 P1 X0 Y0 W50 H20 C ; probe near purge place
G29 P3.2 ; interpolate mbl probes
G29 P3.13 ; extrapolate mbl outside probe area
G29 A ; activate mbl

; prepare for purge
M104 S230
G0 X0 Y-4 Z15 F4800 ; move away and ready for the purge
M109 S230

G92 E0
M569 S0 E ; set spreadcycle mode for extruder

;
; Extrude purge line
;
G92 E0 ; reset extruder position
G1 E2 F2400 ; deretraction after the initial one before nozzle cleaning
G0 E7 X15 Z0.2 F500 ; purge
G0 X25 E4 F500 ; purge
G0 X35 E4 F650 ; purge
G0 X45 E4 F800 ; purge
G0 X48 Z0.05 F8000 ; wipe, move close to the bed
G0 X51 Z0.2 F8000 ; wipe, move quickly away from the bed

G92 E0
M221 S100 ; set flow to 100%