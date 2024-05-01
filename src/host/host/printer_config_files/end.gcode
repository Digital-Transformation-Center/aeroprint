{if layer_z < max_print_height}G1 Z{z_offset+min(layer_z+1, max_print_height)} F720 ; Move print head up{endif}
M104 S0 ; turn off temperature
M140 S0 ; turn off heatbed
M107 ; turn off fan
G1 X241 Y170 F3600 ; park
{if layer_z < max_print_height}G1 Z{z_offset+min(layer_z+23, max_print_height)} F300 ; Move print head up{endif}
G4 ; wait
M572 S0 ; reset PA
M593 X T2 F0 ; disable IS
M593 Y T2 F0 ; disable IS
M84 X Y E ; disable motors
; max_layer_z = [max_layer_z]