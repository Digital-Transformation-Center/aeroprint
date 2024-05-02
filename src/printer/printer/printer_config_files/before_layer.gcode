;BEFORE_LAYER_CHANGE
G92 E0.0
;[layer_z]
M201 X{interpolate_table(extruded_weight_total, (0,4000), (1400,2500), (10000,2500))} Y{interpolate_table(extruded_weight_total, (0,4000), (1400,2500), (10000,2500))}
{if ! spiral_vase}M74 W[extruded_weight_total]{endif}
