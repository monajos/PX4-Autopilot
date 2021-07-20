uint64 timestamp		# time since system start (microseconds)
float32 terrain_alt				# Reference altitude AMSL, (metres)




#Angular rates
float32 current_x_rate
float32 current_y_rate
float32 current_z_rate

#Attitudes
float32 current_att_pitch
float32 current_att_roll
float32 current_att_yaw

#Speeds
float32 current_true_airspeed
float32 current_groundspeed
float32 current_calibrated_airspeed
bool airspeed_valid

#Thrust
float32 current_thrust_body

#Altitude
float32 current_altitude_baro
float32 current_gps_altitude

#Position
float32 current_latitude
float32 current_longitude

#PWM in
float32 pwm_err
float32 pwm_period
float32 pwm_pulse_width
float32 pwm_timestamp

#PWM out
float32 act_out_output0
float32 act_out_output1
float32 act_out_output2
float32 act_out_output3
float32 act_out_output4
float32 act_out_output5
float32 act_out_output6
float32 act_out_output7
float32 act_out_output8
float32 act_out_output9
float32 act_out_output10
float32 act_out_output11
float32 act_out_output12
float32 act_out_output13
float32 act_out_output14
float32 act_out_output15

float32 act_out_timestamp

float32 act_cntr_output0
float32 act_cntr_output1
float32 act_cntr_output2
float32 act_cntr_output3
float32 act_cntr_output4
float32 act_cntr_output5
float32 act_cntr_output6
float32 act_cntr_output7

#Battery status
float32 bat_status_timestamp
float32 bat_status_current_a
float32 bat_status_discharged_mah
float32 bat_status_voltage_v

#Setpoints->Altitude
float32 position_sp_alt
float32 position_sp_airspeed

#TECS energy states
float32 std_tecs_kin_energy
float32 std_tecs_pot_energy
float32 std_tecs_tot_energy

