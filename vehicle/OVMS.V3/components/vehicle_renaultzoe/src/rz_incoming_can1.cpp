/**
 * Handles incoming CAN-frames on bus 1, the C-bus
 */

#include "vehicle_renaultzoe.h"

void OvmsVehicleRenaultZoe::IncomingFrameCan1(CAN_frame_t* p_frame)
	{
	//ESP_LOGI(TAG, "Renault Zoe IncomingFrameCan1");
	uint8_t *d = p_frame->data.u8;
	#define CAN_7NIBL(b)    (((((UINT)d[(int)b/8] << 8) | (d[((int)b/8)+1])) >> (b-((int)b/8)*8)) & 127);
	#define CAN_8NIBL(b)    (((((UINT)d[(int)b/8] << 8) | (d[((int)b/8)+1])) >> (b-((int)b/8)*8)) & 255);
	#define CAN_9NIBL(b)    (((((UINT)d[(int)b/8] << 8) | (d[((int)b/8)+1])) >> (b-((int)b/8)*8)) & 511);
	switch (p_frame->MsgID)
		{
	case 0x1fd:
		{
		// 1fd,48,55,1,80,0,kW,,,ff Consumption ?
		}
		break;
	case 0x427:
		{
		// 427,40,47,0.3,0,0,kW,,,e2\n"  Available Charging Power
		// 427,49,57,0.1,0,1,kWh,,,e2\n" Available Energy
		}
		break;
	case 0x42a:
		{
		// 42a,30,39,0.1,400,1,°C,,,ff\n" Evaporator Temp Measure
		// 42a,48,50,1,0,0,,,,e2\n" ClimLoopMode
		}
		break;
	case 0x42e:
		{
		// 42e,0,12,0.02,0,2,%,,,e3\n"  State of Charge
		// 42e,20,24,5,0,0,%,,,e2\n"  Engine Fan Speed
		// 42e,38,43,1,0,1,A,,,e3\n"  Charging Pilot Current
		m_charge_pilot_current->SetValue((int(d[4]) << 4 | d[5] >> 4) & 63);
		if (m_charge_pilot_current->AsInt() > 0) {
			StdMetrics.ms_v_charge_pilot->SetValue(true);
		} else {
			StdMetrics.ms_v_charge_pilot->SetValue(false);
		}
		// 42e,44,50,1,40,0,°C,,,e3\n"  HV Battery Temp
		// We rather use calculation than measurement since value recieved has no decimals
		//            StdMetrics.ms_v_bat_temp->SetValue((float(CAN_7NIBL(45))-40));
		// 42e,56,63,0.3,0,1,kW,,,ff\n"  Max Charge Power
		m_b_bat_max_charge_power->SetValue((float(d[7])*0.3));
		}
		break;
	case 0x430:
		{
				// 430,24,33,0.5,30,1,°C,,,e2\n"  Comp Temperature Discharge
				// 430,38,39,1,0,0,,,,e2\n"  HV Battery Cooling State
				// 430,40,49,0.1,400,1,°C,,,e2\n"  HV Battery Evaporator Temp
				m_bat_heat_sink_temp->SetValue((float(UINT((d[5] << 2) | d[6] >> 6) & 1023) - 400)*0.1);
		}
		break;
	case 0x432:
		{
		// 432,36,37,1,0,0,,,,e2\n" HV Bat Conditionning Mode
		}
		break;
	case 0x534:
		{
		// 534,32,40,1,40,0,°C,,,e5\n" // Temp out (Vielleicht dieser statt 656
		// StdMetrics.ms_v_env_temp->SetValue((float(CAN_9NIBL(33))-40));
		}
		break;
					
	case 0x654:
		{
		// Battery Parmeters
		// 654,2,2,1,0,0,,,,ff\n" // Charging Plug Connected
		// 654,25,31,1,0,0,,,,ff\n" // State of Charge
		// 654,32,41,1,0,0,min,,,ff\n" // Time to Full
		if (d[0] && 0x40 == 0x40)
			{
			//charging plug connected
			PollSetState(2);
			}
		StdMetrics.ms_v_charge_duration_full->SetValue((UINT(d[4] << 2) | d[5] >> 6) & 1023);
		// 654,42,51,1,0,0,km,,,ff\n" // Available Distance
		StdMetrics.ms_v_bat_range_est->SetValue((UINT(d[5] << 2) | d[6] >> 4) & 1023);
		}
		break;
	case 0x656:
		{
		// External Temp
		// 656,48,55,1,40,0,°C,,,e2\n" // External Temp
		StdMetrics.ms_v_env_temp->SetValue((float(d[6])-40));
		}
		break;
	case 0x658:
		{
		// SOH
		// 658,33,39,1,0,0,%,,,ff\n" // Battery Health
		StdMetrics.ms_v_bat_soh->SetValue(d[4]);
		}
		break;
	case 0x65b:
		{
		// 65b,41,43,1,0,0,,,,ff\n" // Charging Status Display
		}
		break;
	case 0x673:
		{
		// TPMS
		// 673,2,4,1,0,0,,,,ff\n" // Rear right wheel state
		// 673,5,7,1,0,0,,,,ff\n" // Rear left wheel state
		// 673,8,10,1,0,0,,,,ff\n" // Front right wheel state
		// 673,11,13,1,0,0,,,,ff\n" // Front left wheel state
		// 673,16,23,13.725,0,0,mbar,,,ff\n" // Rear right wheel pressure
		StdMetrics.ms_v_tpms_rr_p->SetValue((float(d[2])*13.725));
		// 673,24,31,13.725,0,0,mbar,,,ff\n" // Rear left wheel pressure
		StdMetrics.ms_v_tpms_rl_p->SetValue((float(d[3])*13.725));
		// 673,32,39,13.725,0,0,mbar,,,ff\n" // Front right wheel pressure
		StdMetrics.ms_v_tpms_fr_p->SetValue((float(d[4])*13.725));
		// 673,40,47,13.725,0,0,mbar,,,ff\n" // Front left wheel pressure
		StdMetrics.ms_v_tpms_fl_p->SetValue((float(d[5])*13.725));
		}
		break;
		}
	}

