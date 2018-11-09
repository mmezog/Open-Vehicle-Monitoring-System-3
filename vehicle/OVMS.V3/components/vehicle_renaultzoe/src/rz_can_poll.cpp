/**
 * Incoming poll reply messages
 */

#include "vehicle_renaultzoe.h"

void OvmsVehicleRenaultZoe::IncomingPollReply(canbus* bus, uint16_t type, uint16_t pid, uint8_t* data, uint8_t length, uint16_t mlremain)
{
	//    ESP_LOGI(TAG, "Renault Zoe IncomingPollReply");
	//    uint8_t bVal;
	//    UINT base;
	//    uint32_t lVal;


  switch (m_poll_moduleid_low) {
  case 0x7ec:
    switch (pid)
	 		{
		case 0x2001:
			{
			// 7ec,24,31,1,40,0,°C,222001,622001,ff,Battery Rack temperature
			StdMetrics.ms_v_bat_temp->SetValue( (float) (CAN_UINT(0)) - 40 );
			}
			break;
		case 0x2002:
			{
			// 7ec,24,39,2,0,2,%,222002,622002,e2\n" // SOC
			StdMetrics.ms_v_bat_soc->SetValue( float(CAN_UINT(0)) * 2 / 100);
			}
			break;
		case 0x2003:
			{
			 // 7ec,24,39,.01,0,0,km/h,222003,622003,ff,Vehicle speed
			StdMetrics.ms_v_pos_speed->SetValue( float(CAN_UINT(0))  * 100);
			}
			break;
		case 0x2006:
			{
			// 7ec,24,47,1,0,0,km,222006,622006,ff\n" // Odometer
			StdMetrics.ms_v_pos_odometer->SetValue(float(CAN_UINT24(0)));
			}
			break;
		case 0x200e:
			{
			// 7ec,31,31,1,0,0,,22200E,62200E,ff,Key state,0:key off;1:key on\n" //
			if (CAN_BYTE(0) && 0x01 == 0x01)
				{
				vehicle_renaultzoe_car_on(true);
				}
			else
				{
				vehicle_renaultzoe_car_on(false);
				}
			}
			break;
		case 0x3028:
			{
			// 7ec,24,31,0.5,0,1,A,223028,623028,ff\n" // 14V current
			StdMetrics.ms_v_bat_12v_current->SetValue(float(CAN_BYTE(0))*0.1);
			}
			break;
		case 0x320C:
			{
			// 7ec,24,39,.005,0,0,kwh,22320C,62320C,ff,Available discharge Energy
			m_b_bat_avail_energy->SetValue(float(CAN_UINT(0))*0.005);
			}
			break;
		case 0x3203:
			{
			// 7ec,24,39,0.5,0,2,V,223203,623203,ff\n" // HV Battery voltage
			rz_bat_voltage = float(CAN_UINT(0)) * 50/100;
			}
			break;
		case 0x3204:
			{
			// 7ec,24,39,0.25,32768,2,A,223204,623204,ff\n" // HV Battery current
			rz_bat_current = (float(CAN_UINT(0))-32768) * 25/100;
			}
			break;
		case 0x33dc:
			{
			// 7ec,24,47,0.001,1,0,kWh,2233dc,6233dc,ff\n" // Consumed domestic energy
			StdMetrics.ms_v_charge_kwh->SetValue(CAN_UINT24(0)*0.001);
			}
				break;
		}
		break;
	case 0x7bb:
		switch (pid)
			{
		case 0x01:
			{
			// Todo
			// 7bb,336,351,0.01,0,2,kW,2101,6101,e2\n" // Maximum battery input power
			}
			break;
		case 0x03:
			{
			// Todo
			// 7bb,56,71,10,0,0,°C,2103,6103,5\n" // Mean battery compartment temp
			// 7bb,96,111,.01,0,0,V,2103,6103,ff,21_03_#13_Maximum_Cell_voltage\n" //
			// 7bb,112,127,.01,0,0,V,2103,6103,ff,21_03_#15_Minimum_Cell_voltage\n" //
			// 7bb,192,207,0.01,0,2,%,2103,6103,e2\n" // Real State of Charge
//                    if (type == VEHICLE_POLL_TYPE_OBDIIGROUP)
//                    {
//                        if (m_poll_ml_frame == 1)
//                        {
//                            m_b_cell_volt_max->SetValue(float( CAN_BYTE(6)*0.01 ),Volts);
//                        }
//                        else if (m_poll_ml_frame == 2)
//                        {
//                            m_b_cell_volt_min->SetValue(float( CAN_BYTE(1)*0.01 ),Volts);
//                        }
//                    }
			}
			break;
		case 0x04:
			{
			// Todo
			// 7bb,32,39,1,40,0,°C,2104,6104,e2\n" // Cell 1 Temperature
			// 7bb,56,63,1,40,0,°C,2104,6104,e2\n" // Cell 2 Temperature
			// 7bb,80,87,1,40,0,°C,2104,6104,e2\n" // Cell 3 Temperature
			// 7bb,104,111,1,40,0,°C,2104,6104,e2\n" // Cell 4 Temperature
			// 7bb,128,135,1,40,0,°C,2104,6104,e2\n" // Cell 5 Temperature
			// 7bb,152,159,1,40,0,°C,2104,6104,e2\n" // Cell 6 Temperature
			// 7bb,176,183,1,40,0,°C,2104,6104,e2\n" // Cell 7 Temperature
			// 7bb,200,207,1,40,0,°C,2104,6104,e2\n" // Cell 8 Temperature
			// 7bb,224,231,1,40,0,°C,2104,6104,e2\n" // Cell 9 Temperature
			// 7bb,248,255,1,40,0,°C,2104,6104,e2\n" // Cell 10 Temperature
			// 7bb,272,279,1,40,0,°C,2104,6104,e2\n" // Cell 11 Temperature
			// 7bb,296,303,1,40,0,°C,2104,6104,e2\n" // Cell 12 Temperature
			if (type == VEHICLE_POLL_TYPE_OBDIIGROUP)
				{
				if (m_poll_ml_frame == 0)
					{
					rz_battery_module_temp[0]=( (INT)CAN_BYTE(1)-40 );
					}
				else if (m_poll_ml_frame == 1)
					{
					rz_battery_module_temp[1]=( (INT)CAN_BYTE(1)-40 );
					rz_battery_module_temp[2]=( (INT)CAN_BYTE(4)-40 );
					}
				else if (m_poll_ml_frame == 2)
					{
					rz_battery_module_temp[3]=( (INT)CAN_BYTE(0)-40 );
					rz_battery_module_temp[4]=( (INT)CAN_BYTE(3)-40 );
					rz_battery_module_temp[5]=( (INT)CAN_BYTE(6)-40 );
					}
				else if (m_poll_ml_frame == 3)
					{
					rz_battery_module_temp[6]=( (INT)CAN_BYTE(2)-40 );
					rz_battery_module_temp[7]=( (INT)CAN_BYTE(5)-40 );
					}
				else if (m_poll_ml_frame == 4)
					{
					rz_battery_module_temp[8]=( (INT)CAN_BYTE(1)-40 );
					rz_battery_module_temp[9]=( (INT)CAN_BYTE(4)-40 );
					}
				else if (m_poll_ml_frame == 5)
					{
					rz_battery_module_temp[10]=( (INT)CAN_BYTE(0)-40 );
					rz_battery_module_temp[11]=( (INT)CAN_BYTE(3)-40 );
					}
				int sum=0;
				for (int i=0; i<12; i++) {
						sum+=rz_battery_module_temp[i];
				}
				StdMetrics.ms_v_bat_temp->SetValue(float(sum)/12); // Calculate Mean Value
				}
			}
			break;
		}
		break;
	case 0x7bc:
		switch (pid) {
		case 0x4B7C:
			{
					// Todo
					// //+"7bc,28,39,1,4094,0,N·m,224B7C,624B7C,ff,Electric brake wheels torque request\n"  // Brake Torque
			}
			break;
			case 0x4B7D:
			{
					//"7bc,28,39,1,4094,0,N·m,224B7D,624B7D,ff,Total Hydraulic brake wheels torque request\n"
					m_v_hydraulic_brake_power->SetValue(float(CAN_12NIBL(28) -4094)*StdMetrics.ms_v_pos_speed->AsFloat()/3.6/1.28*2*3.141 );
			}
			break;
	}
	break;
	case 0x77e:
	switch (pid)
 		{
		//            case 0x3018:
		//                zoe_dcdc_temp=CAN_UINT(4);
		//                break;
	case 0x302b:
		{
		// 77e,24,31,0.015625,0,2,°C,22302b,62302b,ff\n" // inverter temperature
		// 77e,24,39,0.015625,0,2,ºC,223018,623018,ff\n" // DCDC converter temperature
		}
		break;
		}
		break;
	case 0x793:
		switch (pid)
			{
		case 0x504A:
			{
			// 793,24,39,1,20000,0,W,22504A,62504A,ff\n" // Mains active power consumed
			m_mains_power->SetValue((float(CAN_UINT(0)-20000)/1000),kW);
			}
			break;
		case 0x5063:
			{
			// 793,24,31,1,0,0,,225063,625063,ff\n"
			// Supervisor state,0:Init;1:Wait;2:ClosingS2;3:InitType;4:InitLkg;5:InitChg;6:Charge;7:ZeroAmpMode;8:EndOfChg;9:OpeningS2;10:ReadyToSleep;11:EmergencyStop;12:InitChargeDF;13:OCPStop;14:WaitS2
			rz_charge_state_local=CAN_BYTE(0);
			m_b_temp1->SetValue((INT)rz_charge_state_local);
			if (rz_charge_state_local==0) 
				{          // Init,Wait,ClosingS2,InitType,InitLkg,InitChg
				SET_CHARGE_STATE("prepare");
				}
		 	else if (rz_charge_state_local==1)
				{  // Charge
				SET_CHARGE_STATE("stopped");
				}
		 	else if (rz_charge_state_local==2)
				{  // Charge
				SET_CHARGE_STATE("prepare");
				}
		 	else if (rz_charge_state_local==3)
				{  // Charge
				SET_CHARGE_STATE("prepare");
				}
			else if (rz_charge_state_local==4)
				{  // Charge
				SET_CHARGE_STATE("prepare");
				}
			else if (rz_charge_state_local==5)
				{  // Charge
				SET_CHARGE_STATE("prepare");
				}
			else if (rz_charge_state_local==6)
				{  // Charge
				SET_CHARGE_STATE("charging");
				}
			else if (rz_charge_state_local==7)
				{  // ZeroAmpMode
				SET_CHARGE_STATE("topoff");
				}
			else if (rz_charge_state_local==8)
				{  // EndOfChg
				SET_CHARGE_STATE("done");
				}
			else if (rz_charge_state_local==9)
				{  // OpeningS2
				SET_CHARGE_STATE("prepare");
				}
			else if (rz_charge_state_local==10)
				{ // ReadyToSleep
				SET_CHARGE_STATE("stopped");
				}
			else if (rz_charge_state_local==11)
				{ //EmergencyStopp
				SET_CHARGE_STATE("stopped");
				}
			else if (rz_charge_state_local==12)
				{ //InitChargeDF
				SET_CHARGE_STATE("prepare");
				}
			else if (rz_charge_state_local==13)
				{ //OCPStop
				SET_CHARGE_STATE("stopped");
				}
			else if (rz_charge_state_local==14)
				{ //WaitS2
				SET_CHARGE_STATE("prepare");
				}
			}
			break;
			}
		break;
    }
	}

