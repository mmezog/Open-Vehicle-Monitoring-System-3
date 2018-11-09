/*
;    Project:       Open Vehicle Monitor System
;    Date:          14th March 2017
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2011       Michael Stegen / Stegen Electronics
;    (C) 2011-2017  Mark Webb-Johnson
;    (C) 2011        Sonny Chen @ EPRO/DX
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include "ovms_log.h"
static const char *TAG = "v-renaultzoe";

#include <stdio.h>
#include "pcp.h"
#include "vehicle_renaultzoe.h"
#include "ovms_metrics.h"
#include "metrics_standard.h"

#define VERSION "0.1.0"
// Polling types ("modes") supported: see vehicle.h
//
// polltime[state] = poll period in seconds, i.e. 10 = poll every 10 seconds
//
// state: 0..2; set by vehicle_poll_setstate()
//     0=off, 1=on, 2=charging
//
// Use broadcasts to query all ECUs. The request is sent to ID 0x7df (broadcast
// listener) and the poller accepts any response ID from 0x7e8..0x7ef.
// The moduleid is not used for broadcasts, just needs to be != 0.
//
// To poll a specific ECU, send to its specific ID (one of 0x7e0..0x7e7).
// ECUs will respond using their specific ID plus 8 (= range 0x7e8..0x7ef).

static const OvmsVehicle::poll_pid_t vehicle_renaultzoe_polls[] =
	{
	{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2002, {  10, 10,  10 } },  // SOC
	{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2006, {  10, 10,  10 } },  // Odometer
	{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3203, {  10, 10,  10 } },  // Battery Voltage
	{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3204, {  10, 10,  10 } },  // Battery Current
	{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3028, {  10, 10,  10 } },  // 12Battery Current
	// 7ec,24,39,.005,0,0,kwh,22320C,62320C,ff,Available discharge Energy
	{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x320C, {  10, 10,  10 } },  // Available discharge Energy
	// 7ec,30,31,1,0,0,,223332,623332,ff,Electrical Machine functionning Authorization,0:Not used;1:Inverter Off;2:Inverter On\n" //
	//{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3332, {  10, 10,  10 } },  //  Inverter Off: 1; Inverter On: 2
	//{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2005, {  10, 10,  10 } },  // 12Battery Voltage
	//{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3206, {  10, 10,  10 } },  // Battery SOH
	{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x33dc, {  10, 10,  10 } },  // Consumed Domnestic Engergy
	//{ 0x75a, 0x77e, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3018, {  10, 10,  10 } },  // DCDC Temp
	//{ 0x7e4, 0x77e, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x302b, {  10, 10,  10 } },  // Inverter Temp
	{ 0x792, 0x793, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x504A, {  10, 10,  10 } },  // Mains active Power consumed
	{ 0x792, 0x793, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x5063, {  10, 10,  10 } },  // Charging State
	// { 0x792, 0x793, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x5062, {  10, 10,  10 } },  // Ground Resistance
	{ 0x79b, 0x7bb, VEHICLE_POLL_TYPE_OBDIIGROUP, 0x04, {  10, 10,  10 } },  // Temp Bat Module 1
	//+"7bc,28,39,1,4094,0,N·m,224B7C,624B7C,ff,Electric brake wheels torque request\n" //
	//+ "Uncoupled Braking Pedal,2197,V,7bc,79c,UBP,-,5902ff\n"
	{ 0x79c, 0x7bc, VEHICLE_POLL_TYPE_OBDIIGROUP, 0x04, {  120, 1,  120 } }, // Braking Pedal
	// -END-
	{ 0, 0, 0x00, 0x00, { 0, 0, 0 } }

	};

// Init
OvmsVehicleRenaultZoe::OvmsVehicleRenaultZoe()
  {
  ESP_LOGI(TAG, "Renault Zoe vehicle module");
	StdMetrics.ms_v_bat_soc->SetValue(0);
	StdMetrics.ms_v_bat_12v_current->SetValue(0);
	StdMetrics.ms_v_bat_range_est->SetValue(0);
	//StdMetrics.ms_v_bat_cac->SetValue(0);
	StdMetrics.ms_v_bat_current->SetValue(0);
	StdMetrics.ms_v_bat_power->SetValue(0);
	StdMetrics.ms_v_charge_pilot->SetValue(false);

	memset( m_vin, 0, sizeof(m_vin));
	memset( rz_battery_cell_voltage ,0, sizeof(rz_battery_cell_voltage));
	memset( rz_battery_module_temp, 0, sizeof(rz_battery_module_temp));
	memset( rz_tpms_id, 0, sizeof(rz_tpms_id));
	
	rz_obc_volt = 230;
	rz_battery_current = 0;
	
	rz_battery_cum_charge_current = 0;
	rz_battery_cum_discharge_current = 0;
	rz_battery_cum_charge_power = 0;
	rz_battery_cum_discharge_power = 0;
	
	rz_charge_bits.Charging = false;
	rz_charge_bits.FanStatus = 0;
	
	rz_battery_min_temperature = 0;
	rz_battery_max_temperature = 0;
	
	rz_heatsink_temperature = 0;
	rz_charge_pilot_current = 0;
	
	rz_send_can.id = 0;
	rz_send_can.status = 0;
	memset( rz_send_can.byte, 0, sizeof(rz_send_can.byte));
	
	rz_maxrange = CFG_DEFAULT_MAXRANGE;
	
	// C-Bus
	RegisterCanBus(1, CAN_MODE_ACTIVE, CAN_SPEED_500KBPS);
	
	MyConfig.RegisterParam("x.rz", "Renault Zoe", true, true);
	ConfigChanged(NULL);
	
	// init metrics:
	m_b_cell_volt_max = MyMetrics.InitFloat("x.rz.m.b.cell.volt.max", 10, 0, Volts);
	m_b_cell_volt_min = MyMetrics.InitFloat("x.rz.m.b.cell.volt.min", 10, 0, Volts);
	m_b_bat_avail_energy = MyMetrics.InitFloat("x.rz.m.b.bat.avail.energy", 10, 0, kWh);
	m_b_bat_max_charge_power = MyMetrics.InitFloat("x.rz.m.b.bat.max.charge.power", 10, 0, kW);
	m_v_hydraulic_brake_power = MyMetrics.InitFloat("x.rz.m.v.hydraulic.brake.power", 10, 0, kW);
	m_mains_power = MyMetrics.InitFloat("x.rz.m.mains.power",10,0, kW);
	m_bat_heat_sink_temp = MyMetrics.InitFloat("x.rz.m.bat.heat.sink.temp",10,0,Celcius);
	m_charge_pilot_current = MyMetrics.InitInt("x.rz.m.charge.pilot.current",10,0,Amps);
	m_b_temp1 = MyMetrics.InitInt("x.rz.m.bat.t1",10,0, Celcius);
	m_b_temp2 = MyMetrics.InitInt("x.rz.m.bat.t2",10,0, Celcius);
	m_b_temp3 = MyMetrics.InitInt("x.rz.m.bat.t3",10,0, Celcius);
	m_b_temp4 = MyMetrics.InitInt("x.rz.m.bat.t4",10,0, Celcius);
	m_b_temp5 = MyMetrics.InitInt("x.rz.m.bat.t5",10,0, Celcius);
	m_b_temp6 = MyMetrics.InitInt("x.rz.m.bat.t6",10,0, Celcius);
	m_b_temp7 = MyMetrics.InitInt("x.rz.m.bat.t7",10,0, Celcius);
	m_b_temp8 = MyMetrics.InitInt("x.rz.m.bat.t8",10,0, Celcius);
	m_b_temp9 = MyMetrics.InitInt("x.rz.m.bat.t9",10,0, Celcius);
	m_b_temp10 = MyMetrics.InitInt("x.rz.m.bat.t10",10,0, Celcius);
	m_b_temp11 = MyMetrics.InitInt("x.rz.m.bat.t11",10,0, Celcius);
	m_b_temp12 = MyMetrics.InitInt("x.rz.m.bat.t12",10,0, Celcius);
	
	
	PollSetPidList(m_can1,vehicle_renaultzoe_polls);
	PollSetState(0);
  }

OvmsVehicleRenaultZoe::~OvmsVehicleRenaultZoe()
  {
  ESP_LOGI(TAG, "Shutdown Renault Zoe vehicle module");
  }


/**
 * ConfigChanged: reload single/all configuration variables
 */

void OvmsVehicleRenaultZoe::ConfigChanged(OvmsConfigParam* param)
	{
	ESP_LOGD(TAG, "Renault Zoe reload configuration");
	
	// Instances:
	//  x.rz
	//  batteryCapacity   Battery capacity in wH (Default: 270000)
	//  suffsoc           Sufficient SOC [%] (Default: 0=disabled)
	//  suffrange         Sufficient range [km] (Default: 0=disabled)
	//  maxrange          Maximum ideal range at 20 °C [km] (Default: 160)
	//
	//  canwrite          Bool: CAN write enabled (Default: no)
	//  autoreset         Bool: SEVCON reset on error (Default: yes)
	//  kickdown          Bool: SEVCON automatic kickdown (Default: yes)
	//  autopower         Bool: SEVCON automatic power level adjustment (Default: yes)
	//  console           Bool: SimpleConsole inputs enabled (Default: no)
	//
	
	rz_battery_capacity = (float)MyConfig.GetParamValueInt("x.rz", "batteryCapacity", CGF_DEFAULT_BATTERY_CAPACITY);
    
	}

////////////////////////////////////////////////////////////////////////
// vehicle_renaultzoe_car_on()
// Takes care of setting all the state appropriate when the car is on
// or off. Centralized so we can more easily make on and off mirror
// images.
void OvmsVehicleRenaultZoe::vehicle_renaultzoe_car_on(bool isOn)
	{
	if (isOn && !StdMetrics.ms_v_env_on->AsBool())
		{
		// Car is ON
		StdMetrics.ms_v_env_on->SetValue(isOn);
		StdMetrics.ms_v_env_awake->SetValue(isOn);
		PollSetState(1);
		//TODO net_req_notification(NET_NOTIFY_ENV);
		}
	else if(!isOn && StdMetrics.ms_v_env_on->AsBool())
		{
		// Car is OFF
		StdMetrics.ms_v_env_on->SetValue( isOn );
		StdMetrics.ms_v_env_awake->SetValue( isOn );
		StdMetrics.ms_v_pos_speed->SetValue( 0 );
		PollSetState(0);
		}
}


/**
 * Ticker1: Called every second
 */
void OvmsVehicleRenaultZoe::Ticker1(uint32_t ticker)
	{
	StdMetrics.ms_v_bat_voltage->SetValue( rz_bat_voltage );
	StdMetrics.ms_v_bat_current->SetValue( rz_bat_current );
	StdMetrics.ms_v_bat_power->SetValue(rz_bat_voltage * rz_bat_current/1000);
	if (StdMetrics.ms_v_bat_12v_voltage->AsFloat() > 13.5)
		{
			vehicle_renaultzoe_car_on(true);
		}
	else
		{
			vehicle_renaultzoe_car_on(false);
		}
	}

/**
 *  Sets the charge metrics
 */
void OvmsVehicleRenaultZoe::SetChargeMetrics(float voltage, float current, float climit, bool chademo)
	{
  StdMetrics.ms_v_charge_voltage->SetValue( voltage, Volts );
  StdMetrics.ms_v_charge_current->SetValue( current, Amps );
  StdMetrics.ms_v_charge_climit->SetValue( climit, Amps);
	}

class OvmsVehicleRenaultZoeInit
  {
  public: OvmsVehicleRenaultZoeInit();
	} MyOvmsVehicleRenaultZoeInit  __attribute__ ((init_priority (9000)));

OvmsVehicleRenaultZoeInit::OvmsVehicleRenaultZoeInit()
  {
  ESP_LOGI(TAG, "Registering Vehicle: Renault Zoe (9000)");

  MyVehicleFactory.RegisterVehicle<OvmsVehicleRenaultZoe>("RZ","Renault Zoe");
  }

