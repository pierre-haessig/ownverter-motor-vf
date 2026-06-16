/*
 * Copyright (c) 2026 Pierre Haessig
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  Application for inverter driving a brushless motor,
 *         with an open loop V/f control,
 *         using the three-phase OwnVerter board.
 *
 * @author Pierre Haessig <pierre.haessig@centralesupelec.fr>
 */

/* --------------OWNTECH APIs---------------------------------- */
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"

/* OWNTECH CONTROL LIBRARY (including trigonometric functions) */
#include "control_factory.h"
#include "transform.h"

#include "zephyr/console/console.h"

/* --------------SETUP AND LOOP FUNCTIONS DECLARATION------------------- */

/* Set up of hardware and software at board startup */
void setup_routine();

/* Interaction with the user on the serial monitor (slow background task) */
void user_interface_task();
/* Displaying board status messages on the serial monitor (slow background task) */
void status_display_task();
/* Power converter control (critical periodic task) */
void control_task();
/* Compute duty cycles (subroutine of control task)*/
void compute_duties();
/* Read analog measurements (subroutine of control task)*/
void read_measurements();

/* -------------- VARIABLES DECLARATIONS------------------- */

static const float32_t T_control = 100e-6F; // Control task period (s)
static const uint32_t T_control_micro = (uint32_t)(T_control * 1.e6F); // Control task period (integer number of µs)

/* SINUSOIDAL SIGNAL GENERATION VARIABLES */
static float32_t v_freq = 10.0; // inverter voltage frequency (Hz)
static float32_t v_angle = 0.0; // inverter voltage angle (rad)
const float32_t FREQ_INCREMENT = 10.0; // frequency up or down increment (Hz)
float32_t Vi_ref = 0.0; // Inverter voltage reference (V)
const float32_t VI_STEP = 0.5; // Inverter voltage increment/decrement step (V)

/* BOARD POWER CONVERSION STATE VARIABLES */
static bool power_enable = false; // Power conversion state of the leg (PWM activation state)
static float32_t duty_a, duty_b, duty_c; // three-phase PWM duty cycle (phases a, b, c)

const uint8_t PGPIO=41; // Pin used as GPIO (e.g. for external scope trigger, Pin 41=PB10)

/* Possible modes for the OwnTech board */
enum serial_interface_menu_mode
{
	IDLE_MODE = 0,
	POWER_MODE
};

uint8_t mode = IDLE_MODE; // Currently user-requested mode

/* COMMUNICATION AND MEASUREMENT VARIABLES */

uint8_t received_serial_char; // Temporary storage for serial monitor character reception

/* Measurement variables */

// DC side variables
static float32_t V_dc; // DC bus voltage (V)
static float32_t I_dc; // Current drawn from the DC bus (A)

// static float32_t Va, Vb, Vc; // AC-side phase voltages
static float32_t Ia, Ib, Ic; // AC-side phase currents

static float meas_data; // Temporary storage for measured value

// Vdc lowpass filter (5 ms time constant)
static LowPassFirstOrderFilter vdc_filter = controlLibFactory.lowpassfilter(T_control, 5.0e-3F);
static float32_t V_dc_filt; // DC bus voltage, lowpass filtered (V)
static float32_t inv_V_dc_filt; // 1/Vdc, with a 1/V_DC_MIN bound


/* -------------- SETUP FUNCTION -------------------------------*/

/**
 * Setup routine, called at board startup.
 * It is used to initialize the board (spin microcontroller and power shield)
 * and the application (set tasks).
 */
void setup_routine()
{
	spin.led.turnOn(); // Blink LED at board startup

	/* Set the high switch convention for all legs */
	shield.power.initBuck(ALL);
	shield.power.setDutyCycleMin(ALL, 0.0);
	shield.power.setDutyCycleMax(ALL, 1.0);

	/* GPIO pin (e.g. for external scope trigger) */
	spin.gpio.configurePin(PGPIO, OUTPUT);
	spin.gpio.resetPin(PGPIO);

	/* Setup all the measurements */
	shield.sensors.enableDefaultOwnverterSensors();

	/* Declare tasks */
	uint32_t app_task_number = task.createBackground(status_display_task);
	uint32_t com_task_number = task.createBackground(user_interface_task);
	task.createCritical(control_task, T_control_micro);

	/* Start tasks */
	task.startBackground(app_task_number);
	task.startBackground(com_task_number);
	task.startCritical();
}

/* --------------LOOP FUNCTIONS (TASKS) ------------------------------- */

/**
 * User interface task, running in a loop in the background.
 * It allows controlling the application through the serial monitor.
 *
 * It waits for the user to press a key to select an action.
 * In particular, 'h' displays the help menu.
 */
void user_interface_task()
{
	received_serial_char = console_getchar();
	switch (received_serial_char) {
	case 'h':
		/* ----------SERIAL INTERFACE MENU----------------------- */

		printk( " ______________________________________________ \n"
				"|     ------- MENU ---------                   |\n"
				"|     press i   : idle mode                    |\n"
				"|     press p   : power mode                   |\n"
				"|     press u   : voltage amplitude UP         |\n"
				"|     press j   : voltage amplitude DOWN       |\n"
				"|     press f   : frequency UP                 |\n"
				"|     press v   : frequency DOWN               |\n"
				"|______________________________________________|\n\n");

		/* ------------------------------------------------------ */
		break;
	case 'i':
		printk("Idle mode request\n");
		mode = IDLE_MODE;
		break;
	case 'p':
		printk("Power mode request\n");
		mode = POWER_MODE;
		break;
	case 'u':
		Vi_ref += VI_STEP;
		printk("Amplitude UP (%.1f V) \n", (double) Vi_ref);
		break;
	case 'j':
		Vi_ref -= VI_STEP;
		printk("Amplitude DOWN (%.1f V) \n", (double) Vi_ref);
		break;
	case 'f':
		v_freq += FREQ_INCREMENT;
		printk("Frequency UP (%.2f Hz) \n", (double) v_freq);
		break;
	case 'v':
		v_freq -= FREQ_INCREMENT;
		printk("Frequency DOWN (%.2f Hz) \n", (double) v_freq);
		break;
	default:
		break;
	}
}

/**
 * Board status display task, called pseudo-periodically.
 * It displays board measurements on the serial monitor
 *
 * It also sets the board LED (blinking when POWER_MODE).
 */
void status_display_task()
{
	if (mode == IDLE_MODE) {
		spin.led.turnOn(); // Constantly ON led when IDLE
		// Display state:
		printk("IDL: ");

	} else if (mode == POWER_MODE) {
		spin.led.toggle(); // Blinking LED when POWER
		// Display state:
		printk("POW: ");
	}
	// Display duty cycle reference and frequency:
	printk("a=%4.1fV ", (double) Vi_ref);
	printk("@%.0f Hz ", (double) v_freq);
	printk("| ");
	// Display measurements
	printk("Vdc %5.2f V, ", (double) V_dc);
	printk("Idc %4.2f A, ", (double) I_dc);
	printk("\n");
	task.suspendBackgroundMs(200);
}

/* Read measurements from analog sensors, possibly applying some filters,
   through microcontroller ADCs (Analog to Digital Converters).

   Measured signals:
   - currents: Ia, Ib, Ic, I_dc
   - voltages: V_high (with smoothed lowpass filtered version)
 */
inline void read_measurements()
{
	meas_data = shield.sensors.getLatestValue(I1_LOW);
	if (meas_data != NO_VALUE) {
		Ia = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(I2_LOW);
	if (meas_data != NO_VALUE) {
		Ib = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(I3_LOW);
	if (meas_data != NO_VALUE) {
		Ic = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(I_HIGH);
	if (meas_data != NO_VALUE) {
		I_dc = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V_HIGH);
	if (meas_data != NO_VALUE) {
		V_dc = meas_data;
	}

	/* Apply filters */
	// Smooth V_high (lowpass)
	V_dc_filt = vdc_filter.calculateWithReturn(V_dc);
}

/* Convert inverter leg voltage to duty cycle, including saturation

Leg voltage in the [-Vdc/2, +Vdc/2] interval is mapped to [0,1],
meaning that the duty cycle offset is added automatically.
*/
inline float32_t voltage_to_duty(float32_t Vleg, float32_t inverse_Vdc)
{
	static float32_t duty_raw;
	const float32_t duty_offset = 0.50F;
	duty_raw = Vleg * inverse_Vdc + duty_offset;
	if (duty_raw > 1.0F) {
		return 1.0F;
	}
	else if (duty_raw < 0.0F) {
		return 0.0F;
	}
	else {
		return duty_raw;
	}
	// TODO: raise flag if duty saturation.
}

/* Compute sinusoidal duty cycles duty_abc from Vi_ref and V_high_filtered */
inline void compute_duties()
{
	// Update inverter phase (∫ω(t).dt, computed with Euler approximation, modulo 2π)
	float32_t omega = 2*PI*v_freq; // frequency conversion (Hz -> rad/s): ω = 2π.f
	v_angle = ot_modulo_2pi(v_angle + omega*T_control);

	const float32_t V_DC_MIN = 10; // min DC voltage
	// Invert Vdc with a bound
	if (V_dc_filt > V_DC_MIN ) {
		inv_V_dc_filt = 1.0F / V_dc_filt;
	}
	else {
		inv_V_dc_filt = 1.0F / V_dc_filt;
	}
	const float32_t duty_offset = 0.5;
	duty_a = voltage_to_duty(Vi_ref * ot_sin(v_angle), inv_V_dc_filt);
	duty_b = voltage_to_duty(Vi_ref * ot_sin(v_angle - 2.0/3.0*PI), inv_V_dc_filt);
	duty_c = voltage_to_duty(Vi_ref * ot_sin(v_angle - 4.0/3.0*PI), inv_V_dc_filt);
}

/* Apply legs duty cycles to the PWM generators */
inline void apply_duties()
{
	shield.power.setDutyCycle(LEG1, duty_a);
	shield.power.setDutyCycle(LEG2, duty_b);
	shield.power.setDutyCycle(LEG3, duty_c);
}

/**
 * This is the code loop of the critical task.
 * It is executed every T_control seconds (100 µs by default).
 *
 * Actions:
 * - measure voltage and currents (in subfunction)
 * - compute duty cycle (in subfunction)
 * - control the power converter leg (ON/OFF state and duty cycle)
 */
void control_task()
{
	/* Retrieve sensor values */
	read_measurements();

	/* Compute sinusoidal duty cycles*/
	compute_duties();

	/* Manage POWER/IDLE modes */
	if (mode == IDLE_MODE) {
		if (power_enable == true) {
			shield.power.stop(ALL);
			spin.gpio.resetPin(PGPIO); // externally signal Power OFF
		}
		power_enable = false;
	} else if (mode == POWER_MODE) {
		/* Set duty cycles of all three legs */
		apply_duties();
		/* Set POWER ON */
		if (!power_enable) {
			power_enable = true;
			shield.power.start(ALL);
			spin.gpio.setPin(PGPIO); // externally signal Power ON
		}
	}
}

/**
 * Main function of the application.
 * This function is generic and does not need editing.
 */
int main(void)
{
	setup_routine();
	return 0;
}