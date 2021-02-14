/** @addtogroup sdio_file SDIO peripheral API
 * @ingroup peripheral_apis

 * @author @htmlonly &copy; @endhtmlonly 2021 Kirill Zhumarin <kirill.zhumarin@gmail.com>

*/
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2021 Kirill Zhumarin <kirill.zhumarin@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/sdio.h>
#include <libopencm3/stm32/rcc.h>

/**@{*/

/*---------------------------------------------------------------------------*/
/** @brief SDIO Reset.

The SDIO peripheral and all its associated configuration registers are placed in
the reset condition. The reset is effected via the RCC peripheral reset system.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_reset(uint32_t sdio)
{
	switch (sdio)
{
#if defined(SDIO_BASE) && !defined(STM32F1)
	case SDIO1:
		rcc_periph_reset_pulse(RST_SDIO);
		break;
#endif
#if defined(SDMMC1_BASE)
	case SDIO1:
		rcc_periph_reset_pulse(RST_SDMMC1);
		break;
#endif
#if defined(SDMMC2_BASE)
	case SDIO2:
		rcc_periph_reset_pulse(RST_SDMMC2);
		break;
#endif
	default:
		break;
	}
}

/** @brief SDIO Power On.

Card is clocked.

@sa sdio_power_off
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_power_on(uint32_t sdio)
{
	SDIO_POWER(sdio) = (SDIO_POWER(sdio) & ~(SDIO_POWER_PWRCTRL_MASK << SDIO_POWER_PWRCTRL_SHIFT)) |
		SDIO_POWER_PWRCTRL_PWRON;
}

/** @brief SDIO Power Off.

Card is stopped.

@sa sdio_power_on
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_power_off(uint32_t sdio)
{
	SDIO_POWER(sdio) = (SDIO_POWER(sdio) & ~(SDIO_POWER_PWRCTRL_MASK << SDIO_POWER_PWRCTRL_SHIFT)) |
		SDIO_POWER_PWRCTRL_PWROFF;
}

/** @brief SDIO Enable Flow Control.

When HW Flow Control is enabled, the meaning of the TXFIFOE and RXFIFOF interrupt
signals, please see SDIO Status register definition in Section 31.9.11 of reference manual.

@sa sdio_disable_flow_control
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_flow_control(uint32_t sdio)
{
	SDIO_CLKCR(sdio) |= SDIO_CLKCR_HWFC_EN;
}

/** @brief SDIO Disable Flow Control.

@sa sdio_disable_flow_control
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_flow_control(uint32_t sdio)
{
	SDIO_CLKCR(sdio) &= ~SDIO_CLKCR_HWFC_EN;
}

/** @brief SDIO Set Clock Phase 1

SDIO_CK generated on the falling edge of the master clock SDIOCLK

@sa sdio_set_clock_phase_positive
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_set_clock_phase_1(uint32_t sdio)
{
	SDIO_CLKCR(sdio) |= SDIO_CLKCR_NEGEDGE;
}

/** @brief SDIO Set Clock Phase 0

SDIO_CK generated on the rising edge of the master clock SDIOCLK

@sa sdio_set_clock_phase_negative
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_set_clock_phase_0(uint32_t sdio)
{
	SDIO_CLKCR(sdio) &= ~SDIO_CLKCR_NEGEDGE;
}

/** @brief SDIO Set Bus Width

@sa sdio_set_clock_phase_negative
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] bus_width Unsigned int8. Bus width value @ref sdio_bus_width.
*/

void sdio_set_bus_width(uint32_t sdio, uint32_t bus_width)
{
	SDIO_CLKCR(sdio) = (SDIO_CLKCR(sdio) & ~(SDIO_CLKCR_WIDBUS_SHIFT << SDIO_CLKCR_WIDBUS_SHIFT)) | bus_width;
}

/** @brief SDIO Enable Clock Bypass

SDIOCLK directly drives the SDIO_CK output signal.

@sa sdio_disable_clock_bypass
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_clock_bypass(uint32_t sdio)
{
	SDIO_CLKCR(sdio) |= SDIO_CLKCR_BYPASS;
}

/** @brief SDIO Disable Clock Bypass

SDIOCLK is divided according to the CLKDIV value before driving the SDIO_CK output signal.

@sa sdio_enable_clock_bypass
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_clock_bypass(uint32_t sdio)
{
	SDIO_CLKCR(sdio) &= ~SDIO_CLKCR_BYPASS;
}

/** @brief SDIO Enable Power Save

For power saving, SDIO_CK is only enabled when the bus is active.

@sa sdio_disable_power_save
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_power_save(uint32_t sdio)
{
	SDIO_CLKCR(sdio) |= SDIO_CLKCR_PWRSAV;
}

/** @brief SDIO Disable Power Save

SDIO_CK clock is always enabled.

@sa sdio_enable_power_save
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_power_save(uint32_t sdio)
{
	SDIO_CLKCR(sdio) &= ~SDIO_CLKCR_PWRSAV;
}

/** @brief SDIO Enable Clock

SDIO_CK is enabled.

@sa sdio_disable_clock
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_clock(uint32_t sdio)
{
	SDIO_CLKCR(sdio) |= SDIO_CLKCR_CLKEN;
}

/** @brief SDIO Disable Clock

SDIO_CK is disabled.

@sa sdio_enable_clock
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_clock(uint32_t sdio)
{
	SDIO_CLKCR(sdio) &= ~SDIO_CLKCR_CLKEN;
}

/** @brief SDIO Set Clock Div

Set the divide factor between the input clock (SDIOCLK) and the output clock (SDIO_CK):
SDIO_CK frequency = SDIOCLK / [CLKDIV + 2].

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] div Unsigned int8. CLKDIV setting 0...127.
*/

void sdio_set_clock_div(uint32_t sdio, uint8_t div)
{
	SDIO_CLKCR(sdio) = (SDIO_CLKCR(sdio) & ~(SDIO_CLKCR_CLKDIV_MASK << SDIO_CLKCR_CLKDIV_SHIFT)) |
		(div << SDIO_CLKCR_CLKDIV_SHIFT);
}

/** @brief SDIO Enable CPSM

Enable command path state machine (CPSM).

@sa sdio_disable_cpsm
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_cpsm(uint32_t sdio)
{
	SDIO_CMD(sdio) |= SDIO_CMD_CPSMEN;
}

/** @brief SDIO Disable CPSM

Disable command path state machine (CPSM).

@sa sdio_enable_cpsm
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_cpsm(uint32_t sdio)
{
	SDIO_CMD(sdio) &= ~SDIO_CMD_CPSMEN;
}

/** @brief SDIO Enable CPSM CE-ATA Command

Enable transfering CMD61 by CPSM,

@sa sdio_disable_cpsm_ceata_command
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_cpsm_ceata_command(uint32_t sdio)
{
	SDIO_CMD(sdio) |= SDIO_CMD_ATACMD;
}

/** @brief SDIO Disable CPSM CE-ATA Command

Disable transfering CMD61 by CPSM,

@sa sdio_enable_cpsm_ceata_command
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_cpsm_ceata_command(uint32_t sdio)
{
	SDIO_CMD(sdio) &= ~SDIO_CMD_ATACMD;
}

/** @brief SDIO Enable CPSM CE-ATA Interrupt

Enable the CE-ATA interrupt.

@sa sdio_disable_cpsm_ceata_interrupt
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_cpsm_ceata_interrupt(uint32_t sdio)
{
	SDIO_CMD(sdio) &= ~SDIO_CMD_NIEN;
}

/** @brief SDIO Disable CPSM CE-ATA Interrupt

Disable the CE-ATA interrupt.

@sa sdio_enable_cpsm_ceata_interrupt
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_cpsm_ceata_interrupt(uint32_t sdio)
{
	SDIO_CMD(sdio) |= SDIO_CMD_NIEN;
}

/** @brief SDIO Enable CPSM Command Completion Signal

Enable the CE-ATA complection signal.

@sa sdio_disable_cpsm_ceata_command_completion_signal
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_cpsm_ceata_command_completion_signal(uint32_t sdio)
{
	SDIO_CMD(sdio) |= SDIO_CMD_ENCMDCOMPL;
}

/** @brief SDIO Disable CPSM Command Completion Signal

Disable the CE-ATA complection signal.

@sa sdio_enable_cpsm_ceata_command_completion_signal
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_cpsm_ceata_command_completion_signal(uint32_t sdio)
{
	SDIO_CMD(sdio) &= ~SDIO_CMD_ENCMDCOMPL;
}

/** @brief SDIO Enable CPSM CmdPend Wait

The CPSM waits for the end of data transfer before it starts sending a command.

@sa sdio_disable_cpsm_cmdpend_wait
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_cpsm_cmdpend_wait(uint32_t sdio)
{
	SDIO_CMD(sdio) |= SDIO_CMD_WAITPEND;
}

/** @brief SDIO Disable CPSM CmdPend Wait

@sa sdio_enable_cpsm_cmdpend_wait
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_cpsm_cmdpend_wait(uint32_t sdio)
{
	SDIO_CMD(sdio) &= ~SDIO_CMD_WAITPEND;
}

/** @brief SDIO Enable CPSM Interrupt Wait

The CPSM disables command timeout and waits for an interrupt request.

@sa sdio_disable_cpsm_interrupt_wait
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_cpsm_interrupt_wait(uint32_t sdio)
{
	SDIO_CMD(sdio) |= SDIO_CMD_WAITINT;
}

/** @brief SDIO Disable CPSM Interrupt Wait

@sa sdio_enable_cpsm_interrupt_wait
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_cpsm_interrupt_wait(uint32_t sdio)
{
	SDIO_CMD(sdio) &= ~SDIO_CMD_WAITINT;
}

/** @brief SDIO Enable CPSM Response Wait

The CPSM disables waits for an response.

@sa sdio_disable_cpsm_response_wait
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_cpsm_response_wait(uint32_t sdio)
{
	SDIO_CMD(sdio) |= SDIO_CMD_WAITRESP;
}

/** @brief SDIO Disable CPSM Response Wait

@sa sdio_enable_cpsm_response_wait
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_cpsm_response_wait(uint32_t sdio)
{
	SDIO_CMD(sdio) &= ~SDIO_CMD_WAITRESP;
}

/** @brief SDIO Set CPSM Long Response Mode

The CPSM receives a 136-bit long response.

@sa sdio_set_cpsm_short_response_mode
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_set_cpsm_long_response_mode(uint32_t sdio)
{
	SDIO_CMD(sdio) |= SDIO_CMD_LONGRESP;
}

/** @brief SDIO Set CPSM Short Response Mode

The CPSM receives a short response.

@sa sdio_set_cpsm_long_response_mode
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_set_cpsm_short_response_mode(uint32_t sdio)
{
	SDIO_CMD(sdio) &= ~SDIO_CMD_LONGRESP;
}

/** @brief SDIO Set Command Index

The command index is sent to the card as part of a command message.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_set_command_index(uint32_t sdio, uint8_t index)
{
	SDIO_CMD(sdio) = (SDIO_CMD(sdio) & ~(SDIO_CMD_CMDINDEX_MASK << SDIO_CMD_CMDINDEX_SHIFT)) |
		(index << SDIO_CMD_CMDINDEX_SHIFT);
}

/** @brief SDIO Set Command Argument

Set a 32-bit command argument, which is sent to a card as part of a command message.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] sdio Unsigned int32. Command argument.
*/

void sdio_set_command_argument(uint32_t sdio, uint32_t argument)
{
	SDIO_ARG(sdio) = argument;
}

/** @brief SDIO Get Response Command

Read command index of the last command response received.
If the command response transmission does not contain the command
index field (long or OCR response), the RESPCMD field is unknown, although it must
contain 111111b (the value of the reserved field from the response).

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.

@returns Unsigned int32. Command index of the last command response received.
*/

uint8_t sdio_get_response_command(uint32_t sdio)
{
	return SDIO_RESPCMD(sdio) & (SDIO_CMD_CMDINDEX_MASK << SDIO_CMD_CMDINDEX_SHIFT);
}

/** @brief SDIO Get Response Status

Read the status of a card, which is part of the received response.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] n Unsigned int8. number of response status which need return (1, 2, 3, 4).

@returns Unsigned int32. Value of requested response status.
*/

uint32_t sdio_get_response_status(uint32_t sdio, uint8_t n)
{
	return SDIO_RESPN(sdio, n) & (SDIO_RESPCMD_MASK << SDIO_RESPCMD_SHIFT);
}

/** @brief SDIO Read Short Response

Read SDIO_RESP1 as short response.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] response uint32_t* Output array with short response data.
*/

void sdio_read_short_response(uint32_t sdio, uint32_t *response)
{
	response[0] = SDIO_RESP1(sdio) & (SDIO_RESPCMD_MASK << SDIO_RESPCMD_SHIFT);
}

/** @brief SDIO Read Long Response

Read SDIO_RESP1, SDIO_RESP2, SDIO_RESP3, SDIO_RESP4 as long response.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] response uint32_t* Output array with long response data.
*/

void sdio_read_long_response(uint32_t sdio, uint32_t *response)
{
	response[0] = SDIO_RESP1(sdio) & (SDIO_RESPCMD_MASK << SDIO_RESPCMD_SHIFT);
	response[1] = SDIO_RESP2(sdio) & (SDIO_RESPCMD_MASK << SDIO_RESPCMD_SHIFT);
	response[2] = SDIO_RESP3(sdio) & (SDIO_RESPCMD_MASK << SDIO_RESPCMD_SHIFT);
	response[3] = SDIO_RESP4(sdio) & (SDIO_RESPCMD_MASK << SDIO_RESPCMD_SHIFT);
}

/** @brief SDIO Set Timeout

Set the data timeout period, in card bus clock periods. 
A counter loads the value from the SDIO_DTIMER register, and starts decrementing when
the data path state machine (DPSM) enters the Wait_R or Busy state. If the timer reaches 0
while the DPSM is in either of these states, the timeout status flag is set.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] timeout Unsigned int32. data timeout period in card bus clock periods.
*/

void sdio_set_timeout(uint32_t sdio, uint32_t timeout)
{
	SDIO_DTIMER(sdio) = timeout;
}

/** @brief SDIO Set Data Length

Set the number of data bytes to be transferred. The value is
loaded into the data counter when data transfer starts.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] length Unsigned int32. number of data bytes to be transferred.
*/

void sdio_set_data_length(uint32_t sdio, uint32_t length)
{
	SDIO_DLEN(sdio) = length;
}

/** @brief SDIO Get Data Counter

Get current data counter.

The SDIO_DCOUNT register loads the value from the data length register (see SDIO_DLEN) when the DPSM moves from the Idle state to the Wait_R or Wait_S state. 
As data is transferred, the counter decrements the value until it reaches 0.
The DPSM then moves to the Idle state and the data status end flag, DATAEND, is set.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.

@returns Unsigned int32. Data counter value.
*/

uint32_t sdio_get_data_counter(uint32_t sdio)
{
	return SDIO_DCOUNT(sdio);
}

/** @brief SDIO Enable DPSM Data Transfer

The DPMS starts transfer data.

@sa sdio_disable_dpsm_data_transfer
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_dpsm_data_transfer(uint32_t sdio)
{
	SDIO_DCTRL(sdio) |= SDIO_DCTRL_DTEN;
}

/** @brief SDIO Disable DPSM Data Transfer

@sa sdio_enable_dpsm_data_transfer
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_dpsm_data_transfer(uint32_t sdio)
{
	SDIO_DCTRL(sdio) &= ~SDIO_DCTRL_DTEN;
}

/** @brief SDIO Enable DPSM SD I/O Specific

The DPSM performs an SD I/O-card-specific operation.

@sa sdio_disable_dpsm_sdio_specific
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_dpsm_sdio_specific(uint32_t sdio)
{
	SDIO_DCTRL(sdio) |= SDIO_DCTRL_SDIOEN;
}

/** @brief SDIO Disable DPSM SD I/O Specific

@sa sdio_enable_dpsm_sdio_specific
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_dpsm_sdio_specific(uint32_t sdio)
{
	SDIO_DCTRL(sdio) &= ~SDIO_DCTRL_SDIOEN;
}

/** @brief SDIO Set DPSM Read Wait CLK Mode

Read Wait control using SDIO_CK.

@sa sdio_set_dpsm_read_wait_d2_mode
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_set_dpsm_read_wait_ck_mode(uint32_t sdio)
{
	SDIO_DCTRL(sdio) |= SDIO_DCTRL_RWMOD;
}

/** @brief SDIO Set DPSM Read Wait D2 Mode

Read Wait control stopping SDIO_D2.

@sa sdio_set_dpsm_read_wait_ck_mode
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_set_dpsm_read_wait_d2_mode(uint32_t sdio)
{
	SDIO_DCTRL(sdio) &= ~SDIO_DCTRL_RWMOD;
}

/** @brief SDIO Enable DPSM Read Wait Stop

Enable for read wait stop (if RWSTART bit is set).

@sa sdio_disable_dpsm_read_wait_stop
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_dpsm_read_wait_stop(uint32_t sdio)
{
	SDIO_DCTRL(sdio) |= SDIO_DCTRL_RWSTOP;
}

/** @brief SDIO Disable DPSM Read Wait Stop

Read wait in progress (if RWSTART bit is set).

@sa sdio_enable_dpsm_read_wait_stop
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_dpsm_read_wait_stop(uint32_t sdio)
{
	SDIO_DCTRL(sdio) &= ~SDIO_DCTRL_RWSTOP;
}

/** @brief SDIO Enable DPSM Read Wait Start

Starts read wait operation.

@sa sdio_disable_dpsm_read_wait_start
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_dpsm_read_wait_start(uint32_t sdio)
{
	SDIO_DCTRL(sdio) |= SDIO_DCTRL_RWSTART;
}

/** @brief SDIO Disable DPSM Read Wait Start

@sa sdio_enable_dpsm_read_wait_start
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_dpsm_read_wait_start(uint32_t sdio)
{
	SDIO_DCTRL(sdio) &= ~SDIO_DCTRL_RWSTART;
}

/** @brief SDIO Set DPSM Block Size

Set the data block length when the block data transfer mode is selected.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] size Unsigned int32. block size setting @ref sdio_block_size.
*/

void sdio_set_dpsm_block_size(uint32_t sdio, uint32_t size)
{
	SDIO_DCTRL(sdio) = (SDIO_DCTRL(sdio) & ~(SDIO_DCTRL_DBLOCKSIZE_MASK << SDIO_DCTRL_DBLOCKSIZE_SHIFT)) | size;
}

/** @brief SDIO Enable DMA

@sa sdio_disable_dma
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_enable_dma(uint32_t sdio)
{
	SDIO_DCTRL(sdio) |= SDIO_DCTRL_DMAEN;
}

/** @brief SDIO Disable DMA

@sa sdio_enable_dma
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_disable_dma(uint32_t sdio)
{
	SDIO_DCTRL(sdio) &= ~SDIO_DCTRL_DMAEN;
}

/** @brief SDIO Set DPSM Block Transfer Mode

Set block data transfer mode.

@sa sdio_set_dpsm_stream_transfer_mode
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_set_dpsm_block_transfer_mode(uint32_t sdio)
{
	SDIO_DCTRL(sdio) &= ~SDIO_DCTRL_DTMODE;
}

/** @brief SDIO Set DPSM Stream Transfer Mode

Set stream or SDIO multibyte data transfer mode.

@sa sdio_set_dpsm_block_transfer_mode
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_set_dpsm_stream_transfer_mode(uint32_t sdio)
{
	SDIO_DCTRL(sdio) |= SDIO_DCTRL_DTMODE;
}

/** @brief SDIO Set DPSM Direction To Card

Set data transfer direction: from controller to card.

@sa sdio_set_dpsm_direction_from_card
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_set_dpsm_direction_to_card(uint32_t sdio)
{
	SDIO_DCTRL(sdio) &= ~SDIO_DCTRL_DTDIR;
}

/** @brief SDIO Set DPSM Direction From Card

Set data transfer direction: from card to controller.

@sa sdio_set_dpsm_direction_to_card
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
*/

void sdio_set_dpsm_direction_from_card(uint32_t sdio)
{
	SDIO_DCTRL(sdio) |= SDIO_DCTRL_DTDIR;
}

/** @brief SDIO Get Status

Get status of all signals.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.

@returns uint32_t SDIO Status @ref sdio_status_flags
*/

uint32_t sdio_get_status(uint32_t sdio)
{
	return SDIO_STA(sdio);
}

/** @brief SDIO Get Status Flag

Get status of specified flag.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] flag Unsigned int32. Status flag @ref sdio_status_flags.

@returns bool. Boolean value for flag set.
*/

bool sdio_get_status_flag(uint32_t sdio, uint32_t flag)
{
	return (SDIO_STA(sdio) & flag) == flag;
}

/** @brief SDIO Clear Status Flag

Clear status of specified flag.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] sdio Unsigned int32. Status flag for clear @ref sdio_status_clear_flags.
*/

void sdio_clear_status_flag(uint32_t sdio, uint32_t flag)
{
	SDIO_ICR(sdio) = flag;
}

/** @brief SDIO Unmask Status Flag

Unmask specified flag.

@sa sdio_mask_status_flag
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] sdio Unsigned int32. Status flag for unmasking @ref sdio_status_mask_flags.
*/

void sdio_unmask_status_flag(uint32_t sdio, uint32_t flags)
{
	SDIO_MASK(sdio) &= ~flags;
}

/** @brief SDIO Mask Status Flag

Mask specified signals.

@sa sdio_unmask_status_flag
@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] sdio Unsigned int32. Status flag for masking @ref sdio_status_mask_flags.
*/

void sdio_mask_status_flag(uint32_t sdio, uint32_t flags)
{
	SDIO_MASK(sdio) |= flags;
}

/** @brief SDIO Get FIFO Counter

Get the remaining number of words to be written to or read from the FIFO.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.

@returns sdio Unsigned int32. Remaining number of words to be written to or read from the FIFO.
*/

uint32_t sdio_get_fifo_counter(uint32_t sdio)
{
	return SDIO_FIFOCNT(sdio);
}

/** @brief SDIO Read FIFO

Read data from FIFO.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.

@returns sdio Unsigned int32. Data from FIFO.
*/

uint32_t sdio_read_fifo(uint32_t sdio)
{
	return SDIO_FIFO(sdio);
}

/** @brief SDIO Write FIFO

Write data to FIFO.

@param[in] sdio Unsigned int32. SDIO peripheral identifier @ref sdio_reg_base.
@param[in] data Unsigned int32. Data to be written to FIFO.
*/

void sdio_write_fifo(uint32_t sdio, uint32_t data)
{
	SDIO_FIFO(sdio) = data;
}

/**@}*/
