/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "MFRC552.h"

extern void setChipSelectLow(void);
extern void setChipSelectHigh(void);
extern uint8_t spiTransfer(uint8_t data);

#define delay(ms) vTaskDelay((TickType_t) (ms) / portTICK_RATE_MS)

#define LOW 0
#define HIGH 1

#define MFRC522_PCD_IS_IN_RESET() 0
#define MFRC522_PCD_SET_RESET_PIN(x)

Uid uid;

/**
 * Writes a uint8_t to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522_PCD_WriteRegister(uint8_t reg, ///< The register to write to. One of the PCD_Register enums.
								uint8_t value		///< The value to write.
								)
{
	setChipSelectLow();			// Select slave
	spiTransfer(reg & 0x7E);	// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	spiTransfer(value);
	setChipSelectHigh();		// Release slave again
} // End MFRC522_PCD_WriteRegister()

/**
 * Writes a number of uint8_ts to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522_PCD_WriteRegisterArray(uint8_t reg, ///< The register to write to. One of the PCD_Register enums.
								size_t count, ///< The number of bytes to write to the register
								uint8_t *values ///< The values to write. uint8_t array.
								)
{
	uint8_t index;

	setChipSelectLow();		// Select slave
	spiTransfer(reg & 0x7E);		// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	for (index = 0; index < count; index++)
	{
		spiTransfer(values[index]);
	}
	setChipSelectHigh();		// Release slave again
} // End MFRC522_PCD_WriteRegister()

/**
 * Reads a uint8_t from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
uint8_t MFRC522_PCD_ReadRegister(uint8_t reg	///< The register to read from. One of the PCD_Register enums.
								)
{
	uint8_t value;
	setChipSelectLow();			// Select slave
	spiTransfer(0x80 | (reg & 0x7E));// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	value = spiTransfer(0);					// Read the value back. Send 0 to stop reading.
	setChipSelectHigh();			// Release slave again
	return value;
} // End MFRC522_PCD_ReadRegister()

/**
 * Reads a number of uint8_ts from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522_PCD_ReadRegisterArray(uint8_t reg,		///< The register to read from. One of the PCD_Register enums.
								uint8_t count,		///< The number of uint8_ts to read
								uint8_t *values,	///< uint8_t array to store the values in.
								uint8_t rxAlign	///< Only bit positions rxAlign..7 in values[0] are updated.
								)
{
	uint8_t address, index, mask, value, i;
	if (count == 0)
	{
		return;
	}
	address = 0x80 | (reg & 0x7E);	// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	index = 0;							// Index in values array.
	setChipSelectLow();		// Select slave
	count--;								// One read is performed outside of the loop
	spiTransfer(address);					// Tell MFRC522 which address we want to read
	while (index < count)
	{
		if (index == 0 && rxAlign)
		{		// Only update bit positions rxAlign..7 in values[0]
			// Create bit mask for bit positions rxAlign..7
			mask = 0;
			for (i = rxAlign; i <= 7; i++)
			{
				mask |= (1 << i);
			}
			// Read value and tell that we want to read the same address again.
			value = spiTransfer(address);
			// Apply mask to both current value of values[0] and the new data in value.
			values[0] = (values[index] & ~mask) | (value & mask);
		}
		else
		{ // Normal case
			values[index] = spiTransfer(address);	// Read value and tell that we want to read the same address again.
		}
		index++;
	}
	values[index] = spiTransfer(0);			// Read the final uint8_t. Send 0 to stop reading.
	setChipSelectHigh();			// Release slave again
} // End MFRC522_PCD_ReadRegister()

/**
 * Sets the bits given in mask in register reg.
 */
void MFRC522_PCD_SetRegisterBitMask(uint8_t reg,	///< The register to update. One of the PCD_Register enums.
									uint8_t mask	///< The bits to set.
									)
{
	uint8_t tmp = MFRC522_PCD_ReadRegister(reg);
	MFRC522_PCD_WriteRegister(reg, tmp | mask);			// set bit mask
} // End MFRC522_PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void MFRC522_PCD_ClearRegisterBitMask(uint8_t reg,	///< The register to update. One of the PCD_Register enums.
										uint8_t mask	///< The bits to clear.
										)
{
	uint8_t tmp = MFRC522_PCD_ReadRegister(reg);
	MFRC522_PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
} // End MFRC522_PCD_ClearRegisterBitMask()

/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MFRC522_PCD_CalculateCRC(	uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
								uint8_t length,	///< In: The number of uint8_ts to transfer.
								uint8_t *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low uint8_t first.
					 ) {
	uint16_t i = 5000;
	uint8_t n;

	MFRC522_PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
	MFRC522_PCD_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	MFRC522_PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
	MFRC522_PCD_WriteRegisterArray(FIFODataReg, length, data);	// Write data to the FIFO
	MFRC522_PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73�s.
	while (1) {
		n = MFRC522_PCD_ReadRegister(DivIrqReg);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		if (n & 0x04) {						// CRCIRq bit set - calculation done
			break;
		}
		if (--i == 0) {						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}
	MFRC522_PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop calculating CRC for new content in the FIFO.

	// Transfer the result from the registers to the result buffer
	result[0] = MFRC522_PCD_ReadRegister(CRCResultRegL);
	result[1] = MFRC522_PCD_ReadRegister(CRCResultRegH);
	return STATUS_OK;
} // End MFRC522_PCD_CalculateCRC()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522 chip.
 */
void MFRC522_PCD_Init(void) {
	if (MFRC522_PCD_IS_IN_RESET()) {	//The MFRC522 chip is in power down mode.
		MFRC522_PCD_SET_RESET_PIN(HIGH);		// Exit power down mode. This triggers a hard reset.
		// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
		delay(50);
	}
	else { // Perform a soft reset
		MFRC522_PCD_Reset();
	}

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	MFRC522_PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	MFRC522_PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25�s.
	MFRC522_PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	MFRC522_PCD_WriteRegister(TReloadRegL, 0xE8);

	MFRC522_PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	MFRC522_PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	MFRC522_PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End MFRC522_PCD_Init()

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void MFRC522_PCD_Reset(void) {
	MFRC522_PCD_WriteRegister(CommandReg, PCD_SoftReset);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
	delay(50);
	// Wait for the PowerDown bit in CommandReg to be cleared
	while (MFRC522_PCD_ReadRegister(CommandReg) & (1<<4)) {
		// PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
	}
} // End MFRC522_PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void MFRC522_PCD_AntennaOn(void) {
	uint8_t value = MFRC522_PCD_ReadRegister(TxControlReg);
	if ((value & 0x03) != 0x03) {
		MFRC522_PCD_WriteRegister(TxControlReg, value | 0x03);
	}
} // End MFRC522_PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void MFRC522_PCD_AntennaOff(void) {
	MFRC522_PCD_ClearRegisterBitMask(RFCfgReg, 0x03);
} // End MFRC522_PCD_AntennaOff()

/**
 * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 *
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
uint8_t MFRC522_PCD_GetAntennaGain(void) {
	return MFRC522_PCD_ReadRegister(RFCfgReg) & (0x07<<4);
} // End MFRC522_PCD_GetAntennaGain()

/**
 * Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 */
void MFRC522_PCD_SetAntennaGain(uint8_t mask) {
	if (MFRC522_PCD_GetAntennaGain() != mask) {						// only bother if there is a change
		MFRC522_PCD_ClearRegisterBitMask(RFCfgReg, (0x07<<4));		// clear needed to allow 000 pattern
		MFRC522_PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07<<4));	// only set RxGain[2:0] bits
	}
} // End MFRC522_PCD_SetAntennaGain()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MFRC522_PCD_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
									uint8_t sendLen,		///< Number of uint8_ts to transfer to the FIFO.
									uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
									uint8_t *backLen,		///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
									uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default NULL.
									uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
									BOOL checkCRC		///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
								 ) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return MFRC522_PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End MFRC522_PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MFRC522_PCD_CommunicateWithPICC(	uint8_t command,		///< The command to execute. One of the PCD_Command enums.
										uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
										uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
										uint8_t sendLen,		///< Number of uint8_ts to transfer to the FIFO.
										uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
										uint8_t *backLen,		///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
										uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits.
										uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
										BOOL checkCRC		///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
									 ) {
	uint8_t n, _validBits, errorRegValue, controlBuffer[2];
	uint32_t i;

	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming	= (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	MFRC522_PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	MFRC522_PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	MFRC522_PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	MFRC522_PCD_WriteRegisterArray(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	MFRC522_PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	MFRC522_PCD_WriteRegister(CommandReg, command);				// Execute the command
	if (command == PCD_Transceive) {
		MFRC522_PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}

	// Wait for the command to complete.
	// In MFRC522_PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86�s.
	i = 2000;
	while (1) {
		n = MFRC522_PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
		if (--i == 0) {						// The emergency break. If all other condions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}

	// Stop now if any errors except collisions were detected.
	errorRegValue = MFRC522_PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}

	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		n = MFRC522_PCD_ReadRegister(FIFOLevelReg);			// Number of uint8_ts in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of uint8_ts returned
		MFRC522_PCD_ReadRegisterArray(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = MFRC522_PCD_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}

	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		n = MFRC522_PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (n != STATUS_OK) {
			return n;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}

	return STATUS_OK;
} // End MFRC522_PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MFRC522_PICC_RequestA(uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
							uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
							) {
	return MFRC522_PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MFRC522_PICC_WakeupA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
							uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
							) {
	return MFRC522_PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MFRC522_PICC_REQA_or_WUPA(	uint8_t command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
									uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
									uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
							   ) {
	uint8_t validBits;
	uint8_t status;

	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 uint8_ts long.
		return STATUS_NO_ROOM;
	}
	MFRC522_PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = MFRC522_PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()


/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 *
 * A PICC UID consists of 4, 7 or 10 uint8_ts.
 * Only 4 uint8_ts can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID uint8_ts		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MFRC522_PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
							uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
						 ) {
	BOOL uidComplete;
	BOOL selectDone;
	BOOL useCascadeTag;
	uint8_t cascadeLevel	= 1;
	uint8_t result;
	uint8_t count;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	char currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 uint8_t standard frame + 2 uint8_ts CRC_A
	uint8_t bufferUsed;				// The number of uint8_ts used in the buffer, ie the number of uint8_ts to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted uint8_t.
	uint8_t *responseBuffer;
	uint8_t responseLength;

	uint8_t uint8_tsToCopy;
	uint8_t maxuint8_ts;
	uint8_t collisionPos;

	// Description of buffer structure:
	//		uint8_t 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		uint8_t 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete uint8_ts, Low nibble: Extra bits.
	//		uint8_t 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		uint8_t 3: UID-data
	//		uint8_t 4: UID-data
	//		uint8_t 5: UID-data
	//		uint8_t 6: BCC					Block Check Character - XOR of uint8_ts 2-5
	//		uint8_t 7: CRC_A
	//		uint8_t 8: CRC_A
	// The BCC and CRC_A is only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of uint8_ts 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	uint8_t2	uint8_t3	uint8_t4	uint8_t5
	//		========	=============	=====	=====	=====	=====
	//		 4 uint8_ts		1			uid0	uid1	uid2	uid3
	//		 7 uint8_ts		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 uint8_ts		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9

	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}

	// Prepare MFRC522
	MFRC522_PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.

	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while ( ! uidComplete) {
		// Set the Cascade Level in the SEL uint8_t, find out if we need to use the Cascade Tag in uint8_t 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 uint8_ts
				break;

			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 uint8_ts
				break;

			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;

			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}

		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		uint8_tsToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of uint8_ts needed to represent the known bits for this level.
		if (uint8_tsToCopy) {
			maxuint8_ts = useCascadeTag ? 3 : 4; // Max 4 uint8_ts in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (uint8_tsToCopy > maxuint8_ts) {
				uint8_tsToCopy = maxuint8_ts;
			}
			for (count = 0; count < uint8_tsToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}

		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while ( ! selectDone) {
			// Find out how many bits and uint8_ts to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print("SELECT: currentLevelKnownBits="); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole uint8_ts
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = MFRC522_PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 uint8_ts of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print("ANTICOLLISION: currentLevelKnownBits="); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole uint8_ts in the UID part.
				index			= 2 + count;					// Number of whole uint8_ts: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}

			// Set bit adjustments
			rxAlign = txLastBits;											// Having a seperate variable is overkill. But it makes the next line easier to read.
			MFRC522_PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

			// Transmit the buffer and receive the response.
			result = MFRC522_PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				result = MFRC522_PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (result & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				collisionPos = result & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits = collisionPos;
				count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First uint8_t is index 0.
				buffer[index]	|= (1 << count);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while ( ! selectDone)

		// We do not check the CBB - it was constructed by us above.

		// Copy the found UID uint8_ts from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		uint8_tsToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < uint8_tsToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}

		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) {		// SAK must be exactly 24 bits (1 uint8_t + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those uint8_ts are not needed anymore.
		result = MFRC522_PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while ( ! uidComplete)

	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MFRC522_PICC_HaltA(void) {
	uint8_t result;
	uint8_t buffer[4];

	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = MFRC522_PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}

	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is an success.
	result = MFRC522_PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0, NULL, 0, false);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()

/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 *
 * @return BOOL
 */
BOOL MFRC522_PICC_IsNewCardPresent() {
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);
	uint8_t result = MFRC522_PICC_RequestA(bufferATQA, &bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 *
 * @return BOOL
 */
BOOL MFRC522_PICC_ReadCardSerial() {
	uint8_t result = MFRC522_PICC_Select(&uid, 0);
	return (result == STATUS_OK);
} // End PICC_ReadCardSerial()
