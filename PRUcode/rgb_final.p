.origin 0
.entrypoint INIT
#include "rgb_final.hp"
INIT:
	MOV32	regOffset,		0x00000000								// Initialize pointer to INTC registers
	MOV32	r31,			0x00000000								// Clear SYS_EVT32
	MOV32	regVal,			(0x00000000 | HOST_NUM)					// Enable host interrupt 2
	SBCO	regVal,			CONST_PRUSSINTC,	HIESR_OFFSET,	4
	LDI		regOffset.w0,	INTC_HOSTMAP_REGS_OFFSET				// Map channel 2 to host 2
	ADD		regOffset.w0,	regOffset.w0,		HOST_NUM
	LDI		regVal.w0,		CHN_NUM
	SBCO	regVal,			CONST_PRUSSINTC,	regOffset.w0,	1
	LDI		regOffset.w0,	INTC_CHNMAP_REGS_OFFSET					// Map SYS_EVT32 interrupt to channel 2
	ADD		regOffset,		regOffset,			SYS_EVT
	LDI		regVal.b0,		CHN_NUM
	SBCO	regVal.b0,		CONST_PRUSSINTC,	regOffset.w0,	1
	MOV32	regVal,			SYS_EVT									// Make sure the SYS_EVT32 system interrupt is cleared
	SBCO	regVal,			CONST_PRUSSINTC,	SICR_OFFSET,	4
	SBCO	regVal,			CONST_PRUSSINTC,	EISR_OFFSET,	4	// Enable SYS_EVT32 system interrupt
	LDI		regVal.w0,		0x0001									// Global enable the all host interrupts
	SBCO	regVal,			CONST_PRUSSINTC,	GER_OFFSET,		2
	MOV32	regVal,			SYS_EVT									// Set up regVal to generate interrupt
START_FRAME:
	MOV32	RGB_PTR,        RGB_DATA_BASE							// Load the pointer to the RGB data
    MOV32   RGB_PTR2,       RGB2_DATA_BASE                          // Load the pointer to the second row of RGB data
    MOV32   RGBSMALL_PTR,   RGB_SMALL_BASE                          // Load the pointer to the 88X72 RGB data
	MOV32	RAW_FIELD1_PTR, RAW_DATA_BASE1							// Load the pointer to the first field
	MOV32	RAW_FIELD2_PTR, RAW_DATA_BASE2							// Load the pointer to the second field
    MOV32   RAW_FIELD3_PTR, RAW_DATA_BASE3                          // Load the pointer to the third field
    MOV32   RAW_FIELD4_PTR, RAW_DATA_BASE4                          // Load the pointer to the fourth field
	LDI		ROW_CNTR,		0										// Zero the row counter
	SLP		0														// Wait for a frame
START_ROW:
	LDI		COL_CNTR,		0										// Zero the column counter
START_PIX:
    LDI     Rsmall,         0
    LDI     Gsmall,         0
    LDI     Bsmall,         0
	LBBO	B_RAW,			RAW_FIELD1_PTR,		0,				2	// Load raw values
	LBBO	G2_RAW,			RAW_FIELD2_PTR,		0,				2
	ADD		G_FULL,			G1_RAW,				G2_RAW				// Average greens
	LSR		G,				G_FULL,				1
    MOV     Gtmp,           G
    MOV		R,				R_RAW									// Move to output register
    MOV     Rtmp,           R_RAW
    MOV		B,				B_RAW
    MOV     Btmp,           B_RAW
    ADD     Rsmall,         Rsmall,               Rtmp
    ADD     Gsmall,         Gsmall,               Gtmp
    ADD     Bsmall,         Bsmall,               Btmp
    SBBO	B,				RGB_PTR,			0,				3	// Store RGB values
    ADD		RGB_PTR,		RGB_PTR,			3					// Increment the rgb data pointer
    ADD		RAW_FIELD1_PTR,	RAW_FIELD1_PTR,		2					// Increment the local data pointers
    ADD		RAW_FIELD2_PTR,	RAW_FIELD2_PTR,		2
    LBBO	B_RAW,			RAW_FIELD1_PTR,		0,				2	// Load raw values
    LBBO	G2_RAW,			RAW_FIELD2_PTR,		0,				2
    ADD		G_FULL,			G1_RAW,				G2_RAW				// Average greens
    LSR		G,				G_FULL,				1
    MOV     Gtmp,           G
    MOV		R,				R_RAW									// Move to output register
    MOV     Rtmp,           R_RAW
    MOV		B,				B_RAW
    MOV     Btmp,           B_RAW
    ADD     Rsmall,         Rsmall,               Rtmp
    ADD     Gsmall,         Gsmall,               Gtmp
    ADD     Bsmall,         Bsmall,               Btmp
    SBBO	B,				RGB_PTR,			0,				3	// Store RGB values
    ADD		RGB_PTR,		RGB_PTR,			3					// Increment the rgb data pointer
    ADD		RAW_FIELD1_PTR,	RAW_FIELD1_PTR,		2					// Increment the local data pointers
    ADD		RAW_FIELD2_PTR,	RAW_FIELD2_PTR,		2
    LBBO	B_RAW,			RAW_FIELD3_PTR,		0,				2	// Load raw values
    LBBO	G2_RAW,			RAW_FIELD4_PTR,		0,				2
    ADD		G_FULL,			G1_RAW,				G2_RAW				// Average greens
    LSR		G,				G_FULL,				1
    MOV     Gtmp,           G
    MOV		R,				R_RAW									// Move to output register
    MOV     Rtmp,           R_RAW
    MOV		B,				B_RAW
    MOV     Btmp,           B_RAW
    ADD     Rsmall,         Rsmall,               Rtmp
    ADD     Gsmall,         Gsmall,               Gtmp
    ADD     Bsmall,         Bsmall,               Btmp
    SBBO	B,				RGB_PTR2,			0,				3	// Store RGB values
    ADD		RGB_PTR2,		RGB_PTR2,			3					// Increment the rgb data pointer
    ADD		RAW_FIELD3_PTR,	RAW_FIELD3_PTR,		2					// Increment the local data pointers
    ADD		RAW_FIELD4_PTR,	RAW_FIELD4_PTR,		2
    LBBO	B_RAW,			RAW_FIELD3_PTR,		0,				2	// Load raw values
    LBBO	G2_RAW,			RAW_FIELD4_PTR,		0,				2
    ADD		G_FULL,			G1_RAW,				G2_RAW				// Average greens
    LSR		G,				G_FULL,				1
    MOV     Gtmp,           G
    MOV		R,				R_RAW									// Move to output register
    MOV     Rtmp,           R_RAW
    MOV		B,				B_RAW
    MOV     Btmp,           B_RAW
    ADD     Rsmall,         Rsmall,               Rtmp
    ADD     Gsmall,         Gsmall,               Gtmp
    ADD     Bsmall,         Bsmall,               Btmp
    SBBO	B,				RGB_PTR2,			0,				3	// Store RGB values
    ADD		RGB_PTR2,		RGB_PTR2,			3					// Increment the rgb data pointer
    ADD		RAW_FIELD3_PTR,	RAW_FIELD3_PTR,		2					// Increment the local data pointers
    ADD		RAW_FIELD4_PTR,	RAW_FIELD4_PTR,		2
    LSR     G,              Gsmall,             2
    LSR     R,              Rsmall,             2
    LSR     B,              Bsmall,             2
    SBBO    B,              RGBSMALL_PTR,       0,              3
    ADD     RGBSMALL_PTR,   RGBSMALL_PTR,       3
	ADD		COL_CNTR,		COL_CNTR,			2					// Increment the column counter
	QBNE	START_PIX,		COL_CNTR,			IMG_COLS			// Loop if the row isn't done
    ADD		RAW_FIELD1_PTR,	RAW_FIELD1_PTR,		176					// Move the raw data pointers to next line
    ADD		RAW_FIELD1_PTR,	RAW_FIELD1_PTR,		176
    ADD		RAW_FIELD1_PTR,	RAW_FIELD1_PTR,		176
    ADD		RAW_FIELD1_PTR,	RAW_FIELD1_PTR,		176
    ADD		RAW_FIELD1_PTR,	RAW_FIELD1_PTR,		176
    ADD		RAW_FIELD1_PTR,	RAW_FIELD1_PTR,		176    
    ADD		RAW_FIELD2_PTR,	RAW_FIELD2_PTR,		176
    ADD		RAW_FIELD2_PTR,	RAW_FIELD2_PTR,		176
    ADD		RAW_FIELD2_PTR,	RAW_FIELD2_PTR,		176
    ADD		RAW_FIELD2_PTR,	RAW_FIELD2_PTR,		176
    ADD		RAW_FIELD2_PTR,	RAW_FIELD2_PTR,		176
    ADD		RAW_FIELD2_PTR,	RAW_FIELD2_PTR,		176
    ADD		RAW_FIELD3_PTR,	RAW_FIELD3_PTR,		176					// Move the raw data pointers to next line
    ADD		RAW_FIELD3_PTR,	RAW_FIELD3_PTR,		176
    ADD		RAW_FIELD3_PTR,	RAW_FIELD3_PTR,		176
    ADD		RAW_FIELD3_PTR,	RAW_FIELD3_PTR,		176
    ADD		RAW_FIELD3_PTR,	RAW_FIELD3_PTR,		176
    ADD		RAW_FIELD3_PTR,	RAW_FIELD3_PTR,		176    
    ADD		RAW_FIELD4_PTR,	RAW_FIELD4_PTR,		176
    ADD		RAW_FIELD4_PTR,	RAW_FIELD4_PTR,		176
    ADD		RAW_FIELD4_PTR,	RAW_FIELD4_PTR,		176
    ADD		RAW_FIELD4_PTR,	RAW_FIELD4_PTR,		176
    ADD		RAW_FIELD4_PTR,	RAW_FIELD4_PTR,		176
    ADD		RAW_FIELD4_PTR,	RAW_FIELD4_PTR,		176    
    ADD     RGB_PTR,        RGB_PTR,            176
    ADD     RGB_PTR,        RGB_PTR,            176
    ADD     RGB_PTR,        RGB_PTR,            176    
    ADD     RGB_PTR2,        RGB_PTR2,            176
    ADD     RGB_PTR2,        RGB_PTR2,            176
    ADD     RGB_PTR2,        RGB_PTR2,            176    
	ADD		ROW_CNTR,		ROW_CNTR,			2					// Increment the row counter
	QBNE	START_ROW,		ROW_CNTR,			IMG_ROWS			// Loop if the image isn't done
	MOV32	r31,			SYS_EVT									// Generate SYS_EVT32 by event out mapping
	SBCO	EVENT,			CONST_PRUSSINTC,	SICR_OFFSET,	4	// Clear SYS_EVT32
	JMP		START_FRAME												// Restart