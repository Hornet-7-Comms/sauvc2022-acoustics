
// [ADC + DMA]
// https://forum.pjrc.com/threads/30171-Reconfigure-ADC-via-a-DMA-transfer-to-allow-multiple-Channel-Acquisition

// [PDB]
// https://forum.pjrc.com/threads/59001-Clock-speed-for-PDB-Tested-on-Teensy-3-2-and-3-5
// https://forum.pjrc.com/threads/24492-Using-the-PDB-on-Teensy-3

// [PDB + DMA]
// https://gist.github.com/samyk/6273cf9a45d63a50c38c8b50a39a8f52
// https://journeytoengineering.netlify.app/blog/how-to-dma-teensy-3-6-dac-waveforms/

/*
	PDB_SC_TRGSEL(15)        Select software trigger
	PDB_SC_PDBEN             PDB enable
	PDB_SC_PDBIE             Interrupt enable
	PDB_SC_CONT              Continuous mode
	PDB_SC_PRESCALER(7)      Prescaler = 128
	PDB_SC_MULT(1)           Prescaler multiplication factor = 10
	PDB_SC_DMAEN             Enable DMA
*/
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE | PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1) | PDB_SC_DMAEN)

void Setup_PDB() {
    // Enable PDB clock
    SIM_SCGC6 |= SIM_SCGC6_PDB;

	// Enable the PDB clock
	SIM_SCGC6 |= SIM_SCGC6_PDB;

	// Modulus Register
    const uint32_t mod = F_BUS / 128 / 10;
    PDB0_MOD = (uint16_t)(mod - 1);

	// Interrupt delay
	PDB0_IDLY = 0;

	// PDB status and control
	PDB0_SC = PDB_CONFIG;

	// Software trigger (reset and restart counter)
	PDB0_SC |= PDB_SC_SWTRIG;

	// Load OK
	PDB0_SC |= PDB_SC_LDOK;

	// Enable pre-trigger
    PDB0_CH0C1 = 0x0101;

	// Enable interrupt request
	NVIC_ENABLE_IRQ(IRQ_PDB);
}

void pdb_isr() {
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK; // (also clears interrupt flag)
  GPIOC_PTOR |= (1 << 5); // toggle pin 13 each interrupt
}
