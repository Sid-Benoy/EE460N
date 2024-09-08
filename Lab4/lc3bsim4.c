/***************************************************************/
/*                                                             */
/*   LC-3b Simulator                                           */
/*                                                             */
/*   EE 460N                                                   */
/*   The University of Texas at Austin                         */
/*                                                             */
/***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/***************************************************************/
/*                                                             */
/* Files:  ucode        Microprogram file                      */
/*         isaprogram   LC-3b machine language program file    */
/*                                                             */
/***************************************************************/

/***************************************************************/
/* These are the functions you'll have to write.               */
/***************************************************************/

void eval_micro_sequencer();
void cycle_memory();
void eval_bus_drivers();
void drive_bus();
void latch_datapath_values();

void setCC(int num);

/***************************************************************/
/* A couple of useful definitions.                             */
/***************************************************************/
#define FALSE 0
#define TRUE  1

/***************************************************************/
/* Use this to avoid overflowing 16 bits on the bus.           */
/***************************************************************/
#define Low16bits(x) ((x) & 0xFFFF)

/***************************************************************/
/* Definition of the control store layout.                     */
/***************************************************************/
#define CONTROL_STORE_ROWS 64
#define INITIAL_STATE_NUMBER 18

/***************************************************************/
/* Definition of bit order in control store word.              */
/***************************************************************/
enum CS_BITS {                                                  
    IRD,
    COND2, COND1, COND0,
    J5, J4, J3, J2, J1, J0,
    LD_MAR,
    LD_MDR,
    LD_IR,
    LD_BEN,
    LD_REG,
    LD_CC,
    LD_PC,
    GATE_PC,
    GATE_MDR,
    GATE_ALU,
    GATE_MARMUX,
    GATE_SHF,
    PCMUX1, PCMUX0,
    DRMUX,
    SR1MUX,
    ADDR1MUX,
    ADDR2MUX1, ADDR2MUX0,
    MARMUX,
    ALUK1, ALUK0,
    MIO_EN,
    R_W,
    DATA_SIZE,
    LSHF1,
/* MODIFY: you have to add all your new control signals */
   LD_PSR,
   LD_SAVED_SSP,
   LD_SAVED_USP,
   LD_VECTOR,
   GATE_SP,
   GATE_VECTOR,
   GATE_PSR,
   GATE_DECPC,
   VECTORMUX1,
   VECTORMUX0,
   SPMUX1,
   SPMUX0,
   SPMUXEXTRA,
   CONTROL_STORE_BITS
} CS_BITS;

/***************************************************************/
/* Functions to get at the control bits.                       */
/***************************************************************/
int GetIRD(int *x)           { return(x[IRD]); }
int GetCOND(int *x)          { return((x[COND1] << 1) + x[COND0]); }
int GetJ(int *x)             { return((x[J5] << 5) + (x[J4] << 4) +
				      (x[J3] << 3) + (x[J2] << 2) +
				      (x[J1] << 1) + x[J0]); }
int GetLD_MAR(int *x)        { return(x[LD_MAR]); }
int GetLD_MDR(int *x)        { return(x[LD_MDR]); }
int GetLD_IR(int *x)         { return(x[LD_IR]); }
int GetLD_BEN(int *x)        { return(x[LD_BEN]); }
int GetLD_REG(int *x)        { return(x[LD_REG]); }
int GetLD_CC(int *x)         { return(x[LD_CC]); }
int GetLD_PC(int *x)         { return(x[LD_PC]); }
int GetGATE_PC(int *x)       { return(x[GATE_PC]); }
int GetGATE_MDR(int *x)      { return(x[GATE_MDR]); }
int GetGATE_ALU(int *x)      { return(x[GATE_ALU]); }
int GetGATE_MARMUX(int *x)   { return(x[GATE_MARMUX]); }
int GetGATE_SHF(int *x)      { return(x[GATE_SHF]); }
int GetPCMUX(int *x)         { return((x[PCMUX1] << 1) + x[PCMUX0]); }
int GetDRMUX(int *x)         { return(x[DRMUX]); }
int GetSR1MUX(int *x)        { return(x[SR1MUX]); }
int GetADDR1MUX(int *x)      { return(x[ADDR1MUX]); }
int GetADDR2MUX(int *x)      { return((x[ADDR2MUX1] << 1) + x[ADDR2MUX0]); }
int GetMARMUX(int *x)        { return(x[MARMUX]); }
int GetALUK(int *x)          { return((x[ALUK1] << 1) + x[ALUK0]); }
int GetMIO_EN(int *x)        { return(x[MIO_EN]); }
int GetR_W(int *x)           { return(x[R_W]); }
int GetDATA_SIZE(int *x)     { return(x[DATA_SIZE]); } 
int GetLSHF1(int *x)         { return(x[LSHF1]); }
/* MODIFY: you can add more Get functions for your new control signals */
int GetGATE_SP(int* x)       { return(x[GATE_SP]); }
int GetGATE_PSR(int* x)      { return(x[GATE_PSR]); }
int GetGATE_VECTOR(int* x)   { return(x[GATE_VECTOR]); }
int GetGATE_DECPC(int* x)    { return(x[GATE_DECPC]); }
int GetLD_PSR(int* x)        { return(x[LD_PSR]); }
int GetLD_VECTOR(int* x)     { return(x[LD_VECTOR]); }
int GetLD_SAVED_SSP(int* x)     { return(x[LD_SAVED_SSP]); }
int GetLD_USP_SP(int* x)        { return(x[LD_SAVED_USP]); }
int GetVECTORMUX(int* x)     { return((x[VECTORMUX1] << 1) + x[VECTORMUX0]); }
int GetSPMUX(int* x)         { return((x[SPMUX1] << 1) + x[SPMUX0]); }
int GetSPMUXEXTRA(int* x)    { return(x[SPMUXEXTRA]); }

/***************************************************************/
/* The control store rom.                                      */
/***************************************************************/
int CONTROL_STORE[CONTROL_STORE_ROWS][CONTROL_STORE_BITS];

/***************************************************************/
/* Main memory.                                                */
/***************************************************************/
/* MEMORY[A][0] stores the least significant byte of word at word address A
   MEMORY[A][1] stores the most significant byte of word at word address A 
   There are two write enable signals, one for each byte. WE0 is used for 
   the least significant byte of a word. WE1 is used for the most significant 
   byte of a word. */

#define WORDS_IN_MEM    0x08000 
#define MEM_CYCLES      5
int MEMORY[WORDS_IN_MEM][2];

/***************************************************************/

/***************************************************************/

/***************************************************************/
/* LC-3b State info.                                           */
/***************************************************************/
#define LC_3b_REGS 8
#define LC_3b_ARR_SIZE 17
#define INT_VECTOR 1
#define EXCV_PROTECTION 2
#define EXCV_UNALIGNED 3
#define EXCV_ILLEGAL 4
#define SP 6
int RUN_BIT;	/* run bit */
int BUS;	/* value of the bus */

int mem_cycle = 0;
int result = 0;

typedef struct System_Latches_Struct{

int PC,		/* program counter */
    MDR,	/* memory data register */
    MAR,	/* memory address register */
    IR,		/* instruction register */
    N,		/* n condition bit */
    Z,		/* z condition bit */
    P,		/* p condition bit */
    BEN;        /* ben register */

int READY;	/* ready bit */
  /* The ready bit is also latched as you dont want the memory system to assert it 
     at a bad point in the cycle*/

int REGS[LC_3b_REGS]; /* register file. */

int MICROINSTRUCTION[CONTROL_STORE_BITS]; /* The microintruction */

int STATE_NUMBER; /* Current State Number - Provided for debugging */ 

/* For lab 4 */
int INTV; /* Interrupt vector register */
int EXCV; /* Exception vector register */
int SSP; /* Initial value of system stack pointer */
int USP;
/* MODIFY: You may add system latches that are required by your implementation */

int PSR;
int INT;
int EXC;
int SAVED_SSP;
int SAVED_USP;
int VECTOR;

} System_Latches;

/* Data Structure for Latch */

System_Latches CURRENT_LATCHES, NEXT_LATCHES;

/***************************************************************/
/* A cycle counter.                                            */
/***************************************************************/
int CYCLE_COUNT;

/***************************************************************/
/*                                                             */
/* Procedure : help                                            */
/*                                                             */
/* Purpose   : Print out a list of commands.                   */
/*                                                             */
/***************************************************************/
void help() {                                                    
    printf("----------------LC-3bSIM Help-------------------------\n");
    printf("go               -  run program to completion       \n");
    printf("run n            -  execute program for n cycles    \n");
    printf("mdump low high   -  dump memory from low to high    \n");
    printf("rdump            -  dump the register & bus values  \n");
    printf("?                -  display this help menu          \n");
    printf("quit             -  exit the program                \n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : cycle                                           */
/*                                                             */
/* Purpose   : Execute a cycle                                 */
/*                                                             */
/***************************************************************/
void cycle() {                                                

  eval_micro_sequencer();   
  cycle_memory();
  eval_bus_drivers();
  drive_bus();
  latch_datapath_values();

  CURRENT_LATCHES = NEXT_LATCHES;

  CYCLE_COUNT++;
}

/***************************************************************/
/*                                                             */
/* Procedure : run n                                           */
/*                                                             */
/* Purpose   : Simulate the LC-3b for n cycles.                 */
/*                                                             */
/***************************************************************/
void run(int num_cycles) {                                      
    int i;

    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating for %d cycles...\n\n", num_cycles);
    for (i = 0; i < num_cycles; i++) {
	if (CURRENT_LATCHES.PC == 0x0000) {
	    RUN_BIT = FALSE;
	    printf("Simulator halted\n\n");
	    break;
	}
	cycle();
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : go                                              */
/*                                                             */
/* Purpose   : Simulate the LC-3b until HALTed.                 */
/*                                                             */
/***************************************************************/
void go() {                                                     
    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating...\n\n");
    while (CURRENT_LATCHES.PC != 0x0000)
	cycle();
    RUN_BIT = FALSE;
    printf("Simulator halted\n\n");
}

/***************************************************************/ 
/*                                                             */
/* Procedure : mdump                                           */
/*                                                             */
/* Purpose   : Dump a word-aligned region of memory to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void mdump(FILE * dumpsim_file, int start, int stop) {          
    int address; /* this is a byte address */

    printf("\nMemory content [0x%0.4x..0x%0.4x] :\n", start, stop);
    printf("-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	printf("  0x%0.4x (%d) : 0x%0.2x%0.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    printf("\n");

    /* dump the memory contents into the dumpsim file */
    fprintf(dumpsim_file, "\nMemory content [0x%0.4x..0x%0.4x] :\n", start, stop);
    fprintf(dumpsim_file, "-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	fprintf(dumpsim_file, " 0x%0.4x (%d) : 0x%0.2x%0.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : rdump                                           */
/*                                                             */
/* Purpose   : Dump current register and bus values to the     */   
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void rdump(FILE * dumpsim_file) {                               
    int k; 

    printf("\nCurrent register/bus values :\n");
    printf("-------------------------------------\n");
    printf("Cycle Count  : %d\n", CYCLE_COUNT);
    printf("PC           : 0x%0.4x\n", CURRENT_LATCHES.PC);
    printf("IR           : 0x%0.4x\n", CURRENT_LATCHES.IR);
    printf("STATE_NUMBER : 0x%0.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    printf("BUS          : 0x%0.4x\n", BUS);
    printf("MDR          : 0x%0.4x\n", CURRENT_LATCHES.MDR);
    printf("MAR          : 0x%0.4x\n", CURRENT_LATCHES.MAR);
    printf("CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    printf("Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	printf("%d: 0x%0.4x\n", k, CURRENT_LATCHES.REGS[k]);
    printf("\n");

    /* dump the state information into the dumpsim file */
    fprintf(dumpsim_file, "\nCurrent register/bus values :\n");
    fprintf(dumpsim_file, "-------------------------------------\n");
    fprintf(dumpsim_file, "Cycle Count  : %d\n", CYCLE_COUNT);
    fprintf(dumpsim_file, "PC           : 0x%0.4x\n", CURRENT_LATCHES.PC);
    fprintf(dumpsim_file, "IR           : 0x%0.4x\n", CURRENT_LATCHES.IR);
    fprintf(dumpsim_file, "STATE_NUMBER : 0x%0.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    fprintf(dumpsim_file, "BUS          : 0x%0.4x\n", BUS);
    fprintf(dumpsim_file, "MDR          : 0x%0.4x\n", CURRENT_LATCHES.MDR);
    fprintf(dumpsim_file, "MAR          : 0x%0.4x\n", CURRENT_LATCHES.MAR);
    fprintf(dumpsim_file, "CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    fprintf(dumpsim_file, "Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	fprintf(dumpsim_file, "%d: 0x%0.4x\n", k, CURRENT_LATCHES.REGS[k]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : get_command                                     */
/*                                                             */
/* Purpose   : Read a command from standard input.             */  
/*                                                             */
/***************************************************************/
void get_command(FILE * dumpsim_file) {                         
    char buffer[20];
    int start, stop, cycles;

    printf("LC-3b-SIM> ");

    scanf("%s", buffer);
    printf("\n");

    switch(buffer[0]) {
    case 'G':
    case 'g':
	go();
	break;

    case 'M':
    case 'm':
	scanf("%i %i", &start, &stop);
	mdump(dumpsim_file, start, stop);
	break;

    case '?':
	help();
	break;
    case 'Q':
    case 'q':
	printf("Bye.\n");
	exit(0);

    case 'R':
    case 'r':
	if (buffer[1] == 'd' || buffer[1] == 'D')
	    rdump(dumpsim_file);
	else {
	    scanf("%d", &cycles);
	    run(cycles);
	}
	break;

    default:
	printf("Invalid Command\n");
	break;
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : init_control_store                              */
/*                                                             */
/* Purpose   : Load microprogram into control store ROM        */ 
/*                                                             */
/***************************************************************/
void init_control_store(char *ucode_filename) {                 
    FILE *ucode;
    int i, j, index;
    char line[200];

    printf("Loading Control Store from file: %s\n", ucode_filename);

    /* Open the micro-code file. */
    if ((ucode = fopen(ucode_filename, "r")) == NULL) {
	printf("Error: Can't open micro-code file %s\n", ucode_filename);
	exit(-1);
    }

    /* Read a line for each row in the control store. */
    for(i = 0; i < CONTROL_STORE_ROWS; i++) {
	if (fscanf(ucode, "%[^\n]\n", line) == EOF) {
	    printf("Error: Too few lines (%d) in micro-code file: %s\n",
		   i, ucode_filename);
	    exit(-1);
	}

	/* Put in bits one at a time. */
	index = 0;

	for (j = 0; j < CONTROL_STORE_BITS; j++) {
	    /* Needs to find enough bits in line. */
	    if (line[index] == '\0') {
		printf("Error: Too few control bits in micro-code file: %s\nLine: %d\n",
		       ucode_filename, i);
		exit(-1);
	    }
	    if (line[index] != '0' && line[index] != '1') {
		printf("Error: Unknown value in micro-code file: %s\nLine: %d, Bit: %d\n",
		       ucode_filename, i, j);
		exit(-1);
	    }

	    /* Set the bit in the Control Store. */
	    CONTROL_STORE[i][j] = (line[index] == '0') ? 0:1;
	    index++;
	}

	/* Warn about extra bits in line. */
	if (line[index] != '\0')
	    printf("Warning: Extra bit(s) in control store file %s. Line: %d\n",
		   ucode_filename, i);
    }
    printf("\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : init_memory                                     */
/*                                                             */
/* Purpose   : Zero out the memory array                       */
/*                                                             */
/***************************************************************/
void init_memory() {                                           
    int i;

    for (i=0; i < WORDS_IN_MEM; i++) {
	MEMORY[i][0] = 0;
	MEMORY[i][1] = 0;
    }
}

/**************************************************************/
/*                                                            */
/* Procedure : load_program                                   */
/*                                                            */
/* Purpose   : Load program and service routines into mem.    */
/*                                                            */
/**************************************************************/
void load_program(char *program_filename) {                   
    FILE * prog;
    int ii, word, program_base;

    /* Open program file. */
    prog = fopen(program_filename, "r");
    if (prog == NULL) {
	printf("Error: Can't open program file %s\n", program_filename);
	exit(-1);
    }

    /* Read in the program. */
    if (fscanf(prog, "%x\n", &word) != EOF)
	program_base = word >> 1;
    else {
	printf("Error: Program file is empty\n");
	exit(-1);
    }

    ii = 0;
    while (fscanf(prog, "%x\n", &word) != EOF) {
	/* Make sure it fits. */
	if (program_base + ii >= WORDS_IN_MEM) {
	    printf("Error: Program file %s is too long to fit in memory. %x\n",
		   program_filename, ii);
	    exit(-1);
	}

	/* Write the word to memory array. */
	MEMORY[program_base + ii][0] = word & 0x00FF;
	MEMORY[program_base + ii][1] = (word >> 8) & 0x00FF;
	ii++;
    }

    if (CURRENT_LATCHES.PC == 0) CURRENT_LATCHES.PC = (program_base << 1);

    printf("Read %d words from program into memory.\n\n", ii);
}

/***************************************************************/
/*                                                             */
/* Procedure : initialize                                      */
/*                                                             */
/* Purpose   : Load microprogram and machine language program  */ 
/*             and set up initial state of the machine.        */
/*                                                             */
/***************************************************************/
void initialize(char *argv[], int num_prog_files) { 
    int i;
    init_control_store(argv[1]);

    init_memory();
    for ( i = 0; i < num_prog_files; i++ ) {
	load_program(argv[i + 2]);
    }
    CURRENT_LATCHES.Z = 1;
    CURRENT_LATCHES.STATE_NUMBER = INITIAL_STATE_NUMBER;
    memcpy(CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[INITIAL_STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);
    CURRENT_LATCHES.SSP = 0x3000; /* Initial value of system stack pointer */
    CURRENT_LATCHES.USP = 0xFE00;   /*Initial Value of user stack pointer*/
    CURRENT_LATCHES.PSR = 0x8000 | 0x2;
    CURRENT_LATCHES.SAVED_SSP = CURRENT_LATCHES.SSP;
    CURRENT_LATCHES.SAVED_USP = CURRENT_LATCHES.USP;

    NEXT_LATCHES = CURRENT_LATCHES;

    RUN_BIT = TRUE;
}

/***************************************************************/
/*                                                             */
/* Procedure : main                                            */
/*                                                             */
/***************************************************************/
int main(int argc, char *argv[]) {                              
    FILE * dumpsim_file;

    /* Error Checking */
    if (argc < 3) {
	printf("Error: usage: %s <micro_code_file> <program_file_1> <program_file_2> ...\n",
	       argv[0]);
	exit(1);
    }

    printf("LC-3b Simulator\n\n");

    initialize(argv, argc - 2);

    if ( (dumpsim_file = fopen( "dumpsim", "w" )) == NULL ) {
	printf("Error: Can't open dumpsim file\n");
	exit(-1);
    }

    while (1)
	get_command(dumpsim_file);

}

/***************************************************************/
/* Do not modify the above code, except for the places indicated 
   with a "MODIFY:" comment.

   Do not modify the rdump and mdump functions.

   You are allowed to use the following global variables in your
   code. These are defined above.

   CONTROL_STORE
   MEMORY
   BUS

   CURRENT_LATCHES
   NEXT_LATCHES

   You may define your own local/global variables and functions.
   You may use the functions to get at the control bits defined
   above.

   Begin your code here 	  			       */
/***************************************************************/


void eval_micro_sequencer() {

  /* 
   * Evaluate the address of the next state according to the 
   * micro sequencer logic. Latch the next microinstruction.
   */
    int psr_15 = ((CURRENT_LATCHES.PSR >> 15) & 0x1);

    if((CURRENT_LATCHES.STATE_NUMBER == 18 || CURRENT_LATCHES.STATE_NUMBER == 19) && CURRENT_LATCHES.INT == 0) {
        NEXT_LATCHES.STATE_NUMBER = 33;
        memcpy(NEXT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[33], sizeof(int) * CONTROL_STORE_BITS);
        return;
    }

    if(GetIRD(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
        NEXT_LATCHES.STATE_NUMBER = ((CURRENT_LATCHES.IR) >> 12) & 0xF;
        memcpy(NEXT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[NEXT_LATCHES.STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);
    }
    else{
        if(CURRENT_LATCHES.EXC){
            NEXT_LATCHES.STATE_NUMBER = 46;
            memcpy(NEXT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[NEXT_LATCHES.STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);
            NEXT_LATCHES.EXC = 0;
        }else {
            //added new j bits and cond2 line
            int j_4 = ((CURRENT_LATCHES.MICROINSTRUCTION[COND2]) & (~CURRENT_LATCHES.MICROINSTRUCTION[COND1]) &
                       (~CURRENT_LATCHES.MICROINSTRUCTION[COND0])
                       & (CURRENT_LATCHES.INT));

            int j_3 = ((~CURRENT_LATCHES.MICROINSTRUCTION[COND2]) & (CURRENT_LATCHES.MICROINSTRUCTION[COND1]) &
                       (CURRENT_LATCHES.MICROINSTRUCTION[COND0])
                       & (psr_15));
            int j_2 = ((CURRENT_LATCHES.MICROINSTRUCTION[COND2]) & (CURRENT_LATCHES.MICROINSTRUCTION[COND1]) &
                       (~CURRENT_LATCHES.MICROINSTRUCTION[COND0])
                       & (CURRENT_LATCHES.BEN));
            int j_1 = ((CURRENT_LATCHES.MICROINSTRUCTION[COND2]) & (~CURRENT_LATCHES.MICROINSTRUCTION[COND1]) &
                       (CURRENT_LATCHES.MICROINSTRUCTION[COND0])
                       & (CURRENT_LATCHES.READY));
            int j_0 = ((CURRENT_LATCHES.MICROINSTRUCTION[COND2]) & (CURRENT_LATCHES.MICROINSTRUCTION[COND1]) &
                       (CURRENT_LATCHES.MICROINSTRUCTION[COND0])
                       & ((CURRENT_LATCHES.IR >> 11) & 0x1));
            int next_state =
                    GetJ(CURRENT_LATCHES.MICROINSTRUCTION) + j_0 + (j_1 << 1) + (j_2 << 2) + (j_3 << 3) + (j_4 << 4);
            memcpy(NEXT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[next_state], sizeof(int) * CONTROL_STORE_BITS);
            NEXT_LATCHES.STATE_NUMBER = next_state;

            if(j_4 == 1)
                NEXT_LATCHES.INT = 0;
        }

        if(CURRENT_LATCHES.READY == 1) {
            NEXT_LATCHES.READY = 0;
        }
    }

    if(CYCLE_COUNT == 300) {
        NEXT_LATCHES.INT = 1;
        NEXT_LATCHES.INTV = (0x1 << 1) + 0x0200;
    }
}


void cycle_memory() {
 
  /* 
   * This function emulates memory and the WE logic. 
   * Keep track of which cycle of MEMEN we are dealing with.  
   * If fourth, we need to latch Ready bit at the end of 
   * cycle to prepare microsequencer for the fifth cycle.  
   */

    if(CURRENT_LATCHES.EXC)
        return;

    if(GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
        mem_cycle++;
    }

    if(mem_cycle % 4 == 0 && mem_cycle != 0)
        NEXT_LATCHES.READY = 1;

    if(mem_cycle % 5 == 0)
        mem_cycle = 0;
}



void eval_bus_drivers() {

  /* 
   * Datapath routine emulating operations before driving the bus.
   * Evaluate the input of tristate drivers 
   *             Gate_MARMUX,
   *		 Gate_PC,
   *		 Gate_ALU,
   *		 Gate_SHF,
   *		 Gate_MDR.
   */
    if(CURRENT_LATCHES.EXC && CURRENT_LATCHES.STATE_NUMBER != 46)
        return;

    if(mem_cycle % 5 != 0)
        return;

    int dest_reg = (CURRENT_LATCHES.IR >> 9) & 0x7;
    int sourcereg1_baser = (CURRENT_LATCHES.IR >> 6) & 0x7;
    int sourcereg2 = CURRENT_LATCHES.IR & 0x7;



    if(GetGATE_MARMUX(CURRENT_LATCHES.MICROINSTRUCTION)){

        if(GetLSHF1(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
            if(GetLD_REG(CURRENT_LATCHES.MICROINSTRUCTION)){                                             //LEA
                result = ((CURRENT_LATCHES.IR & 0x1FF) << 1) + CURRENT_LATCHES.PC;
            }
            else {
                result = CURRENT_LATCHES.REGS[sourcereg1_baser] +
                         ((CURRENT_LATCHES.IR & 0x3F) << 1);       //MAR + boffset6 left shift 1 stw, ldw
                if((BUS <= 0x2FFF && BUS > 0x0000) && (CURRENT_LATCHES.PSR & 0x1) == 1 && CURRENT_LATCHES.STATE_NUMBER != 15){                                           //checking for exceptions in state 18/19
                    NEXT_LATCHES.EXC = 1;
                    NEXT_LATCHES.EXCV = (0x2 << 1) + 0x0200;
                }
                else if(BUS % 2 != 0){
                    NEXT_LATCHES.EXC = 1;
                    NEXT_LATCHES.EXCV = (0x3 << 1) + 0x0200;
                }
            }
        }
        else if(GetMARMUX(CURRENT_LATCHES.MICROINSTRUCTION) == 0){                                      //trap
            result = (CURRENT_LATCHES.IR & 0xFF) << 1;
        }
        else {
            int offset;
            if((((CURRENT_LATCHES.IR) >> 5) & 0x1) == 1){
                offset = ((CURRENT_LATCHES.IR) & 0x3F) | 0xFFFFA00;
            }
            else
                offset = (CURRENT_LATCHES.IR & 0x3F);
            result = CURRENT_LATCHES.REGS[sourcereg1_baser] + offset;    //mar + boffset6 stb, ldb
            if((BUS <= 0x2FFF && BUS > 0x0000) && (CURRENT_LATCHES.PSR & 0x1) == 1 && CURRENT_LATCHES.STATE_NUMBER != 15){                                           //checking for exceptions in state 18/19
                NEXT_LATCHES.EXC = 1;
                NEXT_LATCHES.EXCV = (0x2 << 1) + 0x0200;
            }
            else if(BUS % 2 != 0){
                NEXT_LATCHES.EXC = 1;
                NEXT_LATCHES.EXCV = (0x3 << 1) + 0x0200;
            }
        }
    }

    if(GetGATE_PC(CURRENT_LATCHES.MICROINSTRUCTION)){
        result = CURRENT_LATCHES.PC;
    }

    if(GetGATE_SHF(CURRENT_LATCHES.MICROINSTRUCTION)){                                  //SHIFT
        if(CURRENT_LATCHES.STATE_NUMBER == 13){
            int amt4 = (CURRENT_LATCHES.IR) & 0xF;
            if(((CURRENT_LATCHES.IR>> 4) & 0x1) == 0){
                result = CURRENT_LATCHES.REGS[sourcereg1_baser] << amt4;            //left
            }
            else{
                if(((CURRENT_LATCHES.IR>> 5) & 0x1) == 0){
                    result = CURRENT_LATCHES.REGS[sourcereg1_baser] >> amt4;            //right logical
                }
                else{
                    int sr15 = (CURRENT_LATCHES.REGS[sourcereg1_baser] >> 15) & 0x1;
                    if(sr15 == 0){
                        result = CURRENT_LATCHES.REGS[sourcereg1_baser] >> amt4;        //right arithmetic
                    }
                    else {
                        for(int i = 15; i > 15-amt4; i--){
                            CURRENT_LATCHES.REGS[sourcereg1_baser] >>= 1;
                            CURRENT_LATCHES.REGS[sourcereg1_baser] |= 0x8000;
                        }
                        result = CURRENT_LATCHES.REGS[sourcereg1_baser];
                    }
                }
            }

        }
    }

    if(GetGATE_ALU(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(GetALUK(CURRENT_LATCHES.MICROINSTRUCTION) == 0x1){                                              //AND
            int imm5;
            if(((CURRENT_LATCHES.IR >> 5) & 0x1) == 1){
                imm5 = CURRENT_LATCHES.IR & 0x1F;
                if(((imm5 >> 4) & 0x1) == 1){                   //checking if negative number
                    imm5 = imm5 | 0xFFFFFFE0;
                }
                result = CURRENT_LATCHES.REGS[sourcereg1_baser] & imm5;
            }else {
                result = CURRENT_LATCHES.REGS[sourcereg1_baser] & CURRENT_LATCHES.REGS[sourcereg2];
                if (((result >> 15) & 0x1) == 1) {
                    result = result | 0xFFFF0000;
                }
            }
        }

        if(GetALUK(CURRENT_LATCHES.MICROINSTRUCTION) == 0x0){                                              //ADD
            int imm5 = CURRENT_LATCHES.IR & 0x1F;
            if(((imm5 >> 4) & 0x1) == 1){                   //checking if negative number
                imm5 = imm5 | 0xFFFFFFE0;
            }
            if(((CURRENT_LATCHES.IR >> 5) & 0x1) == 1){
                result = CURRENT_LATCHES.REGS[sourcereg1_baser] + imm5;
            }else {
                result = CURRENT_LATCHES.REGS[sourcereg1_baser] + CURRENT_LATCHES.REGS[sourcereg2];
                if(((result >> 15) & 0x1) == 1){
                    result = result | 0xFFFF0000;
                }
            }

        }

        if(GetALUK(CURRENT_LATCHES.MICROINSTRUCTION) == 0x2){                                              //XOR
            if(((CURRENT_LATCHES.IR >> 5) & 0x1) == 1){
                if((CURRENT_LATCHES.IR & 0x1F) == 0x1F){
                    result = ~CURRENT_LATCHES.REGS[sourcereg1_baser];
                    if (((result >> 15) & 0x1) == 1) {
                        result = result | 0xFFFF0000;
                    }
                }else {
                    result = CURRENT_LATCHES.REGS[sourcereg1_baser] ^ (CURRENT_LATCHES.IR & 0x1F);
                    if (((result >> 15) & 0x1) == 1) {
                        result = result | 0xFFFF0000;
                    }
                }
            }else {
                result = CURRENT_LATCHES.REGS[sourcereg1_baser] ^ CURRENT_LATCHES.REGS[sourcereg2];
                if (((result >> 15) & 0x1) == 1) {
                    result = result | 0xFFFF0000;
                }
            }
        }

        if(GetALUK(CURRENT_LATCHES.MICROINSTRUCTION) == 0x3){                                        //pass A (state 23 and 24) MDR gets SR
            if(GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
                result = CURRENT_LATCHES.REGS[dest_reg];
            }
            else
                result = CURRENT_LATCHES.REGS[dest_reg] & 0x00FF;
        }
    }

    if(GetGATE_MDR(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(GetLD_REG(CURRENT_LATCHES.MICROINSTRUCTION) == 1){                               //DR gets MDR and DR gets MDR & 0x00ff
            if(GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
                result = CURRENT_LATCHES.MDR;
            }                                                                                    //puts MDR on bus, but one byte size the rest full
            else
                result = CURRENT_LATCHES.MDR & 0x00FF;
        }
        else{
            result = CURRENT_LATCHES.MDR;
        }
    }

    //lab 4 new additions, firstly checks gate sp. points are sp <- sp+2 sp<-sp-2 and puts sp on bus to update saved ssp and usp
    if(GetGATE_SP(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(GetSPMUXEXTRA(CURRENT_LATCHES.MICROINSTRUCTION) == 1)
            result = CURRENT_LATCHES.REGS[SP];
        else {
            if (GetSPMUX(CURRENT_LATCHES.MICROINSTRUCTION) == 2) {
                result = CURRENT_LATCHES.REGS[SP] + 2;
            } else if (GetSPMUX(CURRENT_LATCHES.MICROINSTRUCTION) == 1)
                result = CURRENT_LATCHES.REGS[SP] - 2;
            else if (GetSPMUX(CURRENT_LATCHES.MICROINSTRUCTION) == 0)
                result = CURRENT_LATCHES.SAVED_USP;
            else if (GetSPMUX(CURRENT_LATCHES.MICROINSTRUCTION) == 3)
                result = CURRENT_LATCHES.SAVED_SSP;
        }


    }

    if(GetGATE_DECPC(CURRENT_LATCHES.MICROINSTRUCTION)){
        result = CURRENT_LATCHES.PC - 2;
    }

    if(GetGATE_PSR(CURRENT_LATCHES.MICROINSTRUCTION)){
        result = CURRENT_LATCHES.PSR;
    }

    if(GetGATE_VECTOR(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(CURRENT_LATCHES.INT == 1)
            result = CURRENT_LATCHES.INTV;
        else if(CURRENT_LATCHES.EXC == 1)
            result = CURRENT_LATCHES.EXCV;
    }


}



void drive_bus() {

  /* 
   * Datapath routine for driving the bus from one of the 5 possible 
   * tristate drivers. 
   */
    if(CURRENT_LATCHES.EXC)
        return;

    BUS = 0;

    if(mem_cycle % 5 != 0)
        return;


    if(GetGATE_MDR(CURRENT_LATCHES.MICROINSTRUCTION)) {
        BUS = Low16bits(result);
    }

    if(GetGATE_MARMUX(CURRENT_LATCHES.MICROINSTRUCTION)){
        BUS = Low16bits(result);
    }

    if(GetGATE_ALU(CURRENT_LATCHES.MICROINSTRUCTION)){
        BUS = Low16bits(result);
    }

    if(GetGATE_SHF(CURRENT_LATCHES.MICROINSTRUCTION)){
        BUS = Low16bits(result);
    }

    if(GetGATE_PC(CURRENT_LATCHES.MICROINSTRUCTION))
        BUS = Low16bits(result);

    if(GetGATE_PSR(CURRENT_LATCHES.MICROINSTRUCTION))
        BUS = Low16bits(result);

    if(GetGATE_DECPC(CURRENT_LATCHES.MICROINSTRUCTION))
        BUS = Low16bits(result);

    if(GetGATE_VECTOR(CURRENT_LATCHES.MICROINSTRUCTION))
        BUS = Low16bits(result);

    if(GetGATE_SP(CURRENT_LATCHES.MICROINSTRUCTION))
        BUS = Low16bits(result);
}


void latch_datapath_values() {

  /* 
   * Datapath routine for computing all functions that need to latch
   * values in the data path at the end of this cycle.  Some values
   * require sourcing the bus; therefore, this routine has to come 
   * after drive_bus.
   */
    int dest_reg = (CURRENT_LATCHES.IR >> 9) & 0x7;
    int sourcereg_1 = (CURRENT_LATCHES.IR >> 6) & 0x7;
    short neg_number_bus;
    int privilege = ((CURRENT_LATCHES.PSR) >> 15) & 0x1;

    if(CURRENT_LATCHES.EXC)
        return;

    if(mem_cycle % 5 != 0)
        return;


    if(CURRENT_LATCHES.STATE_NUMBER == 10 || CURRENT_LATCHES.STATE_NUMBER == 11) {                  //illegal opcode exception
        NEXT_LATCHES.EXC = 1;
        NEXT_LATCHES.EXCV = (0x4 << 1) + 0x0200;
    }

    if((CURRENT_LATCHES.STATE_NUMBER == 18 || CURRENT_LATCHES.STATE_NUMBER == 19)){
        if((BUS <= 0x2FFF && BUS >= 0x0000) && privilege == 1){                                           //checking for exceptions in state 18/19
            NEXT_LATCHES.EXC = 1;
            NEXT_LATCHES.EXCV = (0x2 << 1) + 0x0200;
        }
        else if(BUS % 2 != 0){
            NEXT_LATCHES.EXC = 1;
            NEXT_LATCHES.EXCV = (0x3 << 1) + 0x0200;
        }
    }

    if((CURRENT_LATCHES.STATE_NUMBER == 2 || CURRENT_LATCHES.STATE_NUMBER == 6 || CURRENT_LATCHES.STATE_NUMBER == 7 || CURRENT_LATCHES.STATE_NUMBER == 3) && privilege == 1){
        if((BUS <= 0x2FFF && BUS >= 0x0000)){                                           //exceptions unaligned and prote
            NEXT_LATCHES.EXC = 1;
            NEXT_LATCHES.EXCV = (0x2 << 1) + 0x0200;
        }
        else if(CURRENT_LATCHES.STATE_NUMBER == 6 || CURRENT_LATCHES.STATE_NUMBER == 7) {
            if (BUS % 2 != 0) {
                NEXT_LATCHES.EXC = 1;
                NEXT_LATCHES.EXCV = (0x3 << 1) + 0x0200;
            }
        }
    }
    if(CURRENT_LATCHES.STATE_NUMBER == 37 || CURRENT_LATCHES.STATE_NUMBER == 51){
        if (BUS % 2 != 0) {
            NEXT_LATCHES.EXC = 1;
            NEXT_LATCHES.EXCV = (0x3 << 1) + 0x0200;
        }
        else if(BUS <= 0xFDFF && BUS >= 0x3000){
            NEXT_LATCHES.EXC = 1;
            NEXT_LATCHES.EXCV = (0x2 << 1) + 0x0200;
        }
    }
    //checks for updated stack pointer values
    if(GetGATE_SP(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(GetLD_MAR(CURRENT_LATCHES.MICROINSTRUCTION))
            NEXT_LATCHES.MAR = BUS;

        if(GetLD_REG(CURRENT_LATCHES.MICROINSTRUCTION))
            NEXT_LATCHES.REGS[SP] = BUS;
        else if(GetLD_SAVED_SSP(CURRENT_LATCHES.MICROINSTRUCTION))
           NEXT_LATCHES.SAVED_SSP = CURRENT_LATCHES.REGS[SP];
        else if(GetLD_USP_SP(CURRENT_LATCHES.MICROINSTRUCTION))
            NEXT_LATCHES.SAVED_USP = CURRENT_LATCHES.REGS[SP];
    }

    if(GetGATE_VECTOR(CURRENT_LATCHES.MICROINSTRUCTION)){
        NEXT_LATCHES.MAR = CURRENT_LATCHES.VECTOR;
    }

    //load vector register with intv or excv
    if(GetLD_VECTOR(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(GetVECTORMUX(CURRENT_LATCHES.MICROINSTRUCTION) == 0) {
            NEXT_LATCHES.VECTOR = CURRENT_LATCHES.INTV;
        }
        else if(GetVECTORMUX(CURRENT_LATCHES.MICROINSTRUCTION)) {
            NEXT_LATCHES.VECTOR = CURRENT_LATCHES.EXCV;
        }
    }

    if(GetLD_MDR(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(GetGATE_DECPC(CURRENT_LATCHES.MICROINSTRUCTION))            //40 mdr gets pc-2 lab 4
            NEXT_LATCHES.MDR = BUS;
        if(GetGATE_PSR(CURRENT_LATCHES.MICROINSTRUCTION)) {            //49 mdr gets psr set psr[15] to 0
            NEXT_LATCHES.MDR = BUS;
            NEXT_LATCHES.PSR = (CURRENT_LATCHES.PSR & 0x0007);
        }
    }

    if(GetLD_PC(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(GetLD_MAR(CURRENT_LATCHES.MICROINSTRUCTION)){                        //1819
            NEXT_LATCHES.MAR = BUS;
            NEXT_LATCHES.PC = CURRENT_LATCHES.PC + 2;
        }

    }


    //lab3


    if(GetGATE_MARMUX(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(GetLSHF1(CURRENT_LATCHES.MICROINSTRUCTION) == 1){
            if(GetLD_REG(CURRENT_LATCHES.MICROINSTRUCTION)){                                             //LEA
                NEXT_LATCHES.REGS[dest_reg] = Low16bits(BUS);
            }
            else
                NEXT_LATCHES.MAR = BUS;                                                         //MAR + boffset6 left shift 1 stw, ldw



        }

        else if(GetMARMUX(CURRENT_LATCHES.MICROINSTRUCTION) == 0){                                      //trap
            NEXT_LATCHES.MAR = BUS;
        }
        else
            NEXT_LATCHES.MAR = BUS;                                                             //mar + boffset6 stb, ldb



    }

    if(GetGATE_MDR(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(GetLD_REG(CURRENT_LATCHES.MICROINSTRUCTION) == 1) {                               //DR gets MDR and DR gets MDR & 0x00ff
            if (GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION) == 1) {
                NEXT_LATCHES.REGS[dest_reg] = Low16bits(BUS);
            }                                                                                    //puts MDR on bus, but one byte size the rest full
            else {
                if((CURRENT_LATCHES.MAR & 0x1) == 0) {
                    if (((((CURRENT_LATCHES.MDR) & 0x00FF) >> 7) & 0x1) == 1) {
                        NEXT_LATCHES.REGS[dest_reg] = Low16bits(
                                (CURRENT_LATCHES.MDR & 0x00FF) | 0xFFFFFF00);      //31 and 27
                    } else {
                        NEXT_LATCHES.REGS[dest_reg] = (CURRENT_LATCHES.MDR & 0x00FF);

                    }
                }
                else {
                    int num = (CURRENT_LATCHES.MDR & 0xFF00) >> 8;
                    if ((num >> 7) & 0x1) {
                        NEXT_LATCHES.REGS[dest_reg] = Low16bits(num | 0xFFFFFF00);
                    } else
                        NEXT_LATCHES.REGS[dest_reg] = num;
                }
            }

            if (((result >> 15) & 0x1) == 1) {
                result = result | 0xFFFF0000;
            }
            setCC(result);
        }

        else if(GetLD_PSR(CURRENT_LATCHES.MICROINSTRUCTION)){                                   //psr gets mdr lab4
            NEXT_LATCHES.PSR = BUS;
            int nzp = (CURRENT_LATCHES.N << 2) + (CURRENT_LATCHES.Z << 1) + CURRENT_LATCHES.P;
            NEXT_LATCHES.PSR += nzp;
        }
        else if(GetLD_IR(CURRENT_LATCHES.MICROINSTRUCTION)){                                                     //35
            NEXT_LATCHES.IR = BUS;
        }
        else{
            NEXT_LATCHES.PC = Low16bits(BUS);                                                           //30
        }
    }

    if(GetGATE_ALU(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(GetALUK(CURRENT_LATCHES.MICROINSTRUCTION) == 0x0 || GetALUK(CURRENT_LATCHES.MICROINSTRUCTION) == 0x1 ||
           GetALUK(CURRENT_LATCHES.MICROINSTRUCTION) == 0x2){                       //add/and/xor
            NEXT_LATCHES.REGS[dest_reg] = Low16bits(BUS);
            if(((BUS >> 15) & 0x1) == 1)
                setCC(result);
            else
                setCC(BUS);
        }
        else if(GetALUK(CURRENT_LATCHES.MICROINSTRUCTION) == 0x3){              //pass a, mdr gets sr sr[7:0]
            if(GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION) == 1)
                NEXT_LATCHES.MDR = Low16bits(BUS);
            else
                NEXT_LATCHES.MDR = BUS & 0x00FF;
        }

    }

    if(GetGATE_SHF(CURRENT_LATCHES.MICROINSTRUCTION)){
        NEXT_LATCHES.REGS[dest_reg] = Low16bits(BUS);                                       //shift
        setCC(result);
    }



    if(GetIRD(CURRENT_LATCHES.MICROINSTRUCTION)){                                                       //32
        int ir_11 = ((CURRENT_LATCHES.IR >> 11) & 0x1);
        int ir_10 = ((CURRENT_LATCHES.IR >> 10) & 0x1);
        int ir_9 = ((CURRENT_LATCHES.IR>>9) & 0x1);
        NEXT_LATCHES.BEN = ((ir_11 & CURRENT_LATCHES.N) || (ir_10 & CURRENT_LATCHES.Z) || (ir_9 & CURRENT_LATCHES.P));
    }


    if(GetLD_MDR(CURRENT_LATCHES.MICROINSTRUCTION) && GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION)){
        if(GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION)){
            if(GetLD_REG(CURRENT_LATCHES.MICROINSTRUCTION)){                                //28
                NEXT_LATCHES.REGS[0x7] = Low16bits(BUS);
                NEXT_LATCHES.MDR = (MEMORY[CURRENT_LATCHES.MAR>>1][1] << 8) + MEMORY[CURRENT_LATCHES.MAR>>1][0];            //33 and 25
            }
            else
                NEXT_LATCHES.MDR = (MEMORY[CURRENT_LATCHES.MAR>>1][1] << 8) + MEMORY[CURRENT_LATCHES.MAR>>1][0];            //33 and 25
        }
        else {
            NEXT_LATCHES.MDR = (MEMORY[CURRENT_LATCHES.MAR >> 1][1] << 8) +
                               ((MEMORY[CURRENT_LATCHES.MAR >> 1][0]) & 0xFFFF);          //29

        }

    }

    if(GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION) && GetR_W(CURRENT_LATCHES.MICROINSTRUCTION)) {               //write stb and stw
        if (GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION)) {
            MEMORY[CURRENT_LATCHES.MAR>>1][1] = (CURRENT_LATCHES.MDR & 0xFF00) >> 8;
            MEMORY[CURRENT_LATCHES.MAR>>1][0] = CURRENT_LATCHES.MDR & 0x00FF;
        } else {
            if ((CURRENT_LATCHES.MAR & 0x1) == 1)
                MEMORY[CURRENT_LATCHES.MAR>>1][1] = ((CURRENT_LATCHES.MDR) & 0xFF00) >> 8;
            else
                MEMORY[CURRENT_LATCHES.MAR>>1][0] = CURRENT_LATCHES.MDR & 0x00FF;
        }

    }

    //JSR
    if(GetPCMUX(CURRENT_LATCHES.MICROINSTRUCTION) == 0x2 && GetDRMUX(CURRENT_LATCHES.MICROINSTRUCTION)){                //state 20 and 21
        if(GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION)) {
            NEXT_LATCHES.PC = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR >> 6) & 0x7];
        }else{

            if(((CURRENT_LATCHES.IR >> 10) & 0x1) == 1){
                NEXT_LATCHES.PC = CURRENT_LATCHES.PC + (((CURRENT_LATCHES.IR & 0x7FF) | 0xFFFFF800) << 1);
            }
            else {
                NEXT_LATCHES.PC = CURRENT_LATCHES.PC + ((CURRENT_LATCHES.IR & 0x7FF) << 1);
            }
        }

        NEXT_LATCHES.REGS[0x7] = Low16bits(CURRENT_LATCHES.PC);
    }


    //JMP
    if(CURRENT_LATCHES.STATE_NUMBER == 12){
        int location = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR >> 6) & 0x7];
        if(location >= 0x0000 && location <= 0x2FFF){
            NEXT_LATCHES.EXC = 1;
            NEXT_LATCHES.EXCV = (0x2 << 1) + 0x0200;
        }
        else{
            if (BUS % 2 != 0) {
                NEXT_LATCHES.EXC = 1;
                NEXT_LATCHES.EXCV = (0x3 << 1) + 0x0200;
            }
        }
        NEXT_LATCHES.PC = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR >> 6) & 0x7];

    }

    //BEN
    if(CURRENT_LATCHES.STATE_NUMBER == 0){
        if(CURRENT_LATCHES.BEN) {
            if (((CURRENT_LATCHES.IR >> 8) & 0x1) == 1) {
                NEXT_LATCHES.PC = CURRENT_LATCHES.PC + (((CURRENT_LATCHES.IR & 0x1FF)<<1) | 0xFFFFFE00);
            } else {
                NEXT_LATCHES.PC = CURRENT_LATCHES.PC + ((CURRENT_LATCHES.IR & 0x1FF) << 1);
            }
        }
    }
}

void setCC(int number) {
    NEXT_LATCHES.PSR &= 0x8000;
    if (number < 0) {
        NEXT_LATCHES.N = 1;
        NEXT_LATCHES.P = 0;
        NEXT_LATCHES.Z = 0;
    }
    if (number == 0) {
        NEXT_LATCHES.Z = 1;
        NEXT_LATCHES.P = 0;
        NEXT_LATCHES.N = 0;
    }
    if (number > 0) {
        NEXT_LATCHES.P = 1;
        NEXT_LATCHES.Z = 0;
        NEXT_LATCHES.N = 0;
    }
    int bits = (NEXT_LATCHES.N << 2) + (NEXT_LATCHES.Z << 1) + NEXT_LATCHES.P;
    NEXT_LATCHES.PSR += bits;
}
