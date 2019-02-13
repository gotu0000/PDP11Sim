//	parameters.sv - defenition of various parameters for PDP11
//	
//	Version:		1.0	
//	Author:			Dhruvi Bhadiadra, Jagir Charla
//	Last Modified:	13-Feb-19
//	
//	Revision History
//	----------------
//	13-Feb-19		first implementation of parameters
//
//	Description:
//	------------
//	has structure for memory and registers of PDP-11
//	and different instructions type


package parameters;

`include "pdp_11_const.vh"

//struct to store memory element of our processor
struct {
	//FIXME should 15 come from const ? 
	logic [15:0] flash 	[`FLASH_MEMORY_SIZE - 1 : 0];	//flash memory for 16-bit instructions
	logic [15:0] data 	[`DATA_MEMORY_SIZE  - 1 : 0];	//data memory
}memory;

//status word register
typedef struct packed 
{
	logic [7:0] reserved; 
	logic [2:0] interrupt; 
	logic  		trap; 
	logic  		neg_value; 
	logic  		zero_flag; 
	logic  		overflow_flag; 
	logic  		carry_bit; 
}processor_status_word_t;

//CPU registers
struct {
	//TODO can we have register[7:0]
	//and make stack_pointer register[6] point to same thing?
	//and make program_counter register[7] point to same thing?
	logic [15:0] register[5:0];						//general purpose register (R0-R5)
	logic [15:0] stack_pointer;						//R6 = stack pointer
	logic [15:0] program_counter;					//R7 = program counter
	processor_status_word_t processor_status_word;	//I = Interrupt, T = Trap, N = negative value, Z = Zero, V = overflow, C = carry
	logic [15:0] fp_status_reg;						//floating point status register
}cpu_register;

//all 4 bit opcodes (1st bit(MSB) for byte/Word rest of 3 bits for operation)
//takes care of word as well as byte Instructions
//double operand instructions
//General and Logical
typedef enum logic [3:0] {
	MOV = 4'o01 	//Move : dest = src
	,MOVB = 4'o11	//
	,CMP  = 4'o02	//Compare : compute src - dest, set flag only
	,CMPB = 4'o12	//
	,BIT  = 4'o03 	//Bit test : compute dest & src, set flag only
	,BITB = 4'o13 	//
	,BIC  = 4'o04 	//Bit clear : dest &= ~src
	,BICB  = 4'o14 	//
	,BIS  = 4'o05 	//Bit set : dest |= src
	,BISB  = 4'o15 	//
	,ADD  = 4'o06 	//Add : dest += src
	,SUB  = 4'o16 	//Subtract : dest -= src
}opcode_t_0;

//all 7 bit opcodes
//double operand instructions which require register source operand
typedef enum logic [6:0] {
	MUL = 7'o070 	//Multiply : (R,R+1) = R * src, result becomes 32 bit 16 bit R, 16 in R+1
	,DIV = 7'o071	//Divide : (R,R+1)/src = R and (R,R+1)%src = (R+1)
	,ASH = 7'o072	//Arithmetic shift : R <<= src
	,ASHC = 7'o073	//Arithmetic shift combined : (R,R+1)<<=src 
	,XOR = 7'o074	//EXOR : dest ^= R 
	,FPO = 7'o075	//floating point operation : Reserved 
	,SI = 7'o076	//system instruction : Reserved 
	,SOB = 7'o077	//Subtract 1 and branch, decrement register if result non zero then branch backward 
}opcode_t_1;

//all 10 bit,single operand instructions
//9th(MSB) bit byte/word
//8:5 hard coded
//4:0 actual opcode
typedef enum logic [9:0] {
						SWAB = 10'o0003 	//Swap bytes, rotate 8 bits
						,JSR = 10'o004x 	//STICKEY, check whether x is correct, Jump to subroutine 
						,EMT = 10'o104x 	//STICKEY, check whether x is correct, Emulator Trap
						,CLR = 10'o0050		//Clear : dest = 0
						,CLRB = 10'o1050	//
						,COM = 10'o0051		//Complement : dest =~ dest
						,COMB = 10'o1051	//
						,INC = 10'o0052		//Increment : dest += 1
						,INCB = 10'o1052 	//
						,DEC = 10'o0053 	//Decrement : dest -= 1
						,DECB = 10'o1053	//
						,NEG = 10'o0054		//Negate : dest = ~dest
						,NEGB = 10'o1054	//
						,ADC = 10'o0055		//Add carry : dest += C(from carry flag)
						,ADCB = 10'o1055	//
						,SBC = 10'o0056		//Subtract carry : dest += C(from carry flag)
						,SBCB = 10'o1056	//
						,TST = 10'o0057		//
						,TSTB = 10'o1057	//
						,ROR = 10'o0060		//Rotate right 1 bit
						,RORB = 10'o1060	//
						,ROL = 10'o0061		//Rotate left 1 bit
						,ROLB = 10'o1061	//
						,ASR = 10'o0062		//Shift right : dest >> 1
						,ASRB = 10'o1062	//
						,ASL = 10'o0063		//Shift left : dest << 1
						,ASLB = 10'o1063	//
						,MARK = 10'o0064	//
						,MTPS = 10'o1064	//
						,MFPI = 10'o0065	//
						,MFPD = 10'o1065	//
						,MTPI = 10'o0066	//
						,MTPD = 10'o1066	//
						,SXT = 10'o0067		//
						,MFPS = 10'o1067	//

}opcode_t_2;

//all 10 bit, brnaching opcodes
typedef enum logic [9:0] {
	BR = 10'o0004 	//Branch always
	,BNE = 10'o0010	//Branch if not equal
}opcode_t_3;

//structure for double operand instructions type 1
typedef struct packed {
	opcode_t_0 	opcode;		//byte or word type + opcode
	logic [2:0] mode_src;	//src addr mode
	logic [2:0] src;		//src register
	logic [2:0] mode_dest;	//dest addr mode
	logic [2:0] dest;		//dest register
}instruction_d_1_t;

//structure for double operand instructions type 2
typedef struct packed {
	opcode_t_1 	opcode;		//hard coded bits + opcode
	logic [2:0] register;	//register
	logic [2:0] mode;		//mode
	logic [2:0] src_dest;	//src/destination
}instruction_d_2_t;

//structure for single operand instructions
typedef struct packed {
	opcode_t_2 	opcode;		//byte or word type + hard coded bits + opcode
	logic [2:0] mode_dest;	//dest addr mode
	logic [2:0] dest;		//dest register
}instruction_s_t;

//structure for conditional branch instructions
typedef struct packed {
	opcode_t_3 	opcode;	//hard coded bits + opcode
	logic [5:0] offset;	//operand
}instruction_c_t;

union packed {
	instruction_d_1_t 	instruction_d_1;	//double operand instruction type 1
	instruction_d_2_t 	instruction_d_2;	//double operand instruction type 2
	instruction_s_t 	instruction_s;		//single operand instruction
	instruction_c_t 	instruction_c;		//conditional branch instructions
}instruction;

bool halt;									//halts the pipeline when branch instruction occurs
bool branch_taken;							//indicates if branch is taken

//instruction type
typedef enum logic [1:0] {
	DOUBLE_OPERAND_1
	,DOUBLE_OPERAND_2
	,SINGLE_OPERAND
	,CONDITIONAL_BRANCH
}instruction_type;

endpackage