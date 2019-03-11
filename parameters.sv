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

parameter HARD_CODED_SINGLE_OPERAND_BITS = 4'b0001;
parameter HARD_CODED_COND_BRANCHES_BITS = 4'b0000;
parameter MULTIPLY_INSTRUCTIONS = 4'b0111;

//struct to store memory element of our processor
struct {
	//FIXME should 15 come from const ? 
	logic [7:0] flash 	[`FLASH_MEMORY_SIZE - 1 : 0];	//flash memory for 16-bit instructions
}memory;												//32 K words memory i.e. 64 KB

integer fd_trace;

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
	logic [15:0] program_counter = 16'b0;			//R7 = program counter
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
	,BEQ = 10'o0014
	,BGE = 10'o0020
	,BLT = 10'o0024
	,BGT = 10'o0030
	,BLE = 10'o0034
	,BPL = 10'o1000
	,BMI = 10'o1004
	,BHI = 10'o1010
	,BLOS = 10'o1014
	,BVC = 10'o1020
	,BVS = 10'o1024
	,BCC = 10'o1030
	,BCS = 10'o1034
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
	logic [2:0] reg_op;	//register
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
	logic [15:0]		instruction_x;
}instruction;

typedef enum {TRUE, FALSE} bool_t;
bool_t halt;									//halts the pipeline when branch instruction occurs


//instruction type
typedef enum logic [1:0] {
	DOUBLE_OPERAND_1
	,DOUBLE_OPERAND_2
	,SINGLE_OPERAND
	,CONDITIONAL_BRANCH
}instruction_type_t;

//this function gets the operand for ALU opeartion
//one bit for pc relative
//one bit for PC+2,PC+4,if not pc relative
//one bit for reg/memory, useful while writing back
function automatic logic [34:0] single_operand_get(logic [2:0] reg_mode, 
						logic [2:0] reg_number, logic acc_type, logic [15:0] temp_program_counter);
	logic [34:0] ret_val = 0;
	logic [15:0] x_op = 0;
	logic [15:0] addr_of_addr = 0;
	$display("SIO MODE=%d",reg_mode);
	$display("SIO REG_NUM=%d",reg_number);
	$display("SIO ACC_TYPE=%d",acc_type);
	
	if(reg_number < 3'b110)
	begin
		ret_val[34] = 1'b0;
		unique case (reg_mode)	//decode addressing mode
			3'b000:
			begin
				//PC+2
				ret_val[33] = 1'b0;
				//reg access
				ret_val[32:16] = 0;
				//R contains operand
				ret_val[15:0] = cpu_register.register[reg_number];
			end

			3'b001:
			begin
				//PC+2
				ret_val[33] = 1'b0;
				//address , will be used at time of execution
				//mem access
				ret_val[32:16] = {1'b1,cpu_register.register[reg_number]};				
				//mem[R] contains operand
				ret_val[15:8] = memory.flash[cpu_register.register[reg_number]];
				ret_val[7:0] = memory.flash[cpu_register.register[reg_number]+16'd1]; 
				trace_file_write (0,cpu_register.register[reg_number]);
			end

			3'b010:
			begin
				//PC+2
				ret_val[33] = 1'b0;
				if(acc_type == 1'b1)
				begin
					cpu_register.register[reg_number] = cpu_register.register[reg_number] + 16'd1;
					ret_val[32:16] = {1'b1,(cpu_register.register[reg_number]-16'd1)};
					ret_val[15:8] = 8'b0;
					ret_val[7:0] = memory.flash[(cpu_register.register[reg_number]-16'd1)];
					trace_file_write (0,cpu_register.register[reg_number]-16'd1);
				end
				else
				begin
					cpu_register.register[reg_number] = cpu_register.register[reg_number] + 16'd2;
					ret_val[32:16] = {1'b1,(cpu_register.register[reg_number]-16'd2)};
					ret_val[15:8] = memory.flash[cpu_register.register[reg_number]-16'd2];
					ret_val[7:0] = memory.flash[cpu_register.register[reg_number]-16'd1];
					trace_file_write (0,ret_val[31:16]);
				end
			end

			3'b011:
			begin
				//PC+2
				ret_val[33] = 1'b0;
				cpu_register.register[reg_number] = cpu_register.register[reg_number]+16'd2;

				ret_val[32:24] = {1'b1,memory.flash[cpu_register.register[reg_number]-16'd2]};
				ret_val[23:16] = memory.flash[cpu_register.register[reg_number]-16'd1];
				addr_of_addr = ret_val[31:16];

				ret_val[15:8] = memory.flash[addr_of_addr];
				ret_val[7:0] = memory.flash[addr_of_addr+16'd1];
				trace_file_write(0,addr_of_addr);
			end

			3'b100:
			begin
				ret_val[33] = 1'b0;
				//if byte
				if(acc_type == 1'b1)
				begin
					cpu_register.register[reg_number] = cpu_register.register[reg_number]-16'd1;
					ret_val[32:16] = {1'b1,(cpu_register.register[reg_number])};
					ret_val[15:8] = 8'b0;
					ret_val[7:0] = memory.flash[cpu_register.register[reg_number]];
					trace_file_write(0,ret_val[31:16]);
				end
				else
				begin
					cpu_register.register[reg_number] = cpu_register.register[reg_number]-16'd2;
					ret_val[32:16] = {1'b1,(cpu_register.register[reg_number])};
					ret_val[15:8] = memory.flash[cpu_register.register[reg_number]];
					ret_val[7:0] = memory.flash[cpu_register.register[reg_number]+16'd1];
					trace_file_write(0,ret_val[31:16]);
				end
			end

			3'b101:
			begin
				ret_val[33] = 1'b0;
				cpu_register.register[reg_number] = cpu_register.register[reg_number]-16'd2;
				ret_val[32:24] = {1'b1,(memory.flash[cpu_register.register[reg_number]])};
				ret_val[23:16] = (memory.flash[cpu_register.register[reg_number]+16'd1]);
				addr_of_addr = ret_val[31:16];
				ret_val[15:8] = memory.flash[addr_of_addr];
				ret_val[7:0] = memory.flash[addr_of_addr+16'd1];
				trace_file_write(0,ret_val[31:16]);
			end

			3'b110:
			begin
				//PC + 4
				ret_val[33] = 1'b1;
				x_op = {memory.flash[temp_program_counter+16'd2]
										,memory.flash[temp_program_counter+16'd3]};
				trace_file_write(0,temp_program_counter+16'd2);
				ret_val[32:16] = {1'b1,(cpu_register.register[reg_number]+x_op)};
				addr_of_addr = ret_val[31:16];
				ret_val[15:8] = memory.flash[addr_of_addr];
				ret_val[7:0] = memory.flash[addr_of_addr+16'd1];
				trace_file_write(0,ret_val[31:16]);
			end

			3'b111:
			begin
				//PC + 4
				ret_val[33] = 1'b1;
				x_op = {memory.flash[temp_program_counter+16'd2],memory.flash[temp_program_counter+16'd3]};
				
				trace_file_write(0,temp_program_counter+16'd2);
				ret_val[32:24] = {1'b1,memory.flash[cpu_register.register[reg_number]+
										x_op]};
				ret_val[23:16] = memory.flash[cpu_register.register[reg_number]+
										x_op+16'd1];

				addr_of_addr = ret_val[31:16];

				ret_val[15:8] = memory.flash[addr_of_addr];
				ret_val[7:0] = memory.flash[addr_of_addr+16'd1];
				trace_file_write(0,ret_val[31:16]);
			end
			
			default:
			begin
				$display("SOMETHING IS WRONG=%d,%d",reg_mode,reg_number);
				ret_val = 0;
			end
		endcase 
	end
	
	//for PC relative
	else if(reg_number == 3'b111)
	begin
		$display("PC RELATIVE=%d",reg_mode);
		ret_val[34] = 1'b1;
		unique case (reg_mode)	//decode addressing mode

			3'b010:
			begin
				//PC+4
				ret_val[33] = 1'b1;
				ret_val[32:16] = {1'b1,(temp_program_counter+16'd2)};
				ret_val[15:0] = {memory.flash[temp_program_counter+16'd2]
								,memory.flash[temp_program_counter+16'd3]};
				trace_file_write(0,ret_val[31:16]);
			end

			3'b011:
			begin
				//PC+4
				ret_val[33] = 1'b1;
				ret_val[32:16] = {1'b1,memory.flash[temp_program_counter+16'd2],
									memory.flash[temp_program_counter+16'd3]};

				addr_of_addr = ret_val[31:16];

				ret_val[15:8] = memory.flash[addr_of_addr];

				ret_val[7:0] = memory.flash[addr_of_addr+16'd1];
				trace_file_write(0,ret_val[31:16]);
			end

			3'b110:
			begin
				//PC + 4
				ret_val[33] = 1'b1;
				ret_val[32:16] = {1'b1,(temp_program_counter+16'd4+
										{memory.flash[temp_program_counter+16'd2]
										,memory.flash[temp_program_counter+16'd3]
											})};
				trace_file_write(0,temp_program_counter+16'd2);
				addr_of_addr = ret_val[31:16];

				ret_val[15:8] = memory.flash[addr_of_addr];

				ret_val[7:0] = memory.flash[addr_of_addr+16'd1];
				trace_file_write(0,ret_val[31:16]);
				// $display("X VAL=%o",({memory.flash[temp_program_counter+16'd2]
				// 		,memory.flash[temp_program_counter+16'd3]}+
				// 		16'd4+
				// 		temp_program_counter));

										
				// $display("RET VAL=%o_%o_%o_%o_%o_%o PC=%d",ret_val[34],ret_val[33],ret_val[32]
				// 							,ret_val[31:16]
				// 							,ret_val[15:8]
				// 							,ret_val[7:0]
				// 							,temp_program_counter);
			end

			3'b111:
			begin
				//PC + 4
				ret_val[33] = 1'b1;

				ret_val[32:24] = {1'b1,memory.flash[(temp_program_counter+16'd4+
										{memory.flash[temp_program_counter+16'd2]
										,memory.flash[temp_program_counter+16'd3]
											})]};
				trace_file_write(0,temp_program_counter+16'd2);
				ret_val[23:16] = {1'b1,memory.flash[(temp_program_counter+16'd4+
										{memory.flash[temp_program_counter+16'd2]
										,memory.flash[temp_program_counter+16'd3]
											})+16'd1]};

				trace_file_write(0,(temp_program_counter+16'd4+
										{memory.flash[temp_program_counter+16'd2]
										,memory.flash[temp_program_counter+16'd3]
											}));
				addr_of_addr = ret_val[31:16];

				ret_val[15:8] = memory.flash[addr_of_addr];

				ret_val[7:0] = memory.flash[addr_of_addr+16'd1];
				trace_file_write(0,addr_of_addr);
			end
			default:
			begin
				$display("SOMETHING IS WRONG=%d,%d",reg_mode,reg_number);
				ret_val = 0;
			end
		endcase
	end
	return ret_val;
endfunction : single_operand_get

//2bits PC increment
//1 bit mem or reg
//16 bit for dest address
//16 bit for destination operand
//16 bit for source operand
function automatic logic [50:0] double_operand_get(logic [2:0] s_reg_mode, logic [2:0] s_reg_number,logic [2:0] d_reg_mode, logic [2:0] d_reg_number, logic acc_type);
	logic [50:0] ret_val = 0;
	/*

	logic [15:0] x_op_s = 0;
	logic [15:0] addr_s = 0;
	logic [15:0] addr_of_addr_s = 0;
	logic [15:0] x_op_d = 0;
	logic [15:0] addr_d = 0;
	logic [15:0] addr_of_addr_d = 0;

	//source and destination are PC
	if((s_reg_number == 3'b111) && (d_reg_number == 3'b111))
	begin
		//add by 6
		ret_val[50:49] = 2'b10;
		ret_val[48] = 1'b1;		
		unique case (s_reg_mode)	//decode addressing mode

			3'b010:
			begin
				x_op_s = {memory.flash[cpu_register.program_counter+16'd2]
										,memory.flash[cpu_register.program_counter+16'd3]
											};
				ret_val[15:0] = x_op_s;
			end

			3'b011:
			begin
				x_op_s = {memory.flash[cpu_register.program_counter+16'd2]
										,memory.flash[cpu_register.program_counter+16'd3]
											};
				addr_s = x_op_s;
				ret_val[15:8] = memory.flash[addr_s];

				ret_val[7:0] = memory.flash[addr_s+16'd1];
			end

			3'b110:
			begin
				x_op_s = {memory.flash[cpu_register.program_counter+16'd2]
										,memory.flash[cpu_register.program_counter+16'd3]
											};
				addr_s = cpu_register.program_counter+16'd4+x_op_s;
				ret_val[15:8] = memory.flash[addr_s];

				ret_val[7:0] = memory.flash[addr_s+16'd1];
			end

			3'b111:
			begin
				x_op_s = {memory.flash[cpu_register.program_counter+16'd2]
										,memory.flash[cpu_register.program_counter+16'd3]
											};
				addr_s = cpu_register.program_counter+16'd4+x_op_s;

				addr_of_addr_s = {memory.flash[addr_s],memory.flash[addr_s+16'd1]};

				ret_val[15:8] = memory.flash[addr_of_addr_s];

				ret_val[7:0] = memory.flash[addr_of_addr_s+16'd1];
			end

			default:
			begin
				$display("SOMETHING IS WRONG=%d,%d",s_reg_mode,s_reg_number);
				ret_val = 0;
			end
		endcase

		unique case (d_reg_mode)	//decode addressing mode

			3'b010:
			begin
				x_op_d = {memory.flash[cpu_register.program_counter+16'd3]
										,memory.flash[cpu_register.program_counter+16'd4]
											};
				ret_val[47:32] = cpu_register.program_counter+16'd3;
				ret_val[31:16] = x_op_d;
			end

			3'b011:
			begin
				x_op_d = {memory.flash[cpu_register.program_counter+16'd3]
										,memory.flash[cpu_register.program_counter+16'd4]
											};
				addr_d = x_op_d;
				ret_val[47:32] = addr_d;
				ret_val[31:24] = memory.flash[addr_d];

				ret_val[23:16] = memory.flash[addr_d+16'd1];
			end

			3'b110:
			begin
				x_op_d = {memory.flash[cpu_register.program_counter+16'd3]
										,memory.flash[cpu_register.program_counter+16'd4]
											};
				addr_d = cpu_register.program_counter+16'd6+x_op_d;
				ret_val[47:32] = addr_d;
				ret_val[31:24] = memory.flash[addr_d];

				ret_val[23:16] = memory.flash[addr_d+16'd1];
			end

			3'b111:
			begin
				x_op_d = {memory.flash[cpu_register.program_counter+16'd3]
										,memory.flash[cpu_register.program_counter+16'd4]
											};
				addr_d = cpu_register.program_counter+16'd6+x_op_d;

				addr_of_addr_d = {memory.flash[addr_d],memory.flash[addr_d+16'd1]};

				ret_val[47:32] = addr_of_addr_d;
				ret_val[31:24] = memory.flash[addr_of_addr_d];

				ret_val[23:16] = memory.flash[addr_of_addr_d+16'd1];
			end

			default:
			begin
				$display("SOMETHING IS WRONG=%d,%d",d_reg_mode,d_reg_number);
				ret_val = 0;
			end
		endcase
	end
	else if((s_reg_number == 3'b111) && (d_reg_number < 3'b110))
	begin
		unique case (s_reg_mode)	//decode addressing mode

			3'b010:
			begin
				x_op_s = {memory.flash[cpu_register.program_counter+16'd2]
										,memory.flash[cpu_register.program_counter+16'd3]
											};
				ret_val[15:0] = x_op_s;
			end

			3'b011:
			begin
				x_op_s = {memory.flash[cpu_register.program_counter+16'd2]
										,memory.flash[cpu_register.program_counter+16'd3]
											};
				addr_s = x_op_s;
				ret_val[15:8] = memory.flash[addr_s];

				ret_val[7:0] = memory.flash[addr_s+16'd1];
			end

			3'b110:
			begin
				x_op_s = {memory.flash[cpu_register.program_counter+16'd2]
										,memory.flash[cpu_register.program_counter+16'd3]
											};
				addr_s = cpu_register.program_counter+16'd4+x_op_s;
				ret_val[15:8] = memory.flash[addr_s];

				ret_val[7:0] = memory.flash[addr_s+16'd1];
			end

			3'b111:
			begin
				x_op_s = {memory.flash[cpu_register.program_counter+16'd2]
										,memory.flash[cpu_register.program_counter+16'd3]
											};
				addr_s = cpu_register.program_counter+16'd4+x_op_s;

				addr_of_addr_s = {memory.flash[addr_s],memory.flash[addr_s+16'd1]};

				ret_val[15:8] = memory.flash[addr_of_addr_s];

				ret_val[7:0] = memory.flash[addr_of_addr_s+16'd1];
			end

			default:
			begin
				$display("SOMETHING IS WRONG=%d,%d",s_reg_mode,s_reg_number);
				ret_val = 0;
			end
		endcase

		unique case (d_reg_mode)	//decode addressing mode

			3'b001:
			begin

				ret_val[47:32] = cpu_register.program_counter+16'd3;
				ret_val[31:16] = cpu_register.program_counter[d_reg_number];
			end

			default:
			begin
				$display("SOMETHING IS WRONG=%d,%d",d_reg_mode,d_reg_number);
				ret_val = 0;
			end
		endcase

	end
	else if((s_reg_number < 3'b110) && (d_reg_number == 3'b111))
	begin
		//
	end
	else
	begin
		//
	end
	*/
	return ret_val;
endfunction : double_operand_get

//states for FSM
typedef enum {
		SM_RESET
		, UPDATE_INIT_PC
		, INSTRUCTION_FETCH
		, INSTRUCTION_DECODE
		, INSTRUCTION_EXECUTE
		, MEM_WRITE
		, CHECK_END_OF_CODE
		, SM_DONE
		} state_t;

//call this initially
//before execution of the program starts
function automatic void open_trace_file_to_write();
	//every time at the beggining 
	//it will flush the file
	// fd_trace = $fopen(TRACE_FILE_OUT,"w");
	fd_trace = $fopen("TraceFile.txt","w");
endfunction : open_trace_file_to_write

function automatic void trace_file_write(logic [1:0] accessType, logic [15:0] memAddr);
	$fwrite(fd_trace, "%d \t %04o\n", accessType, memAddr);
endfunction : trace_file_write

//call this at the end
//after execution of the program
function automatic void close_trace_file_to_write();
	//will release the file
	//necessary to avoid corruption
	$fclose(fd_trace);
endfunction : close_trace_file_to_write



endpackage