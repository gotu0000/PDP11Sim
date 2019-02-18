import parameters::*;

module pdp_isa (
	input clock
);

logic [15:0] src_buffer, dest_buffer, src_dest_buffer;		//src and dest buffers
logic [7:0] imm_offset;						//immediate offset buffer
logic [15:0] branch_address;
logic [15:0] alu_buffer;					//temporary stores alu operations

//buffers for pipelining
struct {
	logic [15:0] instruction [4:1];
	opcode_t_0 opcode_d_1 [4:1];
	opcode_t_1 opcode_d_2 [4:1];
	opcode_t_2 opcode_s [4:1];
	opcode_t_3 opcode_b [4:1];
	logic [15:0] source [4:1];
	logic [15:0] destination [4:1];
	logic [15:0] alu_out [4:1];
	logic [15:0] branch_addr;
	instruction_type_t instruction_type [4:1];
}buffer;


//
state_t p_state, n_state = S0;

//instruction fetch
always_ff @ (posedge clock)
begin
//	if (halt == FALSE && branch_taken == FALSE)
//	begin
	if(p_state == S0)
	begin
		//next_pc <= next_pc + 1;
		cpu_register.program_counter <= cpu_register.program_counter + 1'b1;		//updated program counter
		instruction.instruction_x <= memory.flash[cpu_register.program_counter];
		buffer.instruction[1] <= memory.flash[cpu_register.program_counter];
		//if_buffer <= program_counter;
		//instruction.instruction_d <= program_counter;
	end
	else begin
		cpu_register.program_counter <= cpu_register.program_counter;
		instruction.instruction_x <= instruction.instruction_x;
		buffer.instruction[1] <= buffer.instruction[1];
	end
	//end
//	else if (branch_taken == TRUE)
//		program_counter <= branch_buffer;			//address of branch to be taken;
end

//instruction decode
always_ff @ (posedge clock)
begin
	buffer.instruction[2] <= buffer.instruction[1];
	//determine type of instruction
	if (p_state == S1)
	begin
		if (instruction.instruction_x[14:11] == HARD_CODED_SINGLE_OPERAND_BITS) begin						//implies single operand instruction
			//$display("single instruction : %b",buffer.instruction[1]);
			buffer.instruction_type[2] <= SINGLE_OPERAND; end
		else if (instruction.instruction_x[14:11] == HARD_CODED_DOUBLE_OPERAND_BITS)	begin				//implies conditional branch instruction
			//$display("cb instruction : %b",buffer.instruction[1]);
			buffer.instruction_type[2] <= CONDITIONAL_BRANCH; end
		else if (instruction.instruction_x[15:12] == MULTIPLY_INSTRUCTIONS) begin
			//$display("mul instruction : %b",buffer.instruction[1]);
			buffer.instruction_type[2] <= DOUBLE_OPERAND_2;					end									//double operand type 2
		else begin
			//$display("double instruction : %b",buffer.instruction[1]);
			buffer.instruction_type[2] <= DOUBLE_OPERAND_1; end


		//determine the oprands
		if (buffer.instruction_type[2] == SINGLE_OPERAND)
		begin
			//call function to calculate the operand
			buffer.destination[2] <= operand_get(instruction.instruction_s.mode_dest,instruction.instruction_s.dest);
		end
		else if (buffer.instruction_type[2] == DOUBLE_OPERAND_1)
		begin
			buffer.source[2]  <= operand_get (instruction.instruction_d_1.mode_src,instruction.instruction_d_1.src);
			buffer.destination [2] <= operand_get (instruction.instruction_d_1.mode_dest,instruction.instruction_d_1.dest);
		end
		else if (buffer.instruction_type[2] == DOUBLE_OPERAND_2)
		begin
			buffer.source[2] <= operand_get(instruction.instruction_d_2.mode,instruction.instruction_d_2.src_dest);
			buffer.destination[2] <= cpu_register.register[instruction.instruction_d_2.reg_op];
		end
		else 						//for conditional branch instructions
		begin
			imm_offset = instruction.instruction_c.offset;			//calculate branch address
		end
	end
	//save the opcode
	else begin
		buffer.instruction_type[2] <= buffer.instruction_type[2];
		buffer.source[2] <= buffer.source[2];
		buffer.destination[2] <= buffer.destination[2];
	end
end

//instruction execute
always_ff @ (posedge clock)
begin
	buffer.instruction_type[3] <= buffer.instruction_type[2];
	buffer.source[3] <= buffer.source[2];
	buffer.destination[3] <= buffer.destination[2];
	if(p_state == S3)
	begin
		if (buffer.instruction_type[2] == SINGLE_OPERAND)
		begin
			case (instruction.instruction_s.opcode)
				SWAB:
				buffer.alu_out[3] <= 'b10101010;
				// JSR:
				// EMT:
				// CLR:
				// CLRB:
				// COM:
				// COMB:
				// INC:
				// INCB:
				// DEC:
				// DECB:
				// NEG:
				// NEGB:
				// ADC:
				// ADCB:
				// SBC:
				// SBCB:
				// TST:
				// TSTB:
				// ROR:
				// RORB:
				// ROL:
				// ROLB:
				// ASR:
				// ASRB:
				// ASL:
				// ASLB:
				// MARK:
				// MTPS:
				// MFPI:
				// MFPD:
				// MTPI:
				// MTPD:
				// SXT:
				// MFPS:
				default: buffer.alu_out[3] <= 16'b0;
			endcase // instruction.instruction_s.operand
		end
		else if (buffer.instruction_type[2] == DOUBLE_OPERAND_2)
		begin
			//case (instruction.instruction_d.mode_src)							//mode_src is actually opcode (consider union renaming convention)
					//MUL: 
					buffer.alu_out[3] <= 'b0;
					// DIV:
					// ASH:
					// ASHC:
					// XOR:
					// SOB:
				//endcase // instruction.instruction_d.mode_src)						//mode_src is actually opcode (consider union renaming convention
			end
		else if (buffer.instruction_type[2] == DOUBLE_OPERAND_1)
			begin
				//case (instruction.instruction_d.opcode)							//other ALU instructions
				//	MOV: 
				buffer.alu_out[3] <= 'b0;
					// MOVB:
					// CMP:
					// CMPBL
					// BIT:
					// BITB:
					// BIC:
					// BICB:
					// BIS:
					// BISB:
					// ADD:
					// SUB:
				//endcase
			end
		
		else
		begin 														//conditional branch operations
			buffer.branch_addr[3] <= imm_offset + cpu_register.program_counter;
			//case (instruction.instruction_c.opcode)
		end	
	end
		else begin
			buffer.alu_out[3] <= buffer.alu_out[3];
	end
end

//memory and branch operations
//always_comb

//memory write back
always_ff @ (posedge clock)
begin
	buffer.instruction_type[4] <= buffer.instruction_type[3];
	//buffer.source[4] <= buffer.source[3];
	//buffer.destination[4] <= buffer.destination[3];
	//buffer.alu_out[4] <= buffer.alu_out[3];	
	if(p_state == S3)
	begin
		if (buffer.instruction_type[3] == SINGLE_OPERAND && instruction.instruction_s.mode_dest != 1)			//that is not a register address instruction
			memory.data[buffer.destination[3]] <= buffer.alu_out[3]; //ALU result
		else if (buffer.instruction_type[3] == DOUBLE_OPERAND_1 && instruction.instruction_d_1.mode_dest != 1)
			memory.data[buffer.destination[3]] <= buffer.alu_out[3]; //ALU result
	end
	else
		memory.data[buffer.destination[3]] <= memory.data[buffer.destination[3]];
end


//FSM implementation
always_ff @(posedge clock) 
begin
	p_state <= n_state;
end

always_ff @ (posedge clock)
begin
	case (p_state)
		S0: n_state <= S1;
		S1: n_state <= S2;
		S2: n_state <= S3;
		S3: n_state <= S0;
		default : n_state <= S0;
	endcase // p_state
end

endmodule