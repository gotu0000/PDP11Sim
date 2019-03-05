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
	logic [15:0] next_pc;
}buffer;

logic [15:0] PC1 = 'b0, PC2 = 'b0; //for adding PC offset and relative PC
//
state_t p_state, n_state = S1;

//combination logic block for relative PC

//instruction fetch
always_ff @ (posedge clock)
begin
//	if (halt == FALSE && branch_taken == FALSE)
//	begin
	if(p_state == S1)
	begin
		if (cpu_register.program_counter == 1'b0)
		begin
			buffer.next_pc <= cpu_register.program_counter + PC1;;
			$display("PC = %d",cpu_register.program_counter);
			cpu_register.program_counter <= cpu_register.program_counter + PC1;		//updated program counter
			instruction.instruction_x <= memory.flash[buffer.next_pc];
			buffer.instruction[1] <= memory.flash[buffer.next_pc];
		//if_buffer <= program_counter;
		//instruction.instruction_d <= program_counter;
		end
		else
		begin
			buffer.next_pc <= cpu_register.program_counter;
			$display("PC = %d",cpu_register.program_counter);
			cpu_register.program_counter <= cpu_register.program_counter + 2'd2 + PC2;		//updated program counter
			instruction.instruction_x <= {memory.flash[buffer.next_pc], memory.flash[buffer.next_pc+1'b1]};
			buffer.instruction[1] <= memory.flash[buffer.next_pc];
		end
	end
	else begin
		buffer.next_pc <= buffer.next_pc;
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
	if (p_state == S2)
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
			if (buffer.destination[2] == 'd7)
			begin
				PC2 = 1'd2;
			end
			else 
			begin
				PC2 = 1'b0;
			end
		end
		else if (buffer.instruction_type[2] == DOUBLE_OPERAND_1)
		begin
			//for add
			buffer.source[2]  <= operand_get (instruction.instruction_d_1.mode_src,instruction.instruction_d_1.src);
			buffer.destination [2] <= operand_get (instruction.instruction_d_1.mode_dest,instruction.instruction_d_1.dest);
			if (buffer.source[2] == 'd7 & buffer.destination[2] == 'd7)
			begin
				PC2 = 'd2;
			end
			if (buffer.source[2] == 'd7 & buffer.destination[2] == 'd7)
			begin
				PC2 = 'd4;
			end
			if (buffer.source[2] != 'd7 & buffer.destination[2] != 7)
			begin
				PC2 = 'b0;
			end
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
				//swap bytes
				SWAB:
				begin
					buffer.alu_out[3] <= {buffer.source[3][7:0],buffer.source[3][15:8]};
				end

				JSR: buffer.alu_out[3] <= 'b0;

				EMT: buffer.alu_out[3] <= 'b0;

				CLR: 
				begin
					buffer.alu_out[3] <= 16'b0;
					//STICKY do i need to set the zero flag
					cpu_register.processor_status_word.zero_flag <= 1'b1;
				end

				CLRB: 
				begin
					buffer.alu_out[3][7:0] <= 8'b0;
					cpu_register.processor_status_word.zero_flag <= 1'b1;
				end

				//complement
				//bit wise
				//FIXME do i need to set the zero flag
				COM: 
				begin
					buffer.alu_out[3] <= ~buffer.source[3];
					if(buffer.alu_out[3] == 16'b0)
					begin
						cpu_register.processor_status_word.zero_flag <= 1'b1;
					end
				end

				//complement, bit wise
				COMB:
				begin
					buffer.alu_out[3][7:0] <= ~buffer.source[3][7:0];
					if(buffer.alu_out[3][7:0] == 8'b0)
					begin
						cpu_register.processor_status_word.zero_flag <= 1'b1;
					end
				end

				INC: 
				begin
					{cpu_register.processor_status_word.overflow_flag
					,buffer.alu_out[3]} <= buffer.source[3] + 1'b1;
				end

				INCB: 
				begin
					{cpu_register.processor_status_word.overflow_flag
					,buffer.alu_out[3][7:0]} <= buffer.source[3][7:0] + 1'b1;
				end

				DEC: 
				begin
					buffer.alu_out[3] <= buffer.source[3] - 1'b1;
					//FIXME should i check for overflow flag
					//and the zero flag
				end

				DECB:
				begin
					buffer.alu_out[3][7:0] <= buffer.source[3][7:0] - 1'b1;
					//FIXME should i check for overflow flag
					//and the zero flag
				end

				NEG:
				begin
					buffer.alu_out[3] <= ~buffer.source[3];
				end

				NEGB:
				begin
					buffer.alu_out[3][7:0] <= ~buffer.source[3][7:0];
				end

				ADC:
				begin
					{cpu_register.processor_status_word.overflow_flag
					,buffer.alu_out[3]} <= buffer.source[3] 
					 			+ cpu_register.processor_status_word.carry_bit;
				end

				ADCB:
				begin
					{cpu_register.processor_status_word.overflow_flag
					,buffer.alu_out[3][7:0]} <= buffer.source[3][7:0] 
					 			+ cpu_register.processor_status_word.carry_bit;
				end

				SBC:
				begin
					buffer.alu_out[3] <= buffer.source[3] 
					 			- cpu_register.processor_status_word.carry_bit;
				end

				SBCB:
				begin
					buffer.alu_out[3][7:0] <= buffer.source[3][7:0] 
					 			- cpu_register.processor_status_word.carry_bit;
				end

				TST:
				begin 
					buffer.alu_out[3] <= buffer.source[3];
					if(buffer.alu_out[3] == 16'b0)
					begin
						cpu_register.processor_status_word.zero_flag <= 1'b1;
					end
				end

				TSTB:
				begin 
					buffer.alu_out[3][7:0] <= buffer.source[3][7:0];
					if(buffer.alu_out[3][7:0] == 8'b0)
					begin
						cpu_register.processor_status_word.zero_flag <= 1'b1;
					end
				end

				ROR:
				begin
					buffer.alu_out[3] <= {buffer.source[0],buffer.source[3][15:1]};
				end

				RORB:
				begin
					buffer.alu_out[3][7:0] <= {buffer.source[0],buffer.source[3][7:1]};
				end

				ROL:
				begin
					buffer.alu_out[3] <= {buffer.source[3][14:0],buffer.source[3][15]};
				end
				 
				ROLB: 
				begin
					buffer.alu_out[3] <= {buffer.source[3][6:0],buffer.source[3][7]};
				end

				ASR:
				begin
					buffer.alu_out[3] <= {1'b0,buffer.source[3][15:1]};
				end

				ASRB:
				begin
					buffer.alu_out[3][7:0] <= {1'b0,buffer.source[3][7:1]};
				end

				ASL:
				begin
					buffer.alu_out[3] <= {buffer.source[3][14:0],1'b0};
				end

				ASLB:
				begin
					buffer.alu_out[3][7:0] <= {buffer.source[3][6:0],1'b0};
				end

				MARK: buffer.alu_out[3] <= 'b0;

				MTPS: buffer.alu_out[3] <= 'b0;

				MFPI: buffer.alu_out[3] <= 'b0;

				MFPD:
				begin
					buffer.alu_out[3] <= 'b0;
				end

				MTPI:
				begin
					//FIXME increment by 1 or 2
					buffer.alu_out[3] <= cpu_register.stack_pointer + 1'b1;
				end

				MTPD:
				begin 
					//FIXME increment by 1 or 2
					buffer.alu_out[3] <= cpu_register.stack_pointer + 1'b1;
				end

				SXT: buffer.alu_out[3] <= 'b0;
				MFPS: buffer.alu_out[3] <= 'b0;

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
					ADD: buffer.alu_out[3] <= buffer.source[2] + buffer.destination[2];
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
	if(p_state == S4)
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

always_comb
begin
	case (p_state)
		S1: n_state <= S2;
		S2: n_state <= S3;
		S3: n_state <= S4;
		S4: n_state <= S1;
		default : n_state <= S1;
	endcase // p_state
end

endmodule