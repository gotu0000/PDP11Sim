import parameters::*;

module pdp_isa (
	input clock,
	input reset, 	//reset active high,asynchronous
	input [15:0] pCStart,
	input [15:0] pCEnd,
	output logic doneEXE
);

state_t p_state, n_state;
//use this for decoding
//and fetching operand
logic [2:0] reg_source_mode;
logic [2:0] reg_source;
logic [2:0] reg_dest_mode;
logic [2:0] reg_dest;
logic byte_word;
//ret value of single instruction
logic [34:0] single_inst_ret;
logic [50:0] double_inst_ret;

//source operand should be fetched in this
logic [15:0] source_operand;
logic [15:0] dest_operand;
logic pc_relative;
logic reg_mem;
logic pc_add;
//register number in which i want to store
logic [2:0] dest_operand_reg;
//address of destination in which
//we want to write
logic [15:0] dest_operand_addr;

//output of execution can be stored in this
logic [15:0] alu_out;
logic [15:0] alu_out_lsb;
//incase of branch store address over here
logic [15:0] branch_addr;
//calculate address from here
logic [5:0] branch_addr_offset;
//decide whether to take a branch or not
logic branch_taken;
instruction_type_t instruction_type;

logic carry_buffer;
int byte_op =0;

//FSM implementation
always_ff @(posedge clock,posedge reset) 
begin
	if(reset == 1'b1)
	begin
		p_state <= SM_RESET;
	end
	else
	begin
		p_state <= n_state;
	end
end

//next state logic
always_comb
begin
	case (p_state)
		SM_RESET: 				n_state = UPDATE_INIT_PC;
		UPDATE_INIT_PC: 		n_state = INSTRUCTION_FETCH;
		INSTRUCTION_FETCH: 		n_state = INSTRUCTION_DECODE;
		INSTRUCTION_DECODE: 	n_state = INSTRUCTION_EXECUTE;
		INSTRUCTION_EXECUTE: 	n_state = MEM_WRITE;
		MEM_WRITE: 				n_state = CHECK_END_OF_CODE;
		CHECK_END_OF_CODE:
		begin
			if(cpu_register.program_counter >= pCEnd)
			begin
	 			n_state = SM_DONE;
			end
			else
			begin
	 			n_state = INSTRUCTION_FETCH;
	 		end
		end
		SM_DONE: n_state = SM_DONE;
		default : n_state = SM_RESET;
	endcase // p_state
end

always_comb
begin
	case (p_state)
		SM_RESET:
		begin
			//do nothing
			doneEXE = 1'b0;
		end

		UPDATE_INIT_PC:
		begin
			// cpu_register.register[0] = 16'd0;
			// cpu_register.register[1] = 16'd0;
			// cpu_register.register[2] = 16'd0;
			// cpu_register.register[3] = 16'd0;
			// cpu_register.register[4] = 16'd0;
			// cpu_register.register[5] = 16'd0;

			cpu_register.register[0] = 16'o000014;
			cpu_register.register[1] = 16'o000002;
			cpu_register.register[2] = 16'o000004;
			cpu_register.register[3] = 16'o000012;
			cpu_register.register[4] = 16'o000010;
			cpu_register.register[5] = 16'o000020;
			cpu_register.program_counter = pCStart;
		end

		INSTRUCTION_FETCH:
		begin
			instruction.instruction_x = {memory.flash[cpu_register.program_counter]
									, memory.flash[cpu_register.program_counter + 1'b1]};
		end

		INSTRUCTION_DECODE:
		begin
			//decode the instruction
			$display("INST=%06o",instruction.instruction_x);

			//implies single operand instruction
			if (instruction.instruction_x[14:11] == HARD_CODED_SINGLE_OPERAND_BITS) 
			begin
				instruction_type = SINGLE_OPERAND;
				reg_source_mode = instruction.instruction_s.mode_dest; 
				reg_source = instruction.instruction_s.dest;
				byte_word = instruction.instruction_s[15];
				//will make changes in the register and fetch the operator
				single_inst_ret = single_operand_get(reg_source_mode,reg_source,byte_word);
				//extract values
				source_operand = single_inst_ret[15:0];
				dest_operand_addr = single_inst_ret[31:16];
				reg_mem = single_inst_ret[32];
				pc_add = single_inst_ret[33];
				pc_relative = single_inst_ret[34];
				dest_operand_reg = reg_source;
				$display("SI SOURCE OPERAND=%d",source_operand);
				$display("SI DEST ADDRESS=%d",dest_operand_addr);
				$display("SI DEST REG=%d",dest_operand_reg);
			end
			else if (instruction.instruction_x[14:11] == HARD_CODED_COND_BRANCHES_BITS)
			begin
				instruction_type = CONDITIONAL_BRANCH;
				branch_addr_offset = instruction.instruction_c.offset; 
			end
			else if (instruction.instruction_x[15:12] == MULTIPLY_INSTRUCTIONS) 
			begin
				instruction_type = DOUBLE_OPERAND_2;					
			end
			else 
			begin
				instruction_type = DOUBLE_OPERAND_1; 
			end
		end

		INSTRUCTION_EXECUTE:
		begin
			//execute the instruction
			if (instruction_type == SINGLE_OPERAND)
			begin
				case (instruction.instruction_s.opcode)
					//swap bytes
					SWAB:
					begin
						alu_out = {source_operand[7:0],source_operand[15:8]};
						if (alu_out[7] == 'b1)  
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else 
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						if (alu_out[7:0] == 'd0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						cpu_register.processor_status_word.overflow_flag = 1'b0;
						cpu_register.processor_status_word.carry_bit = 1'b0;
					end

					CLR: 
					begin
						alu_out = 16'b0;
						//STICKY do i need to set the zero flag
						cpu_register.processor_status_word.zero_flag = 1'b1;
						cpu_register.processor_status_word.neg_value	 = 1'b0;
						cpu_register.processor_status_word.overflow_flag = 1'b0;
						cpu_register.processor_status_word.carry_bit	 = 1'b0;
					end

					CLRB: 
					begin
						alu_out[7:0] = 8'b0;
						cpu_register.processor_status_word.zero_flag = 1'b1;
						cpu_register.processor_status_word.neg_value	 = 1'b0;
						cpu_register.processor_status_word.overflow_flag = 1'b0;
						cpu_register.processor_status_word.carry_bit	 = 1'b0;
					end

					//complement
					//bit wise
					//FIXME do i need to set the zero flag
					COM: 
					begin
						alu_out = ~source_operand;
						if(alu_out == 16'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						cpu_register.processor_status_word.overflow_flag = 'b0;
						cpu_register.processor_status_word.carry_bit = 'b1;
						if(alu_out[15] == 1'b1)
						begin 
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin 
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
					end

					//complement, bit wise
					COMB:
					begin
						alu_out[7:0] = ~source_operand[7:0];
						if(alu_out == 8'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						cpu_register.processor_status_word.overflow_flag = 'b0;
						cpu_register.processor_status_word.carry_bit = 'b1;
						if(alu_out[7] == 1'b1)
						begin 
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin 
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
					end

					INC: 
					begin
						{cpu_register.processor_status_word.overflow_flag
						,alu_out} = source_operand + 1'b1;
						if(alu_out == 16'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if (alu_out[15] == 1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end
					end

					INCB: 
					begin
						{cpu_register.processor_status_word.overflow_flag
						,alu_out[7:0]} = source_operand[7:0] + 1'b1;
						if(alu_out[7:0] == 8'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if (alu_out[7] == 1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end
					end

					DEC: 
					begin
						alu_out = source_operand - 1'b1;
						//FIXME should i check for overflow flag
						//and the zero flag
						if (alu_out[15] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						if(alu_out == 'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end
						
						if(alu_out == 'o100000)
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
					end

					DECB:
					begin
						alu_out[7:0] = source_operand[7:0] - 1'b1;
						//FIXME should i check for overflow flag
						//and the zero flag
						if(alu_out[7:0] == 8'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if (alu_out[7] == 1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end
					end

					NEG:
					begin
						alu_out = -source_operand ;
						if (alu_out[15] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						if(alu_out == 'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if(alu_out == 'o100000)
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end
					end

					NEGB:
					begin
						alu_out[7:0] = -source_operand[7:0];
						if (alu_out[7] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						if(alu_out == 'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if(alu_out[7] == 'b1)
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end
					end

					ADC:
					begin
						{cpu_register.processor_status_word.carry_bit
						,alu_out} = source_operand 
						 			+ cpu_register.processor_status_word.carry_bit;
						if (alu_out[15] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						if(alu_out == 'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if(alu_out == 'o100000)
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end
					end
					
					ADCB:
					begin
						{cpu_register.processor_status_word.carry_bit
						,alu_out[7:0]} = source_operand[7:0] 
						 			+ cpu_register.processor_status_word.carry_bit;
						if (alu_out[7] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end
						if(alu_out == 'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
					end

					SBC:
					begin
						{cpu_register.processor_status_word.carry_bit,
						alu_out} = source_operand 
						 			- cpu_register.processor_status_word.carry_bit;
						if (alu_out[15] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end
						if(alu_out == 'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
					end

					SBCB:
					begin
						{cpu_register.processor_status_word.carry_bit
						,alu_out[7:0]} = source_operand[7:0] 
						 			- cpu_register.processor_status_word.carry_bit;
						if (alu_out[7] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end
						if(alu_out == 'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
					end

					TST:
					begin 
						alu_out = source_operand;
						if(alu_out == 16'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if (alu_out[15] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						cpu_register.processor_status_word.overflow_flag = 1'b0;
						cpu_register.processor_status_word.carry_bit = 1'b0;
					end

					TSTB:
					begin 
						alu_out[7:0] = source_operand[7:0];
						if(alu_out[7:0] == 8'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if (alu_out[7] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						cpu_register.processor_status_word.overflow_flag = 1'b0;
						cpu_register.processor_status_word.carry_bit = 1'b0;
					end

					ROR:
					begin
						{alu_out,cpu_register.processor_status_word.carry_bit} = {cpu_register.processor_status_word.carry_bit,source_operand[15:0]};
						if(alu_out == 16'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if (alu_out[15] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						cpu_register.processor_status_word.overflow_flag = cpu_register.processor_status_word.neg_value ^ cpu_register.processor_status_word.carry_bit;
					end

					RORB:
					begin
						{alu_out[7:0],cpu_register.processor_status_word.carry_bit} = {cpu_register.processor_status_word.carry_bit,source_operand[7:0]};
						if(alu_out == 8'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if (alu_out[7] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						cpu_register.processor_status_word.overflow_flag = cpu_register.processor_status_word.neg_value ^ cpu_register.processor_status_word.carry_bit;
					end

					ROL:
					begin
						{cpu_register.processor_status_word.carry_bit,alu_out} = {source_operand[15:0],cpu_register.processor_status_word.carry_bit};
						if(alu_out == 16'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if (alu_out[15] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						cpu_register.processor_status_word.overflow_flag = cpu_register.processor_status_word.neg_value ^ cpu_register.processor_status_word.carry_bit;
					end
					 
					ROLB: 
					begin
						alu_out = {source_operand[6:0],source_operand[7]};
						if(alu_out == 8'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if (alu_out[7] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						cpu_register.processor_status_word.overflow_flag = cpu_register.processor_status_word.neg_value ^ cpu_register.processor_status_word.carry_bit;
					end

					ASR:
					begin
						{alu_out,cpu_register.processor_status_word.carry_bit} = {source_operand[15],source_operand[15:0]};
						if(alu_out == 16'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if (alu_out[15] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						cpu_register.processor_status_word.overflow_flag = cpu_register.processor_status_word.neg_value ^ cpu_register.processor_status_word.carry_bit;
					end

					ASRB:
					begin
						{alu_out[7:0],cpu_register.processor_status_word.carry_bit} = {source_operand[7],source_operand[7:0]};
						if(alu_out == 8'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if (alu_out[7] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						cpu_register.processor_status_word.overflow_flag = cpu_register.processor_status_word.neg_value ^ cpu_register.processor_status_word.carry_bit;
					end

					ASL:
					begin
						{cpu_register.processor_status_word.carry_bit,alu_out} = {source_operand[15:0],1'b0};
						if(alu_out == 16'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if (alu_out[15] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						cpu_register.processor_status_word.overflow_flag = cpu_register.processor_status_word.neg_value ^ cpu_register.processor_status_word.carry_bit;
					end

					ASLB:
					begin
						{cpu_register.processor_status_word.carry_bit,alu_out[7:0]} = {source_operand[7:0],1'b0};
						if(alu_out == 8'b0)
						begin
							cpu_register.processor_status_word.zero_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.zero_flag = 1'b0;
						end
						if (alu_out[7] == 1'b1)
						begin
							cpu_register.processor_status_word.neg_value = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.neg_value = 1'b0;
						end
						cpu_register.processor_status_word.overflow_flag = cpu_register.processor_status_word.neg_value ^ cpu_register.processor_status_word.carry_bit;
					end

					default: alu_out = 16'b0;
				endcase // instruction.instruction_s.operand
			end
			else if (instruction_type == DOUBLE_OPERAND_2)
			begin
				case (instruction.instruction_d_2.opcode)
					MUL: 
					begin
						{alu_out,alu_out_lsb} = source_operand* dest_operand;
					end
					DIV:
					begin
						alu_out = {dest_operand,cpu_register.register[instruction.instruction_d_2.reg_op+1]} / source_operand;
						alu_out_lsb =  {dest_operand,cpu_register.register[instruction.instruction_d_2.reg_op+1]} % source_operand;
					end
					ASH:
					begin
						alu_out = dest_operand <<< source_operand;
					end
					ASHC:
					begin
						{alu_out,alu_out_lsb} = {dest_operand,cpu_register.register[instruction.instruction_d_2.reg_op+1]} <<< source_operand;
					end
					XOR:
					begin
						alu_out = source_operand ^ dest_operand;
					end			
				endcase // instruction.instruction_d.mode_src)						//mode_src is actually opcode (consider union renaming convention
			end
			//General and Logical instrictions	
			//all 4 bit opcodes (1st bit(MSB) for byte/Word rest of 3 bits for operation)
			else if (instruction_type == DOUBLE_OPERAND_1)
			begin
				case (instruction.instruction_d_1.opcode)							//other ALU instructions
					MOV:
					begin
						alu_out =  source_operand ;

						//overflow flag : clear
						//Carry flag    : Not affected
						cpu_register.processor_status_word.overflow_flag = 1'b0;                          
					end
					MOVB:
					begin						
						alu_out[7:0] =  source_operand[7:0] ;
						//overflow flag : clear
						//Carry flag    : Not affected
						//Set Byte operation flag
						cpu_register.processor_status_word.overflow_flag = 1'b0;
						byte_op = 1;
					end
					CMP:
					begin
						{carry_buffer,alu_out} =  source_operand - dest_operand ;

						// Overflow Flag : set if there was arithmetic overflow; that is,
						// 					if the operands were of opposite signs and the sign of the destination was the same as the sign of the reult; cleared otherwise
						if((source_operand[15] != dest_operand[15])&&( dest_operand[15] == alu_out[15]))
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end

						// Carry flag: cleared if there was a carry from the
						//				most significant bit of the result; set otherwise (indicating borrow required)
						if (carry_buffer == 1'b1)
						begin
							cpu_register.processor_status_word.carry_bit = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.carry_bit = 1'b0;
						end
					end
					CMPB:
					begin
						{carry_buffer,alu_out[7:0]} =  source_operand[7:0] - dest_operand[7:0] ;
						//set Byte operation flag
						byte_op = 1;

						// Overflow Flag : set if there was arithmetic overflow; that is,
						// 					if the operands were of opposite signs and the sign of the destination was the same as the sign of the reult; cleared otherwise
						if((source_operand[7] != dest_operand[7])&&( dest_operand[7] == alu_out[7]))
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end

						// Carry flag: cleared if there was a carry from the
						//				most significant bit of the result; set otherwise (indicating borrow required)							
						if (carry_buffer == 1'b1)
						begin
							cpu_register.processor_status_word.carry_bit = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.carry_bit = 1'b0;
						end
					end
					BIT:
					begin
						//overflow flag : clear
						//Carry flag    : Not affected
						alu_out =  source_operand & dest_operand ;
						cpu_register.processor_status_word.overflow_flag = 1'b0;
					end
					BITB:
					begin
						alu_out[7:0] =  source_operand[7:0] & dest_operand[7:0] ;
						//overflow flag : clear
						//Carry flag    : Not affected
						//Set Byte operation flag
						cpu_register.processor_status_word.overflow_flag = 1'b0;
						byte_op = 1;
					end
					BIC:
					begin
						alu_out =  ~source_operand & dest_operand ;
						//overflow flag : clear
						//Carry flag    : Not affected
						cpu_register.processor_status_word.overflow_flag = 1'b0;
					end
					BICB:
					begin
						alu_out[7:0] =  ~source_operand[7:0] & dest_operand[7:0] ;
						//overflow flag : clear
						//Carry flag    : Not affected
						//Set Byte operation flag
						cpu_register.processor_status_word.overflow_flag = 1'b0;
						byte_op = 1;
					end
					BIS:
					begin
						alu_out = dest_operand | source_operand ; 
						//overflow flag : clear
						//Carry flag    : Not affected
						cpu_register.processor_status_word.overflow_flag = 1'b0;
					end
					BISB:
					begin
						alu_out[7:0] = dest_operand[7:0] | source_operand[7:0] ;
						//overflow flag : clear
						//Carry flag    : Not affected
						//Set Byte operation flag
						cpu_register.processor_status_word.overflow_flag = 1'b0;
						byte_op = 1;
					end
					ADD:
					begin
						{carry_buffer,alu_out} = dest_operand + source_operand ;

						// Overflow flag : set if there was arithmetic overflow as a result of the operation; that is 
						//					both operands were of the same sign and the result was of the opposite sign; cleared otherwise
						if((source_operand[15] == dest_operand[15])&&( dest_operand[15] != alu_out[15]))
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end

						// Carry flag: cleared if there was a carry from the
						//				most significant bit of the result; set otherwise
						if (carry_buffer == 1'b1)
						begin
							cpu_register.processor_status_word.carry_bit = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.carry_bit = 1'b0;
						end
					end
					SUB:
					begin
						{carry_buffer,alu_out} = dest_operand - source_operand ;

						//Overflow flag : set if there was arithmetic overflow as a result of the operation; that is, if
						//					the operands were of opposite signs and the sign of the source was the same as the sign of the result; cleared otherwise
						if((source_operand[15] != dest_operand[15])&&( source_operand[15] == alu_out[15]))
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.overflow_flag = 1'b0;
						end

						// Carry flag: cleared if there was a carry from the
						//				most significant bit of the result; set otherwise (indicating borrow required)	
						if (carry_buffer == 1'b1)
						begin
							cpu_register.processor_status_word.carry_bit = 1'b1;
						end
						else
						begin
							cpu_register.processor_status_word.carry_bit = 1'b0;
						end
					end
				endcase

				//Negative flag : set if the result is < 0; cleared otherwise
				//Zero flag : set if result = 0; cleared otherwise
				if (byte_op == 1)
				begin
					if (alu_out[7:0] == 8'b0)
					begin
						cpu_register.processor_status_word.zero_flag = 1'b1;
					end
					else
					begin
						cpu_register.processor_status_word.zero_flag = 1'b0;
					end

					if (alu_out[7] == 1'b1)
					begin
						cpu_register.processor_status_word.neg_value = 1'b1;
					end
					else
					begin
						cpu_register.processor_status_word.neg_value = 1'b0;
					end
					byte_op = 0;
				end
				else 
				begin
					if (alu_out == 16'b0)
					begin
						cpu_register.processor_status_word.zero_flag = 1'b1;
					end
					else
					begin
						cpu_register.processor_status_word.zero_flag = 1'b0;
					end

					if (alu_out[15] == 1'b1)
					begin
						cpu_register.processor_status_word.neg_value = 1'b1;
					end
					else
					begin
						cpu_register.processor_status_word.neg_value = 1'b0;
					end
				end
			end
			//conditional branch operations
			else if (instruction_type == CONDITIONAL_BRANCH)
			begin 														
				case (instruction.instruction_c.opcode)							//other ALU instructions
					//compute this from offset value
					//add code here for branching
					//TODO do this for all the instructions
					BR:
					begin
 						branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
 						//always branch
 						branch_taken = 1'b1;
					end

					BNE:
					begin
						if(cpu_register.processor_status_word.zero_flag == 1'b0)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					end
					BEQ:
					begin
						if(cpu_register.processor_status_word.zero_flag == 1'b1)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					end
					BGE:
					begin
						if((cpu_register.processor_status_word.neg_value ^ cpu_register.processor_status_word.overflow_flag) == 1'b0)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					end
					BLT:
					begin
						if((cpu_register.processor_status_word.neg_value ^ cpu_register.processor_status_word.overflow_flag) == 1'b1)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					end
					BGT:
					begin
						if((cpu_register.processor_status_word.zero_flag | (cpu_register.processor_status_word.neg_value ^ cpu_register.processor_status_word.overflow_flag)) == 1'b0)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					end
					BLE:
					begin
						if((cpu_register.processor_status_word.zero_flag | (cpu_register.processor_status_word.neg_value ^ cpu_register.processor_status_word.overflow_flag)) == 1'b1)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					end
					BPL:
					begin
						if(cpu_register.processor_status_word.neg_value == 1'b0)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					end
					BMI:
					begin
						if(cpu_register.processor_status_word.neg_value == 1'b1)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					
					end
					BHI:
					begin
						if((cpu_register.processor_status_word.carry_bit | cpu_register.processor_status_word.zero_flag) == 1'b0)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					end
					BLOS:
					begin
						if((cpu_register.processor_status_word.carry_bit | cpu_register.processor_status_word.zero_flag) == 1'b1)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					end
					BVC:
					begin
						if(cpu_register.processor_status_word.overflow_flag == 1'b0)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					
					end
					BVS:
					begin
						if(cpu_register.processor_status_word.overflow_flag == 1'b1)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					end
					BCC:
					begin
						if(cpu_register.processor_status_word.carry_bit == 1'b0)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					end
					
					BCS:
					begin
						if(cpu_register.processor_status_word.carry_bit == 1'b1)
						begin
							branch_addr = cpu_register.program_counter + (2*branch_addr_offset);
							branch_taken = 1'b1;
						end
						else
						begin
							branch_taken = 1'b0;
						end
					end
					
				endcase
			end
		end

		MEM_WRITE:
		begin
			if(instruction_type == SINGLE_OPERAND)
			begin
				$display("SI DEST RESULT=%d",alu_out);
				$display("SI DEST ADDRESS=%d",dest_operand_addr);
				$display("SI DEST REG=%d",dest_operand_reg);
				if(reg_mem == 1'b0)
				begin
					if(byte_word == 1'b1)
					begin
						cpu_register.register[dest_operand_reg][7:0] = alu_out[7:0];
					end
					else
					begin
						cpu_register.register[dest_operand_reg][15:8] = alu_out[15:8];
						cpu_register.register[dest_operand_reg][7:0] = alu_out[7:0];
					end
				end
				else
				begin
					if(byte_word == 1'b1)
					begin
						memory.flash[dest_operand_addr] = alu_out[7:0];
					end
					else
					begin
						memory.flash[dest_operand_addr] = alu_out[15:8];
						memory.flash[dest_operand_addr+16'd1] = alu_out[7:0];
					end
				end
				if(pc_add == 1'b1)
				begin
					cpu_register.program_counter = cpu_register.program_counter + 4;
					$display("SIF PC=%d",cpu_register.program_counter);
					
				end
				else
				begin
					cpu_register.program_counter = cpu_register.program_counter + 2;
					$display("SELSE PC=%d",cpu_register.program_counter);
				end
			end
			/*
			else if(instruction_type == DOUBLE_OPERAND_2)
			begin
				cpu_register.program_counter = cpu_register.program_counter + 2;
				$display("PC=%d",cpu_register.program_counter);
			end
			else if(instruction_type == DOUBLE_OPERAND_1)
			begin
				cpu_register.program_counter = cpu_register.program_counter + 2;
				$display("PC=%d",cpu_register.program_counter);
			end
			*/
			else if(instruction_type == CONDITIONAL_BRANCH)
			begin
				if(branch_taken == 1'b1)
				begin
					cpu_register.program_counter = branch_addr;
					$display("BT PC=%d",cpu_register.program_counter);
				end
				else
				begin
					cpu_register.program_counter = cpu_register.program_counter + 2;
					$display("BNT PC=%d",cpu_register.program_counter);
				end
			end
			else 
			begin
				//update the value of program counter here
				cpu_register.program_counter = cpu_register.program_counter + 2;
				$display("PC=%d",cpu_register.program_counter);
			end
		end

		CHECK_END_OF_CODE:
		begin
			//do nothing
		end

		SM_DONE:
		begin
			doneEXE = 1'b1;
		end

		default:
		begin
			//do nothing
		end
	endcase // p_state 
end

endmodule