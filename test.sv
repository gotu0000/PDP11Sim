import parameters::*;
module test();



logic [15:0] flashh 	[1023 : 0];

/*initial begin
	//load the instructions into flash
	$readmemh("addc.ascii",flash);
end*/

// initial begin
// 	// cpu_register.register[0] = 'd5;
// 	// cpu_register.register[1] = 'd1;
// 	$monitor("memory  %b \n %b\n %b \n %b \n %b \n %b \n %b \n", flashh[0] flashh[1], flashh[2], flashh[3], flashh[4], flashh[5], flashh[6], flashh[7]);
// end

// initial begin
// //instruction.instruction_x = 'b0000100011000000;					//give the opcode of any instruction you want to test.
// //int i;
// //initisalizing all registers and data memory to 5 (because it is my lucky number), so that we dont have to initialize it everythime we git instruction
// for (int i = 1; i <= 5; i++)
// begin
// 	cpu_register.register[i] = 16'd0;
// end
// for (int j = 0; j <= 2046; j++)
// begin
// 	memory.data[j] = 16'd0;
// end
// end

//give instructions here

always @ (ins.p_state)
begin
		$display("\n\nSTATE = %p",ins.p_state);
		//$display($time,"	state : %s",ins.p_state);
		if(ins.p_state == S2)
		begin
			$strobe("Program counter = %d" , cpu_register.program_counter);
			case (ins.buffer.instruction_type[2])
				SINGLE_OPERAND:
				begin
					//$display($time,"	%s	%d	%b    %s R%d",ins.p_state, ins.buffer.next_pc, instruction.instruction_x, instruction.instruction_s.opcode, instruction.instruction_s.dest);
					$strobe($time,"	instruction = %s R%d",instruction.instruction_s.opcode, instruction.instruction_s.dest);
					$trobe("operands : %o "instruction.instruction_s.dest);
				end
				DOUBLE_OPERAND_1:
				begin
					//$display($time,"	%s	%d	%b    %s R%d, R%d",ins.p_state, ins.buffer.next_pc, instruction.instruction_x, instruction.instruction_d_1.opcode,instruction.instruction_d_1.dest, instruction.instruction_d_1.src);
					$strobe($time,"	instruction = %s R%d, R%d",instruction.instruction_d_1.opcode, instruction.instruction_d_1.src, instruction.instruction_d_1.dest);
					$trobe("operands :src= %o  dest=%o"instruction.instruction_d_1.src, instruction.instruction_d_1.dest);
				end
				DOUBLE_OPERAND_2:
				begin
					//$display($time,"	%s  %d	%b 	%s R%d, R%d",ins.p_state, ins.buffer.next_pc, instruction.instruction_x, instruction.instruction_d_2.opcode,instruction.instruction_d_2.reg_op, instruction.instruction_d_2.src_dest);
					$strobe($time,"	instruction = %s R%d,R%d",instruction.instruction_d_2.opcode,instruction.instruction_d_2.reg_op, instruction.instruction_d_2.src_dest);
					$trobe("operands :reg= %o  src_dest=%o"instruction.instruction_d_2.reg_op, instruction.instruction_d_2.src_dest);
				end
				CONDITIONAL_BRANCH:
				begin
					//$display($time,"	%s  %d	%b 	%s %d",ins.p_state, ins.buffer.next_pc, instruction.instruction_x, instruction.instruction_c.opcode,instruction.instruction_c.offset);
					$strobe($time,"	instruction = %s %o",instruction.instruction_c.opcode,instruction.instruction_c.offset);
					$strobe("operands : offset = %o "instruction.instruction_c.offset);
				end
				default:
				$strobe("No instruction found");
			endcase
		end
		else if (ins.p_state == S3)
			$strobe("ALU result = %d ", ins.buffer.alu_out[3] );
		else if (ins.p_state == S1)
			$strobe("PC = %o" , instruction.instruction_x);


	if ($isunknown(instruction.instruction_x) && cpu_register.program_counter > 0)
	begin
		$stop;
	end
end
endmodule