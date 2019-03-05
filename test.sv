import parameters::*;
module test();



logic [15:0] flashh 	[1023 : 0];

initial begin
	//load the instructions into flash
	$readmemh("addc.ascii",flashh);
end

initial begin
	// cpu_register.register[0] = 'd5;
	// cpu_register.register[1] = 'd1;
	$monitor("memory  %b \n %b\n %b \n %b \n %b \n %b \n %b \n", flashh[0] flashh[1], flashh[2], flashh[3], flashh[4], flashh[5], flashh[6], flashh[7]);
end

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
initial begin
// memory.flash[0] = 16'o060001;
// memory.flash[1] = 16'o060000;
// memory.flash[2] = 16'o027654;
// memory.flash[3] = 16'o127654;
// memory.flash[4] = 16'o037654;
// memory.flash[5] = 16'o137654;
// memory.flash[6] = 16'o047654;
// memory.flash[7] = 16'o147654;
// memory.flash[8] = 16'o057654;
// memory.flash[9] = 16'o157654;
// memory.flash[10] = 16'o067654;
// memory.flash[11] = 16'o167654;
// memory.flash[12] = 16'o070654;
// memory.flash[13] = 16'o071654;
// memory.flash[14] = 16'o072654;
// memory.flash[15] = 16'o073654;
// memory.flash[16] = 16'o074654;
// memory.flash[0] = 'b0001100011000000;
// memory.flash[1] = 'b0111100011000001;
// memory.flash[2] = 'b0101100011000010;
// memory.flash[3] = 'b0000100011000011;
// memory.flash[4] = 'b0111100011000100;
// memory.flash[5] = 'b0000100011000101;
// memory.flash[6] = 'b0000100011000110;
// memory.flash[7] = 'b0000100011000111;
// memory.flash[8] = 'b0111100011001000;
// memory.flash[9] = 'b0000100011001001;
// memory.flash[10] = 'b0000100011001010;
#1000
$stop;
end


initial
begin

end
always @ (ins.p_state)
begin
	//$display($time,"	state : %s",ins.p_state);
	if(ins.p_state == S1)
	begin
		case (ins.buffer.instruction_type[2])
			SINGLE_OPERAND:
			//$display($time,"	%s	%d	%b    %s R%d",ins.p_state, ins.buffer.next_pc, instruction.instruction_x, instruction.instruction_s.opcode, instruction.instruction_s.dest);
			$display(,"%s R%d",instruction.instruction_s.opcode, instruction.instruction_s.dest);
			DOUBLE_OPERAND_1:
			//$display($time,"	%s	%d	%b    %s R%d, R%d",ins.p_state, ins.buffer.next_pc, instruction.instruction_x, instruction.instruction_d_1.opcode,instruction.instruction_d_1.dest, instruction.instruction_d_1.src);
			$display("%s R%d, R%d",instruction.instruction_d_1.opcode, instruction.instruction_d_1.src, instruction.instruction_d_1.dest);
			DOUBLE_OPERAND_2:
			//$display($time,"	%s  %d	%b 	%s R%d, R%d",ins.p_state, ins.buffer.next_pc, instruction.instruction_x, instruction.instruction_d_2.opcode,instruction.instruction_d_2.reg_op, instruction.instruction_d_2.src_dest);
			$display("%s R%d,R%d",instruction.instruction_d_2.opcode,instruction.instruction_d_2.reg_op, instruction.instruction_d_2.src_dest);
			CONDITIONAL_BRANCH:
			//$display($time,"	%s  %d	%b 	%s %d",ins.p_state, ins.buffer.next_pc, instruction.instruction_x, instruction.instruction_c.opcode,instruction.instruction_c.offset);
			$display("%s %o",instruction.instruction_c.opcode,instruction.instruction_c.offset);
		endcase
	end
	else if (ins.p_state == S2)
			$display("ALU result = %d ", ins.buffer.alu_out[3]);
end

endmodule