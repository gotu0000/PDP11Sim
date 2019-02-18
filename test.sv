import parameters::*;
module test();

logic clock;
pdp_isa ins (.clock(clock));

parameter R1 = 3'b000;

// initial begin
// 	//load the instructions into flash
// 	$readmemh("instructions_file.ascii",memory.flash);
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
initial begin
memory.flash[0] = 'b0001100011000000;
memory.flash[1] = 'b0111100011000001;
memory.flash[2] = 'b0101100011000010;
memory.flash[3] = 'b0000100011000011;
memory.flash[4] = 'b0111100011000100;
memory.flash[5] = 'b0000100011000101;
memory.flash[6] = 'b0000100011000110;
memory.flash[7] = 'b0000100011000111;
memory.flash[8] = 'b0111100011001000;
memory.flash[9] = 'b0000100011001001;
memory.flash[10] = 'b0000100011001010;
#1000
$stop;
end

initial begin
	clock = 1'b0;
	forever #10 clock = ~clock;
end

initial
begin

end
always @ (posedge clock)
begin
	//$display($time,"	state : %s",ins.p_state);
	case (ins.buffer.instruction_type[2])
		SINGLE_OPERAND:
		$display("%s %s",instruction.instruction_s.opcode, instruction.instruction_s.dest);
		DOUBLE_OPERAND_1:
		$display("%s %s %s",instruction.instruction_d_1.opcode,instruction.instruction_d_1.src, instruction.instruction_d_1.dest);
		DOUBLE_OPERAND_2:
		$display("%s %s %s",instruction.instruction_d_2.opcode,instruction.instruction_d_2.reg_op, instruction.instruction_d_2.src_dest);
		CONDITIONAL_BRANCH:
		$display("%s %s",instruction.instruction_c.opcode,instruction.instruction_c.offset);
	endcase
end

endmodule