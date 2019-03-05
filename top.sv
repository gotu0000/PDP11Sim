import parameters::*;
module top ();

string filename;
int symbol;
logic [15:0] PC;
int fd;

int i =0;
logic clock;

test t1();
pdp_isa ins (.clock(clock));


initial begin
	clock = 1'b0;
	forever #10 clock = ~clock;
end

initial begin
	if ($value$plusargs("FILE=%s", filename) == 1);
	begin
		fd = $fopen(filename,"r");
		if (fd == 0) begin
			$display("Simulation stopped. \n File not found!");
			$stop;
		end
		while(!$feof(fd)) begin
			
			//if($fscanf(fd, "%s" , symbol) == 1)
			symbol = $fgetc(fd);
			begin
				if (symbol == "*")
				begin
					$display("* detected");
					PC = $fscanf(fd, "%o ", ins.PC1);
					//$display("PC= %d %o %h %b 	PC1= %b %o",PC,PC,PC,PC, PC1, PC1);
					// PC = $fscanf(fd, "%d ", PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %b ",PC,PC,PC,PC, PC1);
					// PC = $fscanf(fd, "%d ", PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %b ",PC,PC,PC,PC, PC1);
					// PC = $fscanf(fd, "%d ", PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %b ",PC,PC,PC,PC, PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %b ",PC,PC,PC,PC, PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %b ",PC,PC,PC,PC, PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %b ",PC,PC,PC,PC, PC1);
				end
				else if (symbol == "@")
				begin
					$display("@ detected");
					PC = $fscanf(fd, "%o ", PC);
					//$display("PC= %d %o %h %b 	PC1= %d %o %h %b ",PC,PC,PC,PC, PC1, PC1, PC1, PC1);
					;
					// PC = $fscanf(fd, "%d ", PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %d %o %h %b ",PC,PC,PC,PC, PC1, PC1, PC1, PC1);
					// PC = $fscanf(fd, "%d ", PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %d %o %h %b ",PC,PC,PC,PC, PC1, PC1, PC1, PC1);
					// PC = $fscanf(fd, "%d ", PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %d %o %h %b ",PC,PC,PC,PC, PC1, PC1, PC1, PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %d %o %h %b ",PC,PC,PC,PC, PC1, PC1, PC1, PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %d %o %h %b ",PC,PC,PC,PC, PC1, PC1, PC1, PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %d %o %h %b ",PC,PC,PC,PC, PC1, PC1, PC1, PC1);
				end
				else if (symbol == "-")
				begin
					$display("- detected");
					
					PC = $fscanf(fd, "%o ", {memory.flash[i], memory.flash[i+1]});
					$display("memory[%d] = %16o ",i,{memory.flash[i],memory.flash[i+1]});
					i = i+2;
					//$display("PC= %d %o %h %b 	PC1= %d %o %h %b ",PC,PC,PC,PC, PC1, PC1, PC1, PC1);
					// PC = $fscanf(fd, "%d ", PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %d %o %h %b ",PC,PC,PC,PC, PC1, PC1, PC1, PC1);
					// PC = $fscanf(fd, "%d ", PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %d %o %h %b ",PC,PC,PC,PC, PC1, PC1, PC1, PC1);
					// PC = $fscanf(fd, "%d ", PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %d %o %h %b ",PC,PC,PC,PC, PC1, PC1, PC1, PC1);					//PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %d %o %h %b ",PC,PC,PC,PC, PC1, PC1, PC1, PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %b ",PC,PC,PC,PC, PC1);
					// //PC = $fgetc(fd);
					// $display("PC= %d %o %h %b 	PC1= %b ",PC,PC,PC,PC, PC1);
				end
				else 
					$display("Symbol not found");
			end
		end
	end
end

//initial
//begin
//end
endmodule // top