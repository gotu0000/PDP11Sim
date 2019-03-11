import parameters::*;

//top module for PDPD11
module top ();

//name of ascii file to read
string filename;
//file descriptor for ascii file
int fd;
//symbol to be stored in this variable
int symbol;
logic [15:0] fileLine;
logic [15:0] fileLineRet;

logic [15:0] memWrCounter = 0;
logic [15:0] memWrLast = 0;
logic [15:0] dataOffset;
logic [15:0] initPC;

logic clock;
logic reset;
logic doneEXE;

//module instantiation of pdp_11 ISA
pdp_isa ins (.clock(clock),.reset(reset),.pCStart(dataOffset),.pCEnd(memWrLast),.doneEXE(doneEXE));

initial begin
	clock = 1'b0;
	forever #10 clock = ~clock;
end

initial 
begin
	open_trace_file_to_write();
	reset = 1'b1;
	//check for the passed argument
	if ($value$plusargs("FILE=%s", filename) == 1)
	begin
		//open the file
		fd = $fopen(filename,"r");
		//if file not found
		if (fd == 0) 
		begin
			$display("Simulation stopped.\n");
			$display("File not found!\n");
			$stop;
		end
		while(!$feof(fd)) 
		begin
			symbol = $fgetc(fd);
			begin
				if (symbol == "*")
				begin
					$display("* detected");
					fileLineRet = $fscanf(fd, "%o ", fileLine);
					$display("SUCCESS=%d:%06o",fileLineRet,fileLine);
					dataOffset = fileLine;
					$display("Data Offset=%d",dataOffset);
				end
				else if (symbol == "@")
				begin
					$display("@ detected");
					fileLineRet = $fscanf(fd, "%o ", fileLine);
					$display("SUCCESS=%d:%06o",fileLineRet,fileLine);
					initPC = fileLine;
					$display("Init PC=%d",initPC);
				end
				else if (symbol == "-")
				begin
					$display("- detected");
					fileLineRet = $fscanf(fd, "%o ", {memory.flash[memWrCounter+initPC], memory.flash[memWrCounter+1+initPC]});
					fileLine = {memory.flash[memWrCounter+initPC], memory.flash[memWrCounter+1+initPC]};
					$display("SUCCESS=%d:%06o",fileLineRet,fileLine);
					/*
					//write into memory
					memory.flash[memWrCounter] = fileLine[15:8];
					memory.flash[memWrCounter+1] = fileLine[7:0];
					*/
					memWrCounter = memWrCounter + 2;
				end
				else 
				begin
					$display("Symbol not found");
				end
			end
		end
	end
	#100
	//get max value of memWrCounter 
	memWrLast = memWrCounter;
	reset = 1'b0;
	$display("Starting Simulation");
	// #100000
	// $display("Goodbye Cruel World");
	// $stop;
end


//always block will stop the simulation
always@(doneEXE)
begin
	if(doneEXE == 1'b1)
	begin
		$display("Goodbye Cruel World");
		close_trace_file_to_write();
		$stop;
	end
end


endmodule // top