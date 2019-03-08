#to run make file make full INPUT_FILE="<nameOfFileWOE>"
#ASSEMBLER_PATH = /u/druvi
#MACRO_PATH = $(ASSEMBLER_PATH)/macro11
#OBJ_ASCII_PATH = $(ASSEMBLER_PATH)/obj2ascii
ASCII_FILE_PATH = test.ascii

full:
	#$(MACRO_PATH) $(INPUT_FILE).mac -o $(INPUT_FILE).obj -l $(INPUT_FILE).lst
	#$(OBJ_ASCII_PATH) $(INPUT_FILE).obj $(ASCII_FILE_PATH)

	vlib work
	vlog ./*.sv
	vsim -c top +FILE=$(ASCII_FILE_PATH) -do "run -all; quit"
compile:
	vlib work
	vlog ./*.sv
run:
	vsim -c top +FILE=$(ASCII_FILE_PATH) -do "run -all; quit"
clean:
	rm -rf work  transcript vsim.wlf