module registerfile(
	input wire clk,
	input wire[4:0] reg1addr,
	input wire[4:0] reg2addr,
	input wire[4:0] writeRegister,
	input wire[31:0] writeData,
	input wire regWrite,
	output reg[31:0] reg1content,
	output reg[31:0] reg2content
	);

	reg update;								//5 bits to go copy the bits 0 to 31
	reg[31:0] registers[0:31];			//Registers

	initial begin
		registers[0] = 0;
		//OPTIONAL TESTING REGISTERS
		registers[8] = 2;		//$t0
		registers[9] = 4;		//$t1
		registers[10] = 10;	//$t2
		registers[11] = 5;	//$t3
		registers[12] = 77;	//$t4

		registers[17] = 10;	//$s1
	end

	///////////OUTPUT ASSIGNMENT////////////
	always @ (reg1addr) begin
			reg1content <= registers[reg1addr];
	end

	always @ (reg2addr, update) begin
		reg2content <= registers[reg2addr];
		update <= 0;
	end

	////////////DATA WRITE//////////////////
	always@(clk) begin
		if(regWrite == 1) begin
			registers[writeRegister] = writeData;
			update = 1;
		end
	end
endmodule
