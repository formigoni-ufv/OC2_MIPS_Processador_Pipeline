module datamemory(
	input wire clk,
	input wire[31:0] addr,
	input wire[31:0] writeData,
	input wire memRead,
	input wire memWrite,
	output reg[31:0] readData
);

	reg[31:0] memory[0:63];

	always @ (addr, writeData, memRead)begin
		if(memRead) readData <= memory[addr];
	end

	always @ (clk) begin
		if(memWrite) memory[addr] <= writeData;
	end

endmodule

// module testbench ();
// 	reg[31:0] addr = 32'b100, writeData = 32'b1010;
// 	reg memRead, memWrite, clk = 0;
// 	wire[31:0] readData;
//
// 	datamemory dm(
// 		.clk(clk),
// 		.addr(addr),
// 		.writeData(writeData),
// 		.memRead(memRead),
// 		.memWrite(memWrite),
// 		.readData(readData)
// 	);
//
// 	initial begin
// 		#5 memWrite = 1;
// 		#5 clk = 1;
// 		#5	memWrite = 0;
// 		#5	memRead = 1;
// 		#5	memRead = 0;
// 	end
//
// 	always @ (readData) begin
// 		$monitor("Out: %b\n", readData);
// 	end
// endmodule //
