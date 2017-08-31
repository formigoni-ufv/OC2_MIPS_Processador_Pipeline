 module hazard_detection_unit(
	input wire clk,
	input wire ID_EX_MemRead,
	input wire[4:0] ID_EX_RegisterRt,
	input wire[4:0] IF_ID_RegisterRs,
	input wire[4:0] IF_ID_RegisterRt,
	output reg PCWrite,
	output reg IF_ID_Write,
	output reg MUX_ID_EX_Write
	);

	always@(clk, ID_EX_MemRead, ID_EX_RegisterRt, IF_ID_RegisterRs, IF_ID_RegisterRt) begin
		if(ID_EX_MemRead & (IF_ID_RegisterRt == ID_EX_RegisterRt  || IF_ID_RegisterRs == ID_EX_RegisterRt) ) begin
			PCWrite = 0;
			IF_ID_Write = 0;
			MUX_ID_EX_Write = 1;
		end else begin
			PCWrite = 1;
			IF_ID_Write = 1;
			MUX_ID_EX_Write = 0;
		end
	end
endmodule

// module testbench();
// 	reg ID_EX_MemRead;
// 	reg[31:0] ID_EX_RegisterRt;
// 	reg[31:0] IF_ID_RegisterRs;
// 	reg[31:0] IF_ID_RegisterRt;
// 	wire PCWrite;
// 	wire IF_ID_Write;
// 	wire MUX_ID_EX_Write;
//
// 	hazard_detection_unit hazardsUnit(.ID_EX_MemRead(ID_EX_MemRead), .ID_EX_RegisterRt(ID_EX_RegisterRt), .IF_ID_RegisterRs(IF_ID_RegisterRs), .IF_ID_RegisterRt(IF_ID_RegisterRt), .PCWrite(PCWrite), .IF_ID_Write(IF_ID_Write), .MUX_ID_EX_Write(MUX_ID_EX_Write));
//
// 	initial begin
// 		#50
// 		ID_EX_MemRead <= 1;
// 		ID_EX_RegisterRt <= 10;
// 		IF_ID_RegisterRs <= 8;
// 		IF_ID_RegisterRt <= 10;
// 		#50
// 		$display("PCWrite: %b | IF_ID_Write: %b | MUX_ID_EX_Write: %b\n", PCWrite, IF_ID_Write, MUX_ID_EX_Write);
//
// 	end
//
// endmodule
