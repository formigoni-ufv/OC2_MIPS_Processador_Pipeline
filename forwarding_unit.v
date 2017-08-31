 module forwarding_unit
	(
	input wire clk,
	input wire EX_MEM_RegWrite,
	input wire MEM_WB_RegWrite,
	input wire[4:0] ID_EX_Rs,
	input wire[4:0] ID_EX_Rt,
	input wire[4:0] EX_MEM_Rd,
	input wire[4:0] MEM_WB_Rd,
	output reg[1:0] ForwardA,
	output reg[1:0] ForwardB
	);

	always@(clk) begin

		//Avoid dont cares
		#10
		ForwardA <= 2'b0;
		ForwardB <= 2'b0;

		#10
		if((EX_MEM_RegWrite == 1) && (EX_MEM_Rd != 0)) begin
			if(EX_MEM_Rd == ID_EX_Rs) ForwardA <= 2'b10;
			if(EX_MEM_Rd == ID_EX_Rt) ForwardB <= 2'b10;
		end

		if( (MEM_WB_RegWrite == 1) && (MEM_WB_Rd != 0)) begin
			if( !((EX_MEM_RegWrite == 1) && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rs)) )begin
				if(MEM_WB_Rd == ID_EX_Rs) ForwardA <= 2'b01;
			end

			if( !((EX_MEM_RegWrite == 1) && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rt)) )begin
				if(MEM_WB_Rd == ID_EX_Rt) ForwardB <= 2'b01;
			end
		end

	end
endmodule

// module testbench();
// 	reg clk;
// 	reg EX_MEM_RegWrite;
// 	reg MEM_WB_RegWrite;
// 	reg[31:0] ID_EX_Rs;
// 	reg[31:0] ID_EX_Rt;
// 	reg[31:0] EX_MEM_Rd;
// 	reg[31:0] MEM_WB_Rd;
// 	wire[1:0] forwardA;
// 	wire[1:0] forwardB;
//
// 	forwarding_unit ForwardingUnit(.clk(clk), .EX_MEM_RegWrite(EX_MEM_RegWrite), .MEM_WB_RegWrite(MEM_WB_RegWrite), .ID_EX_Rs(ID_EX_Rs), .ID_EX_Rt(ID_EX_Rt), .EX_MEM_Rd(EX_MEM_Rd), .MEM_WB_Rd(MEM_WB_Rd), .ForwardA(forwardA), .ForwardB(forwardB));
//
// 	initial begin
// 		#50
// 		clk <= 1;
// 		EX_MEM_RegWrite <= 0;
// 		MEM_WB_RegWrite <= 1;
// 		ID_EX_Rs <= 10;
// 		ID_EX_Rt <= 10;
// 		EX_MEM_Rd <= 10;
// 		MEM_WB_Rd <= 10;
// 		#50
// 		$display("Forward A: %b | Forward B: %b\n", forwardA, forwardB);
// 	end
//
// endmodule
