module multiplexorRegDst(
	input wire[4:0] i0, i1,
	input wire control,
	output reg[4:0] out
	);

	always @ (i0, i1, control) begin
		case(control)
			0: out <= i0;
			1: out <= i1;
			default: out <= 5'bx;
		endcase
	end
endmodule

module multiplexorALUSrc(
	input wire[31:0] i0, i1,
	input wire control,
	output reg[31:0] out
	);

	always @ (i0, i1, control) begin
		case(control)
			0: out <= i0;
			1: out <= i1;
			default: out <= 32'bx;
		endcase
	end
endmodule


module multiplexorMemtoReg(
	input wire[31:0] i0, i1,
	input wire control,
	output reg[31:0] out
	);

	always @ (i0, i1, control) begin
		case(control)
			0: out <= i0;
			1: out <= i1;
			default: out <= 32'bx;
		endcase
	end
endmodule

module multiplexorPCSrc(
	input wire[31:0] i0, i1,
	input wire control,
	output reg[31:0] out
	);

	always @ (i0, i1, control) begin
		case(control)
			0: out <= i0;
			1: out <= i1;
			default out <= 32'bx;
		endcase
	end
endmodule

module mainALUForwardingMUX(
	input wire[31:0] i0, i1, i2,
	input wire[1:0]  control,
	output reg[31:0] out
	);

	always @* begin
		case(control)
			0: out <= i0;
			1: out <= i1;
			2: out <= i2;
			default out <= 32'bx;
		endcase
	end
endmodule

module controlMUX(
	input wire control,
	input wire regDstIn,
	input wire ALUSrcIn,
	input wire memtoRegIn,
	input wire regWriteIn,
	input wire memReadIn,
	input wire memWriteIn,
	input wire branchIn,
	input wire ALUOp1In,
	input wire ALUOp0In,
	output reg regDstOut,
	output reg ALUSrcOut,
	output reg memtoRegOut,
	output reg regWriteOut,
	output reg memReadOut,
	output reg memWriteOut,
	output reg branchOut,
	output reg ALUOp1Out,
	output reg ALUOp0Out
);
	always @* begin
		case(control)
			0:	begin
					regDstOut <= regDstIn;
					ALUSrcOut <= ALUSrcIn;
					memtoRegOut <= memtoRegIn;
					regWriteOut <= regWriteIn;
					memReadOut <= memReadIn;
					memWriteOut = memWriteIn;
					branchOut <= branchIn;
					ALUOp1Out <= ALUOp1In;
					ALUOp0Out <= ALUOp0In;
				end
			default:
				begin
					regDstOut <= 0;
					ALUSrcOut <= 0;
					memtoRegOut <= 0;
					regWriteOut <= 0;
					memReadOut <= 0;
					memWriteOut <= 0;
					branchOut <= 0;
					ALUOp1Out <= 0;
					ALUOp0Out <= 0;
				end
		endcase
	end
endmodule
