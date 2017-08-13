module PIPE_IF_ID(input wire clk, input wire[31:0] PIPEIN_PCPlus4, input wire[31:0] PIPEIN_insMemory, output reg[31:0] PIPEOUT_PCPlus4, output reg[31:0] PIPEOUT_insMemory);

	//IF STAGE
	always@(posedge clk)begin
		PIPEIN_PCPlus4    <= PIPEOUT_PCPlus4;
		PIPEOUT_insMemory <= PIPEOUT_insMemory;
	end

	//ID STAGE
		//Ler os registradores de saida

endmodule



module PIPE_ID_EX(input wire clk, input wire[31:0] PIPEIN_PCPlus4, input wire[31:0] PIPEIN_readData1, input wire[31:0] PIPEIN_readData2, input wire[31:0] PIPEIN_signExt, output reg[31:0] PIPEOUT_PCPlus4, output reg[31:0] PIPEOUT_readData1, output reg[31:0] PIPEOUT_readData2, output reg[31:0] PIPEOUT_signExt);
	//ID STAGE
	always@(negedge clk)begin
		PIPEIN_PCPlus4   <= PIPEOUT_PCPlus4In;
		PIPEIN_readData1 <= PIPEOUT_readData1In;
		PIPEIN_readData2 <= PIPEOUT_readData2In;
		PIPEIN_signExt   <= PIPEOUT_signExtIn;
	end

	//EX STAGE
		//Ler os registradores de saida
endmodule



module PIPE_EX_MEM(input wire clk, input wire[31:0] PIPEIN_branchALU, input wire[31:0] PIPEIN_mainALUResult, input wire PIPEIN_zero, input wire[31:0] PIPEIN_readData2, output reg[31:0] PIPEOUT_branchALU, output reg[31:0] PIPEIN_mainALUResult, output reg PIPEOUT_zero, output reg[31:0] PIPEOUT_readData2);

	//EX STAGE
	always@(posedge clk)begin
		PIPEOUT_branchALU       <= PIPEIN_branchALU;
		PIPEOUT_mainALUResult   <= PIPEIN_mainALUResult;
		PIPEOUT_zero            <= PIPEIN_zero;
		PIPEOUT_readData2       <= PIPEIN_readData2;
	end

	//MEM STAGE
		//Ler os registradores de saida
endmodule



module PIPE_MEM_WB(input wire clk, input wire[31:0] PIPEIN_dataMemReadData, input wire[31:0] PIPEIN_mainALUResult, output reg[31:0] PIPEOUT_dataMemReadData, output reg[31:0] PIPEOUT_mainALUResult);

	//MEM STAGE
	always@(negedge clk)begin
		PIPEOUT_dataMemReadData <= PIPEIN_dataMemReadData;
		PIPEOUT_mainALUResult   <= PIPEINdataMemReadData;
	end

	//WB STAGE
		//Ler os registradores de saida
endmodule
