module PIPE_IF_ID(input wire clk, input wire[31:0] PIPEIN_PCPlus4, input wire[31:0] PIPEIN_InsMemory, output reg[31:0] PIPEOUT_PCPlus4, output reg[31:0] PIPEOUT_InsMemory);

	reg[31:0] PCPlus4;
	reg[31:0] InstructionMemory;

	//IF STAGE
	always@(posedge clk)begin
		PCPlus4           <= PIPEIN_PCPlus4;
		InstructionMemory <= PIPEIN_InsMemory;
	end

	//ID STAGE
		//Ler os registradores de saida
	always@(negedge clk)begin
		PIPEOUT_PCPlus4   <= PCPlus4;
		PIPEOUT_InsMemory <= InstructionMemory;
	end

endmodule



module PIPE_ID_EX(
	input wire clk,

	//CONTROL REGISTERS
	//EX REGS
	input wire PIPEIN_EX_ALUSrc, input wire[1:0] PIPEIN_EX_ALUOp,	input wire PIPEIN_EX_RegDst,
	output reg PIPEOUT_EX_ALUSrc, output reg[1:0] PIPEOUT_EX_ALUOp, output reg PIPEOUT_EX_RegDst,
	//MEM REGS
	input wire PIPEIN_MEM_Branch, input wire PIPEIN_MEM_MRead, input wire PIPEIN_MEM_MWrite,
	output reg PIPEOUT_MEM_Branch, output reg PIPEOUT_MEM_MRead, output reg PIPEOUT_MEM_MWrite,
	//WB REGS
	input wire PIPEIN_WB_RegWrite, input wire PIPEIN_WB_MemtoReg,
	output reg PIPEOUT_WB_RegWrite, output reg PIPEOUT_WB_MemtoReg,

	//ID FASE REGS
	input wire[31:0] PIPEIN_PCPlus4, input wire[31:0] PIPEIN_ReadData1,
	input wire[31:0] PIPEIN_ReadData2, input wire[31:0] PIPEIN_SignExt,
	input wire[4:0] PIPEIN_RT, input wire[4:0] PIPEIN_RD,
	output reg[31:0] PIPEOUT_PCPlus4, output reg[31:0] PIPEOUT_ReadData1,
	output reg[31:0] PIPEOUT_ReadData2, output reg[31:0] PIPEOUT_SignExt,
	output reg[4:0] PIPEOUT_RT, output reg[4:0] PIPEOUT_RD
	);

	reg PCPlus4;
	reg readData1;
	reg readData2;
	reg signExt;
	reg RT; //INS 25 - 21
	reg RD;	//INS 20 - 16
	reg EX_ALUSrc;
	reg[1:0] EX_ALUOp;
	reg EX_RegDst;
	reg MEM_Branch;
	reg MEM_MRead;
	reg MEM_MWrite;
	reg WB_RegWrite;
	reg WB_MemtoReg;

	//ID STAGE
	always@(posedge clk)begin
		//CONTROL REGS
		//EX REGS
		EX_ALUSrc   <= PIPEIN_EX_ALUSrc;
		EX_ALUOp    <= PIPEIN_EX_ALUOp;
		EX_RegDst   <= PIPEIN_EX_RegDst;
		//MEM REGS
		MEM_Branch  <= PIPEIN_MEM_Branch;
		MEM_MRead   <= PIPEIN_MEM_MRead;
		MEM_MWrite  <= PIPEIN_MEM_MWrite;
		//WB_REGS
		WB_RegWrite <= PIPEIN_WB_RegWrite;
		WB_MemtoReg <= PIPEIN_WB_MemtoReg;

		//ID FASE REGS
		PCPlus4   <= PIPEIN_PCPlus4;
		readData1 <= PIPEIN_ReadData1;
		readData2 <= PIPEIN_ReadData2;
		signExt   <= PIPEIN_SignExt;
		RT        <= PIPEIN_RT;
		RD        <= PIPEIN_RD;
	end

	//EX STAGE
		//Ler os registradores de saida
	always@(negedge clk)begin
		//CONTROL REGS
		//EX REGS
		PIPEOUT_EX_ALUSrc <= EX_ALUSrc;
		PIPEOUT_EX_ALUOp  <= EX_ALUOp;
		PIPEOUT_EX_RegDst <= EX_RegDst;
		//MEM REGS
		PIPEOUT_MEM_Branch <= MEM_Branch;
		PIPEOUT_MEM_MRead  <= MEM_MRead;
		PIPEOUT_MEM_MWrite <= MEM_MWrite;
		//WB REGS
		PIPEOUT_WB_RegWrite <= WB_RegWrite;
		PIPEOUT_WB_MemtoReg <= WB_MemtoReg;

		//EX FASE REGS
		PIPEOUT_PCPlus4   <= PCPlus4;
		PIPEOUT_ReadData1 <= readData1;
		PIPEOUT_ReadData2 <= readData2;
		PIPEOUT_SignExt   <= signExt;
		PIPEOUT_RT        <= RT;
		PIPEOUT_RD        <= RD;
	end

endmodule


module PIPE_EX_MEM(input wire clk, input wire[31:0] PIPEIN_branchALU, input wire[31:0] PIPEIN_mainALUResult, input wire PIPEIN_zero, input wire[31:0] PIPEIN_readData2, output reg[31:0] PIPEOUT_branchALU, output reg[31:0] PIPEOUT_mainALUResult, output reg PIPEOUT_zero, output reg[31:0] PIPEOUT_readData2);

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
		PIPEOUT_mainALUResult   <= PIPEIN_mainALUResult;
	end

	//WB STAGE
		//Ler os registradores de saida
endmodule
