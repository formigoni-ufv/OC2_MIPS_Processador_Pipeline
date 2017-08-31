module PIPE_IF_ID(input wire clk, input wire IF_ID_Write, input wire[31:0] PIPEIN_PCPlus4, input wire[31:0] PIPEIN_InsMemory, output reg[31:0] PIPEOUT_PCPlus4, output reg[31:0] PIPEOUT_InsMemory);

	reg[31:0] PCPlus4;
	reg[31:0] InstructionMemory;

	//IF STAGE
	always@(clk)begin
		if(IF_ID_Write == 1)begin
			PIPEOUT_PCPlus4   <= PIPEIN_PCPlus4;
			PIPEOUT_InsMemory <= PIPEIN_InsMemory;
		end
	end

	//ID STAGE
		//Ler os registradores de saida

endmodule

module PIPE_ID_EX(
	input wire clk,

	//CONTROL REGISTERS
	//EX REGS
	input wire PIPEIN_EX_ALUSrc,
	input wire[1:0] PIPEIN_EX_ALUOp,
	input wire PIPEIN_EX_RegDst,
	output reg PIPEOUT_EX_ALUSrc,
	output reg[1:0] PIPEOUT_EX_ALUOp,
	output reg PIPEOUT_EX_RegDst,
	//MEM REGS
	input wire PIPEIN_MEM_Branch,
	input wire PIPEIN_MEM_MRead,
	input wire PIPEIN_MEM_MWrite,
	output reg PIPEOUT_MEM_Branch,
	output reg PIPEOUT_MEM_MRead,
	output reg PIPEOUT_MEM_MWrite,
	//WB REGS
	input wire PIPEIN_WB_RegWrite, input wire PIPEIN_WB_MemtoReg,
	output reg PIPEOUT_WB_RegWrite, output reg PIPEOUT_WB_MemtoReg,

	//INPUT ON ID FASE
	input wire[31:0] PIPEIN_PCPlus4, input wire[31:0] PIPEIN_ReadData1,
	input wire[31:0] PIPEIN_ReadData2, input wire[31:0] PIPEIN_SignExt,
	input wire[4:0] PIPEIN_RS, input wire[4:0] PIPEIN_RT, input wire[4:0] PIPEIN_RD,
	//OUTPUT ON EX FASE
	output reg[31:0] PIPEOUT_PCPlus4, output reg[31:0] PIPEOUT_ReadData1,
	output reg[31:0] PIPEOUT_ReadData2, output reg[31:0] PIPEOUT_SignExt,
	output reg[4:0] PIPEOUT_RS, output reg[4:0] PIPEOUT_RT, output reg[4:0] PIPEOUT_RD
	);

	//ID STAGE
	always@(clk)begin
		//CONTROL REGS
		//EX REGS
		PIPEOUT_EX_ALUSrc   <= PIPEIN_EX_ALUSrc;
		PIPEOUT_EX_ALUOp    <= PIPEIN_EX_ALUOp;
		PIPEOUT_EX_RegDst   <= PIPEIN_EX_RegDst;
		//MEM REGS
		PIPEOUT_MEM_Branch  <= PIPEIN_MEM_Branch;
		PIPEOUT_MEM_MRead   <= PIPEIN_MEM_MRead;
		PIPEOUT_MEM_MWrite  <= PIPEIN_MEM_MWrite;
		//WB_REGS
		PIPEOUT_WB_RegWrite <= PIPEIN_WB_RegWrite;
		PIPEOUT_WB_MemtoReg <= PIPEIN_WB_MemtoReg;

		//EX FASE REGS
		PIPEOUT_PCPlus4   <= PIPEIN_PCPlus4;
		PIPEOUT_ReadData1 <= PIPEIN_ReadData1;
		PIPEOUT_ReadData2 <= PIPEIN_ReadData2;
		PIPEOUT_SignExt   <= PIPEIN_SignExt;
		PIPEOUT_RS        <= PIPEIN_RS;
		PIPEOUT_RT        <= PIPEIN_RT;
		PIPEOUT_RD        <= PIPEIN_RD;
	end

	//EX STAGE
		//Ler os registradores de saida
endmodule

module PIPE_EX_MEM(
	input wire clk,

	//MEM REGS
	input wire PIPEIN_MEM_Branch,
	input wire PIPEIN_MEM_MRead,
	input wire PIPEIN_MEM_MWrite,
	output reg PIPEOUT_MEM_Branch,
	output reg PIPEOUT_MEM_MRead,
	output reg PIPEOUT_MEM_MWrite,
	//WB REGS
	input wire PIPEIN_WB_RegWrite,
	input wire PIPEIN_WB_MemtoReg,
	output reg PIPEOUT_WB_RegWrite,
	output reg PIPEOUT_WB_MemtoReg,

	//Input on EX fase
	input wire[31:0] PIPEIN_BranchALUOutput,
	input wire       PIPEIN_Zero,
	input wire[31:0] PIPEIN_ALUResult,
	input wire[31:0] PIPEIN_ReadData2,
	input wire[4:0]  PIPEIN_RegDstOutput,
	//Output on MEM FASE
	output reg[31:0] PIPEOUT_BranchALUOutput,
	output reg       PIPEOUT_Zero,
	output reg[31:0] PIPEOUT_ALUResult,
	output reg[31:0] PIPEOUT_ReadData2,
	output reg[4:0]  PIPEOUT_RegDstOutput
	);

	always@(clk)begin
		//MEM REGS
		PIPEOUT_MEM_Branch  <= PIPEIN_MEM_Branch;
		PIPEOUT_MEM_MRead   <= PIPEIN_MEM_MRead;
		PIPEOUT_MEM_MWrite  <= PIPEIN_MEM_MWrite;
		//WB_REGS
		PIPEOUT_WB_RegWrite <= PIPEIN_WB_RegWrite;
		PIPEOUT_WB_MemtoReg <= PIPEIN_WB_MemtoReg;

		//MEM Fase Regs
		PIPEOUT_BranchALUOutput <= PIPEIN_BranchALUOutput;
		PIPEOUT_Zero            <= PIPEIN_Zero;
		PIPEOUT_ALUResult       <= PIPEIN_ALUResult;
		PIPEOUT_ReadData2       <= PIPEIN_ReadData2;
		PIPEOUT_RegDstOutput    <= PIPEIN_RegDstOutput;
	end
endmodule

module PIPE_MEM_WB(
	input wire clk,
	/************CONTROL VARS***************/
	//WB REGS INPUT ON MEM
	input wire PIPEIN_WB_RegWrite,
	input wire PIPEIN_WB_MemtoReg,
	//WB REGS OUTPUT ON WB
	output reg PIPEOUT_WB_RegWrite,
	output reg PIPEOUT_WB_MemtoReg,

	/************DATA VARS***************/
	//INPUT ON MEM
	input wire[31:0] PIPEIN_DataMemoryOutput,
	input wire[31:0] PIPEIN_MainALUOutput,
	input wire[4:0] PIPEIN_RegDstOutput,
	//OUTPUT ON WB
	output reg[31:0] PIPEOUT_DataMemoryOutput,
	output reg[31:0] PIPEOUT_MainALUOutput,
	output reg[4:0] PIPEOUT_RegDstOutput
	);

	always@(clk)begin
		PIPEOUT_WB_RegWrite <= PIPEIN_WB_RegWrite;
		PIPEOUT_WB_MemtoReg <= PIPEIN_WB_MemtoReg;

		PIPEOUT_DataMemoryOutput <= PIPEIN_DataMemoryOutput;
		PIPEOUT_MainALUOutput    <= PIPEIN_MainALUOutput;
		PIPEOUT_RegDstOutput     <= PIPEIN_RegDstOutput;
	end
endmodule
