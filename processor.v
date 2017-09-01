//Ruan Evangelista Formigoni - 2661
//Vitor Guerra Veloso - 2658
`timescale 1ns / 10ps
`include "alu_control.v"
`include "andgate.v"
`include "arithmeticlogicunit.v"
`include "datamemory.v"
`include "decoder.v"
`include "instructionmemory.v"
`include "maincontrolunit.v"
`include "multiplexor.v"
`include "programcounter.v"
`include "registerfile.v"
`include "signext.v"
`include "sll.v"
`include "pipelinestages.v"
`include "forwarding_unit.v"
`include "hazard_detection_unit.v"

module processor (
    output[1:0] CSignal_ForwardingMUX_ALUi0,
    output[1:0] CSignal_ForwardingMUX_ALUi1,
    output[31:0] PCOutput,
    output[31:0] ALUPCPlus4Output,
    output[31:0] instruction,
    output PCWrite,
    output IF_ID_Write,
    output MUX_ID_EX_Write,
    output[31:0] PIPE_IFID_ALUPCPlus4Output,
    output[31:0] PIPE_IFID_Instruction,
    output[31:0] readData1,
    output[31:0] readData2,
    output[31:0] signExtendOutput,
    output CSignal_RegDst,
    output CSignal_ALUSrc,
    output CSignal_MemtoReg,
    output CSignal_RegWrite,
    output CSignal_MemRead,
    output CSignal_MemWrite,
    output CSignal_Branch,
    output[1:0] CSignal_ALUOp,
    output[31:0] PIPE_IDEX_OUT_ALUPCPlus4Output,
    output[31:0] PIPE_IDEX_OUT_ReadData1,
    output[31:0] PIPE_IDEX_OUT_ReadData2,
    output[31:0] PIPE_IDEX_OUT_SignExt,
    output[4:0] PIPE_IDEX_OUT_RS,
    output[4:0] PIPE_IDEX_OUT_RT,
    output[4:0] PIPE_IDEX_OUT_RD,
    output[31:0] sllOutput,
    output[31:0] branchALUOutput,
    output[31:0] ALUSrcOutput,
    output[31:0] forwardingMUXALUi0,
    output[31:0] forwardingMUXALUi1,
    output[31:0] mainALUOutput,
    output zero,
    output[3:0] ALUControlOutput,
    output[4:0] regDstOutput,
    output PIPE_IDEX_OUT_CSignal_EX_RegDst,
    output PIPE_IDEX_OUT_CSignal_EX_ALUSrc,
    output PIPE_IDEX_OUT_CSignal_WB_MemtoReg,
    output PIPE_IDEX_OUT_CSignal_WB_RegWrite,
    output PIPE_IDEX_OUT_CSignal_MEM_MRead,
    output PIPE_IDEX_OUT_CSignal_MEM_MWrite,
    output PIPE_IDEX_OUT_CSignal_MEM_Branch,
    output[1:0] PIPE_IDEX_OUT_CSignal_EX_ALUOp,
    output PIPE_EXMEM_OUT_CSignal_WB_MemtoReg,
    output PIPE_EXMEM_OUT_CSignal_WB_RegWrite,
    output PIPE_EXMEM_OUT_CSignal_MEM_MRead,
    output PIPE_EXMEM_OUT_CSignal_MEM_MWrite,
    output PIPE_EXMEM_OUT_CSignal_MEM_Branch,
    output[31:0] PIPE_EXMEM_OUT_BranchALUOutput,
    output PIPE_EXMEM_OUT_Zero,
    output[31:0] PIPE_EXMEM_OUT_MainALUOutput,
    output[31:0] PIPE_EXMEM_OUT_ReadData2,
    output[4:0] PIPE_EXMEM_OUT_RegDstOutput,
    output[31:0] dataMemoryOutput,
    output branchGateOutput,
    output PIPE_MEMWB_OUT_CSignal_MemtoReg,
    output PIPE_MEMWB_OUT_CSignal_RegWrite,
    output[31:0] PIPE_MEMWB_DataMemoryOutput,
    output[31:0] PIPE_MEMWB_MainALUOutput,
    output[4:0] PIPE_MEMWB_RegDstOutput,
    output[31:0] memtoRegOutput,
    input clk

);

	reg[31:0] i;

	//Fios da saída ou entrada de módulos, não pipelines
	//IF STAGE
	wire[31:0] PCSrcInput; //Proximo endereco do PC
	wire[31:0] PCOutput; //Saida do PC
	wire[31:0] ALUPCPlus4Output; //Saida da ALU PC + 4
	wire[31:0] instruction; //Instrucao que sai da Instruction Memory
	//ID STAGE
	wire       PCWrite;
	wire       IF_ID_Write;
	wire       MUX_ID_EX_Write;
	wire[31:0] readData1; //Dado lido de RS no register file
	wire[31:0] readData2; //Dado lido de RT no register file
	wire[31:0] signExtendOutput; //Saida do modulo de extensao do sinal
	//EX STAGE
	wire[31:0] sllOutput;
	wire[31:0] branchALUOutput;
	wire[31:0] forwardingMUXALUi0;
	wire[31:0] forwardingMUXALUi1;
	wire[31:0] ALUSrcOutput;
	wire[31:0] mainALUOutput;
	wire zero;
	wire[3:0] ALUControlOutput;
	wire[4:0] regDstOutput;
	//MEM STAGE
	wire branchGateOutput;
	wire[31:0] dataMemoryOutput;
	//WB STAGE
	wire[31:0] memtoRegOutput;

	//Sinais do Controle
	wire CSignal_RegWrite;
	wire CSignal_RegDst; //Sinal para escolher o registrador destino (RT ou RD)
	wire CSignal_ALUSrc; //Sinal para escolher entre o valor com sinal extendido ou o valor contido em RT
	wire CSignal_MemtoReg; //Escolhe se o dado é escrito da memória ou da ALU
	wire CSignal_MemRead; //Habilita leitura memoria
	wire CSignal_MemWrite; //Habilida escrita na memoria
	wire CSignal_Branch; //Habilita o Branch
	wire[1:0] CSignal_ALUOp; //Escolhe a operação da ALU
	wire CSignal_RegWrite_HazardMUX;
	wire CSignal_RegDst_HazardMUX;
	wire CSignal_ALUSrc_HazardMUX;
	wire CSignal_MemtoReg_HazardMUX;
	wire CSignal_MemRead_HazardMUX;
	wire CSignal_MemWrite_HazardMUX;
	wire CSignal_Branch_HazardMUX;
	wire[1:0] CSignal_ALUOp_HazardMUX;
	wire[1:0] CSignal_ForwardingMUX_ALUi0;
	wire[1:0] CSignal_ForwardingMUX_ALUi1;

	/*****************************************PIPELINED**********************************/
	reg resetManual; //TODO substituir pelo reset chaveado
	reg[31:0] pcInManual;  //TODO remover pelo PCSrcInput do mux, esta variavel garante dont cares

	//Fios de saida da pipeline IF/ID
	wire[31:0] PIPE_IFID_ALUPCPlus4Output;
	wire[31:0] PIPE_IFID_Instruction;

	//Fios de saida da pipeline ID/EX
	wire[31:0] PIPE_IDEX_OUT_ALUPCPlus4Output;
	wire[31:0] PIPE_IDEX_OUT_ReadData1;
	wire[31:0] PIPE_IDEX_OUT_ReadData2;
	wire[31:0] PIPE_IDEX_OUT_SignExt;
	wire[4:0] PIPE_IDEX_OUT_RS;
	wire[4:0] PIPE_IDEX_OUT_RT;
	wire[4:0] PIPE_IDEX_OUT_RD;
	//Sinais de controle da pipeline ID/EX
	wire PIPE_IDEX_OUT_CSignal_EX_ALUSrc;
	wire[1:0] PIPE_IDEX_OUT_CSignal_EX_ALUOp;
	wire PIPE_IDEX_OUT_CSignal_EX_RegDst;
	wire PIPE_IDEX_OUT_CSignal_MEM_Branch;
	wire PIPE_IDEX_OUT_CSignal_MEM_MRead;
	wire PIPE_IDEX_OUT_CSignal_MEM_MWrite;
	wire PIPE_IDEX_OUT_CSignal_WB_RegWrite;
	wire PIPE_IDEX_OUT_CSignal_WB_MemtoReg;

	//Fios de saida da pipeline EX/MEM
	wire[31:0] PIPE_EXMEM_OUT_BranchALUOutput;
	wire       PIPE_EXMEM_OUT_Zero;
	wire[31:0] PIPE_EXMEM_OUT_MainALUOutput;
	wire[31:0] PIPE_EXMEM_OUT_ReadData2;
	wire[4:0]  PIPE_EXMEM_OUT_RegDstOutput;
	//Sinais de controle da pipeline EX/MEM
	wire PIPE_EXMEM_OUT_CSignal_MEM_Branch;
	wire PIPE_EXMEM_OUT_CSignal_MEM_MRead;
	wire PIPE_EXMEM_OUT_CSignal_MEM_MWrite;
	wire PIPE_EXMEM_OUT_CSignal_WB_RegWrite;
	wire PIPE_EXMEM_OUT_CSignal_WB_MemtoReg;

	//Sinais de controle da pipeline MEM/WB
	wire       PIPE_MEMWB_OUT_CSignal_RegWrite;
	wire       PIPE_MEMWB_OUT_CSignal_MemtoReg;
	//Fios de saida da pipeline MEM/WB
	wire[31:0] PIPE_MEMWB_DataMemoryOutput;
	wire[31:0] PIPE_MEMWB_MainALUOutput;
	wire[4:0]  PIPE_MEMWB_RegDstOutput;

	/******************************Instruction Fetch Stage********************************/
	multiplexorPCSrc PCSrc(.i0(ALUPCPlus4Output), .i1(PIPE_EXMEM_OUT_BranchALUOutput), .control(branchGateOutput), .out(PCSrcInput));

	programcounter PC(.clock(clk), .PCWrite(PCWrite), .in(pcInManual), .out(PCOutput), .reset(resetManual) );

	instructionmemory InstructionMemory(.addr(PCOutput), .instruction(instruction) );

	arithmeticlogicunit ALUPCPlus4(.A(PCOutput), .B(32'b100), .OP(4'b10), .OUT(ALUPCPlus4Output) );

	PIPE_IF_ID IF_ID(.clk(clk), .IF_ID_Write(IF_ID_Write), .PIPEIN_PCPlus4(ALUPCPlus4Output), .PIPEIN_InsMemory(instruction), .PIPEOUT_PCPlus4(PIPE_IFID_ALUPCPlus4Output), .PIPEOUT_InsMemory(PIPE_IFID_Instruction) );

	/******************************Instruction Decode Stage******************************/
	hazard_detection_unit hazardUnit(
		.clk(clk),
		.ID_EX_MemRead(PIPE_IDEX_OUT_CSignal_MEM_MRead),
		.ID_EX_RegisterRt(PIPE_IDEX_OUT_RT),
		.IF_ID_RegisterRs(PIPE_IFID_Instruction[25:21]),
		.IF_ID_RegisterRt(PIPE_IFID_Instruction[20:16]),
		.PCWrite(PCWrite),
		.IF_ID_Write(IF_ID_Write),
		.MUX_ID_EX_Write(MUX_ID_EX_Write)
	);

	maincontrolunit ControlUnit(.op(PIPE_IFID_Instruction[31:26]), .regDst(CSignal_RegDst), .ALUSrc(CSignal_ALUSrc), .memtoReg(CSignal_MemtoReg), .regWrite(CSignal_RegWrite), .memRead(CSignal_MemRead), .memWrite(CSignal_MemWrite), .branch(CSignal_Branch), .ALUOp1(CSignal_ALUOp[1]), .ALUOp0(CSignal_ALUOp[0]) );

	controlMUX hazardMUX(
		.control(MUX_ID_EX_Write),
		.regDstIn(CSignal_RegDst),
		.ALUSrcIn(CSignal_ALUSrc),
		.memtoRegIn(CSignal_MemtoReg),
		.regWriteIn(CSignal_RegWrite),
		.memReadIn(CSignal_MemRead),
		.memWriteIn(CSignal_MemWrite),
		.branchIn(CSignal_Branch),
		.ALUOp1In(CSignal_ALUOp[1]),
		.ALUOp0In(CSignal_ALUOp[0]),
		.regDstOut(CSignal_RegDst_HazardMUX),
		.ALUSrcOut(CSignal_ALUSrc_HazardMUX),
		.memtoRegOut(CSignal_MemtoReg_HazardMUX),
		.regWriteOut(CSignal_RegWrite_HazardMUX),
		.memReadOut(CSignal_MemRead_HazardMUX),
		.memWriteOut(CSignal_MemWrite_HazardMUX),
		.branchOut(CSignal_Branch_HazardMUX),
		.ALUOp1Out(CSignal_ALUOp_HazardMUX[1]),
		.ALUOp0Out(CSignal_ALUOp_HazardMUX[0])
	);

	registerfile RegisterFile(.clk(clk), .reg1addr(PIPE_IFID_Instruction[25:21]), .reg2addr(PIPE_IFID_Instruction[20:16]), .writeRegister(PIPE_MEMWB_RegDstOutput), .writeData(memtoRegOutput), .regWrite(PIPE_MEMWB_OUT_CSignal_RegWrite), .reg1content(readData1),	.reg2content(readData2) );

	signext SignExt(.i0(PIPE_IFID_Instruction[15:0]), .out(signExtendOutput) );


	PIPE_ID_EX ID_EX(
		.clk(clk),
		//DATA VARS IN
		.PIPEIN_PCPlus4(PIPE_IFID_ALUPCPlus4Output),
		.PIPEIN_ReadData1(readData1),
		.PIPEIN_ReadData2(readData2),
		.PIPEIN_SignExt(signExtendOutput),
		.PIPEIN_RS(PIPE_IFID_Instruction[25:21]),
		.PIPEIN_RT(PIPE_IFID_Instruction[20:16]),
		.PIPEIN_RD(PIPE_IFID_Instruction[15:11]),

		//CONTROL VARS IN
		.PIPEIN_EX_ALUSrc(CSignal_ALUSrc_HazardMUX),
		.PIPEIN_EX_ALUOp(CSignal_ALUOp_HazardMUX),
		.PIPEIN_EX_RegDst(CSignal_RegDst_HazardMUX),
		.PIPEIN_MEM_Branch(CSignal_Branch_HazardMUX),
		.PIPEIN_MEM_MRead(CSignal_MemRead_HazardMUX),
		.PIPEIN_MEM_MWrite(CSignal_MemWrite_HazardMUX),
		.PIPEIN_WB_RegWrite(CSignal_RegWrite_HazardMUX),
		.PIPEIN_WB_MemtoReg(CSignal_MemtoReg_HazardMUX),

		//DATA VARS OUT
		.PIPEOUT_PCPlus4(PIPE_IDEX_OUT_ALUPCPlus4Output),
		.PIPEOUT_ReadData1(PIPE_IDEX_OUT_ReadData1),
		.PIPEOUT_ReadData2(PIPE_IDEX_OUT_ReadData2),
		.PIPEOUT_SignExt(PIPE_IDEX_OUT_SignExt),
		.PIPEOUT_RS(PIPE_IDEX_OUT_RS),
		.PIPEOUT_RT(PIPE_IDEX_OUT_RT),
		.PIPEOUT_RD(PIPE_IDEX_OUT_RD),

		//CONTROL VARS OUT
		.PIPEOUT_EX_ALUSrc(PIPE_IDEX_OUT_CSignal_EX_ALUSrc),
		.PIPEOUT_EX_ALUOp(PIPE_IDEX_OUT_CSignal_EX_ALUOp),
		.PIPEOUT_EX_RegDst(PIPE_IDEX_OUT_CSignal_EX_RegDst),
		.PIPEOUT_MEM_Branch(PIPE_IDEX_OUT_CSignal_MEM_Branch),
		.PIPEOUT_MEM_MRead(PIPE_IDEX_OUT_CSignal_MEM_MRead),
		.PIPEOUT_MEM_MWrite(PIPE_IDEX_OUT_CSignal_MEM_MWrite),
		.PIPEOUT_WB_RegWrite(PIPE_IDEX_OUT_CSignal_WB_RegWrite),
		.PIPEOUT_WB_MemtoReg(PIPE_IDEX_OUT_CSignal_WB_MemtoReg)

	);

	/******************************Instruction Execution Stage******************************/
	shiftlogicalleft SLL(.i0(PIPE_IDEX_OUT_SignExt), .out(sllOutput));

	arithmeticlogicunit branchALU(.A(PIPE_IDEX_OUT_ALUPCPlus4Output), .B(sllOutput), .OP(4'b10), .OUT(branchALUOutput) );

	mainALUForwardingMUX ALUi0(.i0(PIPE_IDEX_OUT_ReadData1), .i1(memtoRegOutput), .i2(PIPE_EXMEM_OUT_MainALUOutput), .control(CSignal_ForwardingMUX_ALUi0), .out(forwardingMUXALUi0) );

	mainALUForwardingMUX ALUi1(.i0(PIPE_IDEX_OUT_ReadData2), .i1(memtoRegOutput), .i2(PIPE_EXMEM_OUT_MainALUOutput), .control(CSignal_ForwardingMUX_ALUi1), .out(forwardingMUXALUi1) );

	multiplexorALUSrc ALUSrcMux(.i0(forwardingMUXALUi1), .i1(PIPE_IDEX_OUT_SignExt), .control(PIPE_IDEX_OUT_CSignal_EX_ALUSrc), .out(ALUSrcOutput) );

	arithmeticlogicunit mainALU(.A(forwardingMUXALUi0), .B(ALUSrcOutput), .OP(ALUControlOutput), .OUT(mainALUOutput), .zero(zero) );

	alu_control ALUControl(.ALUOp(PIPE_IDEX_OUT_CSignal_EX_ALUOp), .funcCode(PIPE_IDEX_OUT_SignExt[5:0]), .aluCtrlOut(ALUControlOutput) );

	multiplexorRegDst regDstMUX(.i0(PIPE_IDEX_OUT_RT), .i1(PIPE_IDEX_OUT_RD), .control(PIPE_IDEX_OUT_CSignal_EX_RegDst), .out(regDstOutput) );

	PIPE_EX_MEM EX_MEM(
		.clk(clk),

		//Control Vars IN
		.PIPEIN_MEM_Branch(PIPE_IDEX_OUT_CSignal_MEM_Branch),
		.PIPEIN_MEM_MRead(PIPE_IDEX_OUT_CSignal_MEM_MRead),
		.PIPEIN_MEM_MWrite(PIPE_IDEX_OUT_CSignal_MEM_MWrite),
		.PIPEIN_WB_RegWrite(PIPE_IDEX_OUT_CSignal_WB_RegWrite),
		.PIPEIN_WB_MemtoReg(PIPE_IDEX_OUT_CSignal_WB_MemtoReg),
		//Data Vars IN
		.PIPEIN_BranchALUOutput(branchALUOutput),
		.PIPEIN_Zero(zero),
		.PIPEIN_ALUResult(mainALUOutput),
		.PIPEIN_ReadData2(PIPE_IDEX_OUT_ReadData2),
		.PIPEIN_RegDstOutput(regDstOutput),

		//Control Vars OUT
		.PIPEOUT_MEM_Branch(PIPE_EXMEM_OUT_CSignal_MEM_Branch),
		.PIPEOUT_MEM_MRead(PIPE_EXMEM_OUT_CSignal_MEM_MRead),
		.PIPEOUT_MEM_MWrite(PIPE_EXMEM_OUT_CSignal_MEM_MWrite),
		.PIPEOUT_WB_RegWrite(PIPE_EXMEM_OUT_CSignal_WB_RegWrite),
		.PIPEOUT_WB_MemtoReg(PIPE_EXMEM_OUT_CSignal_WB_MemtoReg),
		//Data Vars OUT
		.PIPEOUT_BranchALUOutput(PIPE_EXMEM_OUT_BranchALUOutput),
		.PIPEOUT_Zero(PIPE_EXMEM_OUT_Zero),
		.PIPEOUT_ALUResult(PIPE_EXMEM_OUT_MainALUOutput),
		.PIPEOUT_ReadData2(PIPE_EXMEM_OUT_ReadData2),
		.PIPEOUT_RegDstOutput(PIPE_EXMEM_OUT_RegDstOutput)
	);

	forwarding_unit ForwardingUnit(
		.clk(clk),
		.EX_MEM_RegWrite(PIPE_EXMEM_OUT_CSignal_WB_RegWrite),
		.MEM_WB_RegWrite(PIPE_MEMWB_OUT_CSignal_RegWrite),
		.ID_EX_Rs(PIPE_IDEX_OUT_RS),
		.ID_EX_Rt(PIPE_IDEX_OUT_RT),
		.EX_MEM_Rd(PIPE_EXMEM_OUT_RegDstOutput),
		.MEM_WB_Rd(PIPE_MEMWB_RegDstOutput),
		.ForwardA(CSignal_ForwardingMUX_ALUi0),
		.ForwardB(CSignal_ForwardingMUX_ALUi1)
	);

	/******************************Memory Stage******************************/
	andgate gate(.i0(PIPE_EXMEM_OUT_CSignal_MEM_Branch), .i1(PIPE_EXMEM_OUT_Zero), .out(branchGateOutput) );

	datamemory datamem(.clk(clk), .addr(PIPE_EXMEM_OUT_MainALUOutput), .writeData(PIPE_EXMEM_OUT_ReadData2), .memRead(PIPE_EXMEM_OUT_CSignal_MEM_MRead), .memWrite(PIPE_EXMEM_OUT_CSignal_MEM_MWrite), .readData(dataMemoryOutput) );

	PIPE_MEM_WB MEM_WB(
		.clk(clk),

		//Control Vars IN
		.PIPEIN_WB_RegWrite(PIPE_EXMEM_OUT_CSignal_WB_RegWrite),
		.PIPEIN_WB_MemtoReg(PIPE_EXMEM_OUT_CSignal_WB_MemtoReg),
		//Data Vars IN
		.PIPEIN_DataMemoryOutput(dataMemoryOutput),
		.PIPEIN_MainALUOutput(PIPE_EXMEM_OUT_MainALUOutput),
		.PIPEIN_RegDstOutput(PIPE_EXMEM_OUT_RegDstOutput),

		//Control Vars OUT
		.PIPEOUT_WB_RegWrite(PIPE_MEMWB_OUT_CSignal_RegWrite),
		.PIPEOUT_WB_MemtoReg(PIPE_MEMWB_OUT_CSignal_MemtoReg),
		//Data Vars OUT
		.PIPEOUT_DataMemoryOutput(PIPE_MEMWB_DataMemoryOutput),
		.PIPEOUT_MainALUOutput(PIPE_MEMWB_MainALUOutput),
		.PIPEOUT_RegDstOutput(PIPE_MEMWB_RegDstOutput)
	);

	/******************************Write Back Stage******************************/
	multiplexorMemtoReg MemtoReg(.i0(PIPE_MEMWB_MainALUOutput), .i1(PIPE_MEMWB_DataMemoryOutput), .control(PIPE_MEMWB_OUT_CSignal_MemtoReg), .out(memtoRegOutput) );

	/******************************************************************************************************************************************************************************************************/



endmodule
