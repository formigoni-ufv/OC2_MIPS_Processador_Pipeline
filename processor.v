//Ruan Evangelista Formigoni - 2661
//Vitor Guerra Veloso - 2658

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
// `include "tb_forwarding_unit.v"
// `include "tb_hazard_detection_unit.v"
// `include "forwarding_unit.v"
// `include "hazard_detection_unit.v"

module processor ();

	reg[31:0] i;
	//clk
	reg clk;
	//PC WIRES
	wire [31:0] PCSrc; //Proximo endereco do PC
	wire[31:0] PCOutput; //Saida do PC

	//ALU(PC + 4) WIRES
	wire[31:0] ALUPCPlus4Output; //Saida da ALU PC + 4
	//INSMEM WIRES
	wire[31:0] instruction; //Instrucao que sai da Instruction Memory
	//RegisterFile WIRES
	wire[31:0] readData1; //Dado lido de RS no register file
	wire[31:0] readData2; //Dado lido de RT no register file
	//Sign Extend WIRE
	wire[31:0] signExtendOutput; //Saida do modulo de extensao do sinal

	//Sinais do Controle
	wire CSignal_RegDst;
	wire CSignal_ALUSrc;
	wire CSignal_MemtoReg;
	wire CSignal_RegWrite;
	wire CSignal_MemRead;
	wire CSignal_MemWrite;
	wire CSignal_Branch;
	wire[1:0] CSignal_ALUOp;

	//Sinais de controle da pipeline ID/EX
	wire PIPE_IDEX_OUT_CSignal_EX_ALUSrc;
	wire[1:0] PIPE_IDEX_OUT_CSignal_EX_ALUOp;
	wire PIPE_IDEX_OUT_CSignal_EX_RegDst;
	wire PIPE_IDEX_OUT_CSignal_MEM_Branch;
	wire PIPE_IDEX_OUT_CSignal_MEM_MRead;
	wire PIPE_IDEX_OUT_CSignal_MEM_MWrite;
	wire PIPE_IDEX_OUT_CSignal_WB_RegWrite;
	wire PIPE_IDEX_OUT_CSignal_WB_MemtoReg;

	/*****************************************PIPELINED**********************************/
	reg resetManual; //TODO substituir pelo reset chaveado
	reg[31:0] pcInManual;  //TODO remover pelo PCIn do mux, esta variavel garante dont cares

	//Fios de saida da pipeline IF/ID
	wire[31:0] PIPE_IFID_ALUPCPlus4Output;
	wire[31:0] PIPE_IFID_Instruction;

	//Fios de saida da pipeline ID/EX
	wire[31:0] PIPE_IDEX_OUT_ALUPCPlus4Output;
	wire[31:0] PIPE_IDEX_OUT_ReadData1;
	wire[31:0] PIPE_IDEX_OUT_ReadData2;
	wire[31:0] PIPE_IDEX_OUT_SignExt;
	wire[4:0] PIPE_IDEX_OUT_RT;
	wire[4:0] PIPE_IDEX_OUT_RD;

	//Fios de saida da pipeline MEM/WB
	wire       PIPE_MEMWB_CSignal_RegWrite;
	wire[4:0]  PIPE_MEMWB_RegDstOutput;
	wire[31:0] PIPE_MEMWB_MemtoRegMUXOutput;

	/******************************Instruction Fetch Stage********************************/
	programcounter PC(.clock(clk), .in(pcInManual), .out(PCOutput), .reset(resetManual) );

	instructionmemory InstructionMemory(.addr(PCOutput), .instruction(instruction) );

	arithmeticlogicunit ALUPCPlus4(.A(PCOutput), .B(32'b100), .OP(4'b10), .OUT(ALUPCPlus4Output) );

	PIPE_IF_ID IF_ID(.clk(clk), .PIPEIN_PCPlus4(ALUPCPlus4Output), .PIPEIN_InsMemory(instruction), .PIPEOUT_PCPlus4(PIPE_IFID_ALUPCPlus4Output), .PIPEOUT_InsMemory(PIPE_IFID_Instruction) );

	/*************Instruction Decode Stage***************/
	maincontrolunit ControlUnit(.op(PIPE_IFID_Instruction[31:26]), .regDst(CSignal_RegDst), .ALUSrc(CSignal_ALUSrc), .memtoReg(CSignal_MemtoReg), .regWrite(CSignal_RegWrite), .memRead(CSignal_MemRead), .memWrite(CSignal_MemWrite), .branch(CSignal_Branch), .ALUOp1(CSignal_ALUOp[1]), .ALUOp0(CSignal_ALUOp[0]) );

	registerfile RegisterFile(.clk(clk), .reg1addr(PIPE_IFID_Instruction[25:21]), .reg2addr(PIPE_IFID_Instruction[20:16]), .writeRegister(PIPE_MEMWB_RegDstOutput), .data(PIPE_MEMWB_MemtoRegMUXOutput), .regWrite(PIPE_MEMWB_CSignal_RegWrite), .reg1content(readData1),	.reg2content(readData2) );

	signext SignExt(.i0(PIPE_IFID_Instruction[15:0]), .out(signExtendOutput) );


	PIPE_ID_EX ID_EX(
	.clk(clk),
	.PIPEIN_PCPlus4(PIPE_IFID_ALUPCPlus4Output),
	.PIPEIN_ReadData1(readData1),
	.PIPEIN_ReadData2(readData2),
	.PIPEIN_SignExt(signExtendOutput),
	.PIPEIN_RT(PIPE_IFID_Instruction[20:16]),
	.PIPEIN_RD(PIPE_IFID_Instruction[15:11]),

	.PIPEIN_EX_ALUSrc(CSignal_ALUSrc),
	.PIPEIN_EX_ALUOp(CSignal_ALUOp),
	.PIPEIN_EX_RegDst(CSignal_RegDst),
	.PIPEIN_MEM_Branch(CSignal_Branch),
	.PIPEIN_MEM_MRead(CSignal_MemRead),
	.PIPEIN_MEM_MWrite(CSignal_MemWrite),
	.PIPEIN_WB_RegWrite(CSignal_RegWrite),
	.PIPEIN_WB_MemtoReg(CSignal_MemtoReg),

	.PIPEOUT_PCPlus4(PIPE_IDEX_OUT_ALUPCPlus4Output),
	.PIPEOUT_ReadData1(PIPE_IDEX_OUT_ReadData1),
	.PIPEOUT_ReadData2(PIPE_IDEX_OUT_ReadData2),
	.PIPEOUT_SignExt(PIPE_IDEX_OUT_SignExt),
	.PIPEOUT_RT(PIPE_IDEX_OUT_RT),
	.PIPEOUT_RD(PIPE_IDEX_OUT_RD),

	.PIPEOUT_EX_ALUSrc(PIPE_IDEX_OUT_CSignal_EX_ALUSrc),
	.PIPEOUT_EX_ALUOp(PIPE_IDEX_OUT_CSignal_EX_ALUOp),
	.PIPEOUT_EX_RegDst(PIPE_IDEX_OUT_CSignal_EX_RegDst),
	.PIPEOUT_MEM_Branch(PIPE_IDEX_OUT_CSignal_MEM_Branch),
	.PIPEOUT_MEM_MRead(PIPE_IDEX_OUT_CSignal_MEM_MRead),
	.PIPEOUT_MEM_MWrite(PIPE_IDEX_OUT_CSignal_MEM_MWrite),
	.PIPEOUT_WB_RegWrite(PIPE_IDEX_OUT_CSignal_WB_RegWrite),
	.PIPEOUT_WB_MemtoReg(PIPE_IDEX_OUT_CSignal_WB_MemtoReg)

	);



	/******************************************************************************************Single Cycle**************************************************************************************************************************/
	//Modules Instantiation
	// programcounter pc(.clock(clk), .in(pcIn), .out(pcOut), .reset(reset) );
	//
	// arithmeticlogicunit aluPCPlus4(.A(pcOut), .B(32'b100), .OP(4'b10), .OUT(aluPCPlus4Out) );
	//
	// instructionmemory insmem(.addr(pcOut), .instruction(insMemOut) );
	//
	// maincontrolunit controlUnit(.op(insMemOut[31:26]), .regDst(regDst), .ALUSrc(aluSrc), .memtoReg(MemtoReg), .regWrite(regWrite), .memRead(memRead), .memWrite(memWrite), .branch(branch), .ALUOp1(ALUOp[1]), .ALUOp0(ALUOp[0]) );
	//
	// multiplexorRegDst regfilemux(.i0(insMemOut[20:16]), .i1(insMemOut[15:11]), .control(regDst), .out(regDstOut) );
	//
	// signext ext(.i0(insMemOut[15:0]), .out(extOut) );
	//
	// registerfile regfile(.clk(clk), .reg1addr(insMemOut[25:21]), .reg2addr(insMemOut[20:16]), .regWaddr(regDstOut), .data(memtoRegOut), .regWrite(regWrite), .reg1content(reg1content),	.reg2content(reg2content) );
	//
	// multiplexorALUSrc alumux(.i0(reg2content), .i1(extOut), .control(aluSrc), .out(aluSrcOut) );
	//
	// alu_control aluControl(.ALUOp(ALUOp), .funcCode(insMemOut[5:0]), .aluCtrlOut(aluCtrlOut) );
	//
	// arithmeticlogicunit mainAlu(.A(reg1content), .B(aluSrcOut), .OP(aluCtrlOut), .OUT(aluMainOut), .zero(zero) );
	//
	// shiftlogicalleft sll(.i0(extOut), .out(sllOut));
	//
	// arithmeticlogicunit branchAlu(.A(aluPCPlus4Out), .B(sllOut), .OP(4'b10), .OUT(branchAluOut) );
	//
	// multiplexorPCSrc branchmux(.i0(aluPCPlus4Out), .i1(branchAluOut), .control(andGateOut), .out(pcIn) );
	//
	// andgate gate(.i0(branch), .i1(zero), .out(andGateOut) );
	//
	// datamemory datamem(.clk(clk), .addr(aluMainOut), .writeData(reg2content), .memRead(memRead), .memWrite(memWrite), .readData(dataMemOut) );
	//
	// multiplexorMemtoReg dataMemMux(.i0(aluMainOut), .i1(dataMemOut), .control(MemtoReg), .out(memtoRegOut) );
	/****************************************************************************************************************************************************************************************************************/




	/*************************************************************************************************PIPELINED*****************************************************************************************************/


	//Testing

	always@(clk) begin
	#10
		$display("\n");
		$display("Clock               :          Clock       | %b", clk);
		$display("Stage IF:           :           PC         | %b", PCOutput);
		$display("Stage IF:           :         PC + 4       | %b", ALUPCPlus4Output);
		$display("Stage IF:           :       Instruction    | %b\n", instruction);

		$display("Stage ID: IF|ID OUT :         PC + 4       | %b", PIPE_IFID_ALUPCPlus4Output);
		$display("Stage ID: IF|ID OUT :       Instruction    | %b", PIPE_IFID_Instruction);
		$display("Stage ID:           :       Read Data 1    | %b", readData1);
		$display("Stage ID:           :       Read Data 2    | %b", readData2);
		$display("Stage ID:           :       Sign Extend    | %b", signExtendOutput);
		$display("Stage ID:           :            RT        | %b", PIPE_IFID_Instruction[20:16]);
		$display("Stage ID:           :            RD        | %b", PIPE_IFID_Instruction[15:11]);
		$display("Stage ID:           :    C_Signal RegDst   | %b", CSignal_RegDst);
		$display("Stage ID:           :    C_Signal ALUSrc   | %b", CSignal_ALUSrc);
		$display("Stage ID:           :    C_Signal MemtoReg | %b", CSignal_MemtoReg);
		$display("Stage ID:           :    C_Signal RegWrite | %b", CSignal_RegWrite);
		$display("Stage ID:           :    C_Signal MemRead  | %b", CSignal_MemRead);
		$display("Stage ID:           :    C_Signal MemWrite | %b", CSignal_MemWrite);
		$display("Stage ID:           :    C_Signal Branch   | %b", CSignal_Branch);
		$display("Stage ID:           :    C_Signal ALUOp    | %b", CSignal_ALUOp);

		$display("Stage EX:           :        PCPlus4       | %b", PIPE_IDEX_OUT_ALUPCPlus4Output);
		$display("Stage EX:           :       ReadData1      | %b", PIPE_IDEX_OUT_ReadData1);
		$display("Stage EX:           :       ReadData2      | %b", PIPE_IDEX_OUT_ReadData2);
		$display("Stage EX:           :       SignExtend     | %b", PIPE_IDEX_OUT_SignExt);
		$display("Stage EX:           :           RT         | %b", PIPE_IDEX_OUT_RT);
		$display("Stage EX:           :           RD         | %b", PIPE_IDEX_OUT_RD);
		$display("Stage EX:           :   C_Signal RegDst    | %b", PIPE_IDEX_OUT_CSignal_EX_RegDst);
		$display("Stage EX:           :   C_Signal ALUSrc    | %b", PIPE_IDEX_OUT_CSignal_EX_ALUSrc);
		$display("Stage EX:           :   C_Signal MemtoReg  | %b", PIPE_IDEX_OUT_CSignal_WB_MemtoReg);
		$display("Stage EX:           :   C_Signal RegWrite  | %b", PIPE_IDEX_OUT_CSignal_WB_RegWrite);
		$display("Stage EX:           :   C_Signal MemRead   | %b", PIPE_IDEX_OUT_CSignal_MEM_MRead);
		$display("Stage EX:           :   C_Signal MemWrite  | %b", PIPE_IDEX_OUT_CSignal_MEM_MWrite);
		$display("Stage EX:           :   C_Signal Branch    | %b", PIPE_IDEX_OUT_CSignal_MEM_Branch);
		$display("Stage EX:           :   C_Signal ALUOp     | %b", PIPE_IDEX_OUT_CSignal_EX_ALUOp);


		// $display("Stage 3: ID|EX OUT  :           RS         |");
		// $display("Stage 3: ID|EX OUT  :           RT         |");
		// $display("Stage 3: MEM|WB IN  :          ALU         |");
		// $display("Stage 3: MEM|WB IN  :           RT         |\n");
		// $display("Stage 4: MEM|WB OUT :          ALU         |");
		// $display("Stage 4: MEM|WB OUT :           RT         |");
		// $display("Stage 4: EX|MEM IN  :        MEM DATA      |");
		// $display("Stage 4: EX|MEM IN  :          ALU         |\n");
		// $display("Stage 5: WB MUX OUT :          WB          |");
	end


	initial begin
	//Inicializando PC com 0
	#20 resetManual = 1;
	#20 clk = 1;
	#20 resetManual = 0;

	//Começando a Endereçar as instruções
	#20 clk = 0;
	#20 clk = 1;
	#20 clk = 0;
	#20 clk = 1;
	#20 clk = 0;
	end

endmodule
